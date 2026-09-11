//! A fake I2C bus the driver cannot distinguish from real hardware.
//!
//! [`MockI2c`] implements [`embedded_hal::i2c::I2c`] over a 256-byte register
//! file seeded with the MPU-6050's power-on defaults, so the driver talks to
//! it exactly as it would talk to the part. Three things make it useful for
//! testing rather than just for compiling:
//!
//! - **A transaction log.** Every bus operation is recorded in order as a
//!   [`Transaction`], so tests assert on the traffic the driver actually
//!   produced — that a six-byte read was one burst and not six, that the
//!   sleep bit was cleared before the clock was selected — rather than on
//!   internal state.
//! - **Device behaviour that matters.** The register pointer auto-increments
//!   the way the chip's does. `DEVICE_RESET` self-clears after a configurable
//!   number of polls. `INT_STATUS` clears when read. `FIFO_COUNT_L` is a shadow
//!   that only updates when `FIFO_COUNT_H` is read. `FIFO_RESET` is ignored
//!   unless `FIFO_EN` is 0. Each of these is a documented trap in the
//!   datasheet, and each is reproduced here so a driver that falls into it
//!   fails a test instead of failing in the field.
//! - **Fault injection.** A [`Fault`] can NACK a chosen transaction, corrupt
//!   a byte in a burst read, report the wrong device identity, hold every
//!   line high the way a bus with missing pull-ups does, or overflow the FIFO
//!   with the byte misalignment that a real overflow causes.
//!
//! The mock is the only part of this crate that uses `std`. It is compiled
//! only under the `mock` feature.

use embedded_hal::i2c::{
    ErrorKind, ErrorType, I2c, NoAcknowledgeSource, Operation, SevenBitAddress,
};
use std::collections::VecDeque;
use std::vec::Vec;

use crate::registers::{
    ADDR_AD0_LOW, FIFO_CAPACITY_BYTES, FIFO_EN_ACCEL, FIFO_EN_TEMP, FIFO_EN_XG, FIFO_EN_YG,
    FIFO_EN_ZG, INT_FIFO_OFLOW, PWR_MGMT_1_DEVICE_RESET, PWR_MGMT_1_SLEEP, REG_FIFO_COUNT_H,
    REG_FIFO_COUNT_L, REG_FIFO_EN, REG_FIFO_R_W, REG_INT_STATUS, REG_PWR_MGMT_1, REG_USER_CTRL,
    REG_WHO_AM_I, USER_CTRL_FIFO_EN, USER_CTRL_FIFO_RESET, USER_CTRL_I2C_MST_RESET,
    USER_CTRL_SIG_COND_RESET, WHO_AM_I_VALUE,
};

/// Bus error produced by the mock.
///
/// Implements [`embedded_hal::i2c::Error`] so the mock is substitutable for
/// a real HAL all the way down to error classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MockError {
    /// Nobody acknowledged. Carries whether it was the address or a data byte.
    NoAcknowledge(NoAcknowledgeSource),
}

impl embedded_hal::i2c::Error for MockError {
    fn kind(&self) -> ErrorKind {
        match self {
            MockError::NoAcknowledge(source) => ErrorKind::NoAcknowledge(*source),
        }
    }
}

/// One operation inside a [`Transaction::Raw`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OpSummary {
    /// Bytes the controller sent.
    Write(Vec<u8>),
    /// Bytes the controller received.
    Read(Vec<u8>),
}

/// A bus transaction as the mock observed it.
///
/// [`I2c`] has one required method, `transaction`, and the provided
/// `write`/`read`/`write_read` all route through it as slices of
/// [`Operation`]. The mock classifies those slices into the common shapes so
/// tests can pattern-match on them directly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Transaction {
    /// A single write: register address followed by zero or more data bytes.
    Write {
        /// Device address the controller targeted.
        addr: u8,
        /// Every byte sent, including the register address in position 0.
        bytes: Vec<u8>,
    },
    /// A read with no preceding write — continues from the current register pointer.
    Read {
        /// Device address the controller targeted.
        addr: u8,
        /// Bytes returned.
        received: Vec<u8>,
    },
    /// A write followed by a read in one transaction (repeated start): the normal register read.
    WriteRead {
        /// Device address the controller targeted.
        addr: u8,
        /// Bytes sent — normally just the register address.
        sent: Vec<u8>,
        /// Bytes returned.
        received: Vec<u8>,
    },
    /// Any other operation sequence, preserved verbatim.
    Raw {
        /// Device address the controller targeted.
        addr: u8,
        /// The operations in order.
        ops: Vec<OpSummary>,
    },
}

/// A programmable failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    /// Fail the transaction with this zero-based index with a data NACK.
    /// Nothing from that transaction reaches the register file.
    NackAfter(usize),
    /// In the transaction with index `txn`, overwrite byte `index` of the
    /// data the controller reads back with `value`. Models a corrupted byte
    /// on the wire; the MPU-6050 has no CRC, so this is undetectable at the
    /// driver layer by design.
    CorruptByte {
        /// Zero-based transaction index.
        txn: usize,
        /// Zero-based index into the read buffer.
        index: usize,
        /// Value to substitute.
        value: u8,
    },
    /// `WHO_AM_I` reads back as this value instead of `0x68`.
    WrongDeviceId(u8),
    /// Every byte read is `0xFF`. This is what a controller sees on a bus
    /// with missing pull-ups when it does not flag the absent ACK.
    StuckHigh,
    /// Overflow the FIFO. The FIFO is filled to its 1024-byte capacity from
    /// a stream of well-formed packets whose length is derived from the
    /// current `FIFO_EN` register, retaining the *most recent* 1024 bytes —
    /// which is what the datasheet says the part does. Because 1024 is not
    /// generally a multiple of the packet length, the retained window starts
    /// mid-packet, and `FIFO_OFLOW_INT` is set in `INT_STATUS`.
    FifoOverflow,
}

/// The mock bus. See the [module docs](self).
#[derive(Debug, Clone)]
pub struct MockI2c {
    addr: u8,
    regs: [u8; 256],
    pointer: u8,
    fifo: VecDeque<u8>,
    fifo_last_read: u8,
    reset_poll_budget: u32,
    reset_polls_remaining: Option<u32>,
    log: Vec<Transaction>,
    faults: Vec<Fault>,
    txn_index: usize,
    nacks: usize,
}

impl Default for MockI2c {
    fn default() -> Self {
        Self::new()
    }
}

impl MockI2c {
    /// A device at the default address (`0x68`, AD0 low) in its power-on state.
    pub fn new() -> Self {
        Self::with_address(ADDR_AD0_LOW)
    }

    /// A device at `addr` in its power-on state.
    pub fn with_address(addr: u8) -> Self {
        let mut mock = Self {
            addr,
            regs: [0; 256],
            pointer: 0,
            fifo: VecDeque::new(),
            fifo_last_read: 0,
            reset_poll_budget: 1,
            reset_polls_remaining: None,
            log: Vec::new(),
            faults: Vec::new(),
            txn_index: 0,
            nacks: 0,
        };
        mock.load_defaults();
        mock
    }

    // --- Inspection ---------------------------------------------------------

    /// Current value of a register in the backing store. Does not trigger
    /// read side effects (does not clear `INT_STATUS`, does not pop the FIFO).
    pub fn register(&self, reg: u8) -> u8 {
        self.regs[reg as usize]
    }

    /// Everything the driver has put on the bus, in order.
    pub fn log(&self) -> &[Transaction] {
        &self.log
    }

    /// Drain the log, returning what was in it.
    pub fn take_log(&mut self) -> Vec<Transaction> {
        core::mem::take(&mut self.log)
    }

    /// Number of transactions that were NACKed, including wrong-address ones.
    pub fn nack_count(&self) -> usize {
        self.nacks
    }

    /// Total transactions attempted so far, successful or not. The next
    /// transaction will have this index — useful for aiming
    /// [`Fault::NackAfter`] and [`Fault::CorruptByte`].
    pub fn transactions_seen(&self) -> usize {
        self.txn_index
    }

    /// Bytes currently in the FIFO.
    pub fn fifo_len(&self) -> usize {
        self.fifo.len()
    }

    /// Packet length implied by the current `FIFO_EN` register, in bytes.
    /// Accel contributes 6 (all three axes under one bit); temperature and
    /// each gyro axis contribute 2.
    pub fn packet_len(&self) -> usize {
        let en = self.regs[REG_FIFO_EN as usize];
        let mut len = 0;
        if en & FIFO_EN_ACCEL != 0 {
            len += 6;
        }
        if en & FIFO_EN_TEMP != 0 {
            len += 2;
        }
        for bit in [FIFO_EN_XG, FIFO_EN_YG, FIFO_EN_ZG] {
            if en & bit != 0 {
                len += 2;
            }
        }
        len
    }

    // --- Setup --------------------------------------------------------------

    /// Overwrite a register directly, bypassing bus semantics. Use it to
    /// stage a sensor reading before the driver fetches it.
    pub fn set_register(&mut self, reg: u8, value: u8) {
        self.regs[reg as usize] = value;
    }

    /// Overwrite consecutive registers starting at `start`.
    pub fn set_registers(&mut self, start: u8, values: &[u8]) {
        for (i, v) in values.iter().enumerate() {
            self.regs[start.wrapping_add(i as u8) as usize] = *v;
        }
    }

    /// Append bytes to the FIFO as if the sensor had sampled them.
    pub fn push_fifo(&mut self, bytes: &[u8]) {
        self.fifo.extend(bytes.iter().copied());
    }

    /// How many reads of `PWR_MGMT_1` still show `DEVICE_RESET` set after a
    /// reset is written, before the bit self-clears. Default 1. Use a very
    /// large value to model a device that never comes back.
    pub fn set_reset_poll_count(&mut self, polls: u32) {
        self.reset_poll_budget = polls;
    }

    /// Arm a fault. Faults accumulate; several can be armed at once.
    pub fn inject(&mut self, fault: Fault) {
        if fault == Fault::FifoOverflow {
            self.overflow_fifo();
        }
        self.faults.push(fault);
    }

    /// Disarm every fault. Does not undo a FIFO overflow that has already
    /// been applied to the buffer — reset the FIFO through the bus for that.
    pub fn clear_faults(&mut self) {
        self.faults.clear();
    }

    // --- Device model -------------------------------------------------------

    fn load_defaults(&mut self) {
        self.regs = [0; 256];
        self.regs[REG_WHO_AM_I as usize] = WHO_AM_I_VALUE;
        self.regs[REG_PWR_MGMT_1 as usize] = PWR_MGMT_1_SLEEP;
        self.fifo.clear();
        self.fifo_last_read = 0;
    }

    fn has_fault(&self, pred: impl Fn(&Fault) -> bool) -> bool {
        self.faults.iter().any(pred)
    }

    /// Fill the FIFO with the last 1024 bytes of a stream of whole packets.
    /// The stream is a byte counter, so a test can tell exactly which stream
    /// offset the retained window begins at.
    fn overflow_fifo(&mut self) {
        let plen = self.packet_len().max(1);
        let cap = FIFO_CAPACITY_BYTES as usize;
        let packets = cap / plen + 2;
        let total = packets * plen;
        let start = total - cap;
        self.fifo = (start..total).map(|p| (p & 0xFF) as u8).collect();
        self.regs[REG_INT_STATUS as usize] |= INT_FIFO_OFLOW;
    }

    fn write_register(&mut self, reg: u8, value: u8) {
        match reg {
            REG_PWR_MGMT_1 => {
                if value & PWR_MGMT_1_DEVICE_RESET != 0 {
                    self.load_defaults();
                    self.regs[reg as usize] = PWR_MGMT_1_SLEEP | PWR_MGMT_1_DEVICE_RESET;
                    self.reset_polls_remaining = Some(self.reset_poll_budget);
                    if self.reset_poll_budget == 0 {
                        self.regs[reg as usize] &= !PWR_MGMT_1_DEVICE_RESET;
                        self.reset_polls_remaining = None;
                    }
                } else {
                    self.regs[reg as usize] = value;
                }
            }
            REG_USER_CTRL => {
                // FIFO_RESET only takes effect while FIFO_EN is 0. Written
                // together with FIFO_EN set, it is silently ignored — the
                // trap the datasheet warns about.
                let fifo_enabled = value & USER_CTRL_FIFO_EN != 0;
                if value & USER_CTRL_FIFO_RESET != 0 && !fifo_enabled {
                    self.fifo.clear();
                    self.fifo_last_read = 0;
                }
                let self_clearing =
                    USER_CTRL_FIFO_RESET | USER_CTRL_I2C_MST_RESET | USER_CTRL_SIG_COND_RESET;
                self.regs[reg as usize] = value & !self_clearing;
            }
            REG_FIFO_R_W => self.fifo.push_back(value),
            // Read-only registers ignore writes.
            REG_WHO_AM_I | REG_INT_STATUS | REG_FIFO_COUNT_H | REG_FIFO_COUNT_L => {}
            _ => self.regs[reg as usize] = value,
        }
    }

    fn read_register(&mut self, reg: u8) -> u8 {
        if self.has_fault(|f| *f == Fault::StuckHigh) {
            return 0xFF;
        }
        match reg {
            REG_WHO_AM_I => {
                let wrong = self.faults.iter().find_map(|f| match f {
                    Fault::WrongDeviceId(id) => Some(*id),
                    _ => None,
                });
                wrong.unwrap_or(self.regs[reg as usize])
            }
            REG_INT_STATUS => {
                let v = self.regs[reg as usize];
                self.regs[reg as usize] = 0;
                v
            }
            REG_FIFO_COUNT_H => {
                // Reading the high byte latches both bytes of the count.
                let count = self.fifo.len().min(u16::MAX as usize) as u16;
                self.regs[REG_FIFO_COUNT_H as usize] = (count >> 8) as u8;
                self.regs[REG_FIFO_COUNT_L as usize] = (count & 0xFF) as u8;
                self.regs[REG_FIFO_COUNT_H as usize]
            }
            // The low byte is a shadow. Read alone, it is whatever was
            // latched last time — stale by design.
            REG_FIFO_COUNT_L => self.regs[reg as usize],
            REG_FIFO_R_W => {
                if let Some(b) = self.fifo.pop_front() {
                    self.fifo_last_read = b;
                }
                self.fifo_last_read
            }
            REG_PWR_MGMT_1 => {
                if let Some(n) = self.reset_polls_remaining {
                    if n == 0 {
                        self.regs[reg as usize] &= !PWR_MGMT_1_DEVICE_RESET;
                        self.reset_polls_remaining = None;
                    } else {
                        self.reset_polls_remaining = Some(n - 1);
                    }
                }
                self.regs[reg as usize]
            }
            _ => self.regs[reg as usize],
        }
    }

    fn apply_write(&mut self, bytes: &[u8]) {
        let Some((first, rest)) = bytes.split_first() else {
            return;
        };
        self.pointer = *first;
        for b in rest {
            self.write_register(self.pointer, *b);
            self.advance_pointer();
        }
    }

    fn read_at_pointer(&mut self) -> u8 {
        let v = self.read_register(self.pointer);
        self.advance_pointer();
        v
    }

    /// The register pointer auto-increments on every byte, except at the
    /// FIFO port, where successive bytes pop successive FIFO entries.
    fn advance_pointer(&mut self) {
        if self.pointer != REG_FIFO_R_W {
            self.pointer = self.pointer.wrapping_add(1);
        }
    }
}

impl ErrorType for MockI2c {
    type Error = MockError;
}

impl I2c<SevenBitAddress> for MockI2c {
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let idx = self.txn_index;
        self.txn_index += 1;

        if address != self.addr {
            self.nacks += 1;
            return Err(MockError::NoAcknowledge(NoAcknowledgeSource::Address));
        }
        if self.has_fault(|f| matches!(f, Fault::NackAfter(n) if *n == idx)) {
            self.nacks += 1;
            return Err(MockError::NoAcknowledge(NoAcknowledgeSource::Data));
        }

        let mut ops = Vec::with_capacity(operations.len());
        for op in operations.iter_mut() {
            match op {
                Operation::Write(bytes) => {
                    self.apply_write(bytes);
                    ops.push(OpSummary::Write(bytes.to_vec()));
                }
                Operation::Read(buf) => {
                    for b in buf.iter_mut() {
                        *b = self.read_at_pointer();
                    }
                    for f in &self.faults {
                        if let Fault::CorruptByte { txn, index, value } = f
                            && *txn == idx
                            && *index < buf.len()
                        {
                            buf[*index] = *value;
                        }
                    }
                    ops.push(OpSummary::Read(buf.to_vec()));
                }
            }
        }

        self.log.push(classify(address, ops));
        Ok(())
    }
}

fn classify(addr: u8, mut ops: Vec<OpSummary>) -> Transaction {
    match ops.as_mut_slice() {
        [OpSummary::Write(w)] => Transaction::Write {
            addr,
            bytes: core::mem::take(w),
        },
        [OpSummary::Read(r)] => Transaction::Read {
            addr,
            received: core::mem::take(r),
        },
        [OpSummary::Write(w), OpSummary::Read(r)] => Transaction::WriteRead {
            addr,
            sent: core::mem::take(w),
            received: core::mem::take(r),
        },
        _ => Transaction::Raw { addr, ops },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registers::{REG_ACCEL_CONFIG, REG_GYRO_CONFIG};

    fn read1(m: &mut MockI2c, reg: u8) -> u8 {
        let mut b = [0u8];
        m.write_read(ADDR_AD0_LOW, &[reg], &mut b).unwrap();
        b[0]
    }

    #[test]
    fn classifies_operation_slices_into_transactions() {
        let mut m = MockI2c::new();
        m.write(ADDR_AD0_LOW, &[REG_GYRO_CONFIG, 0x08]).unwrap();
        let mut one = [0u8];
        m.write_read(ADDR_AD0_LOW, &[REG_WHO_AM_I], &mut one)
            .unwrap();
        m.read(ADDR_AD0_LOW, &mut one).unwrap();
        m.transaction(
            ADDR_AD0_LOW,
            &mut [Operation::Write(&[0x1B]), Operation::Write(&[0x00])],
        )
        .unwrap();

        assert!(matches!(m.log()[0], Transaction::Write { .. }));
        assert!(matches!(m.log()[1], Transaction::WriteRead { .. }));
        assert!(matches!(m.log()[2], Transaction::Read { .. }));
        assert!(matches!(m.log()[3], Transaction::Raw { .. }));
        assert_eq!(m.log().len(), 4);
    }

    #[test]
    fn powers_on_with_datasheet_defaults() {
        let mut m = MockI2c::new();
        assert_eq!(read1(&mut m, REG_WHO_AM_I), WHO_AM_I_VALUE);
        assert_eq!(read1(&mut m, REG_PWR_MGMT_1), PWR_MGMT_1_SLEEP);
        assert_eq!(read1(&mut m, REG_ACCEL_CONFIG), 0);
        assert_eq!(m.fifo_len(), 0);
    }

    #[test]
    fn register_pointer_auto_increments_across_a_burst() {
        let mut m = MockI2c::new();
        // One write covering GYRO_CONFIG then ACCEL_CONFIG.
        m.write(ADDR_AD0_LOW, &[REG_GYRO_CONFIG, 0x08, 0x10])
            .unwrap();
        assert_eq!(m.register(REG_GYRO_CONFIG), 0x08);
        assert_eq!(m.register(REG_ACCEL_CONFIG), 0x10);
        // One read covering both back.
        let mut two = [0u8; 2];
        m.write_read(ADDR_AD0_LOW, &[REG_GYRO_CONFIG], &mut two)
            .unwrap();
        assert_eq!(two, [0x08, 0x10]);
    }

    #[test]
    fn nack_fault_fails_only_the_indexed_transaction_and_applies_nothing() {
        let mut m = MockI2c::new();
        m.inject(Fault::NackAfter(1));
        m.write(ADDR_AD0_LOW, &[REG_GYRO_CONFIG, 0x08]).unwrap();
        let err = m
            .write(ADDR_AD0_LOW, &[REG_ACCEL_CONFIG, 0x10])
            .unwrap_err();
        assert_eq!(err, MockError::NoAcknowledge(NoAcknowledgeSource::Data));
        assert_eq!(
            m.register(REG_ACCEL_CONFIG),
            0,
            "NACKed write must not land"
        );
        m.write(ADDR_AD0_LOW, &[REG_ACCEL_CONFIG, 0x10]).unwrap();
        assert_eq!(m.nack_count(), 1);
        assert_eq!(m.log().len(), 2, "failed transaction is not logged");

        m.clear_faults();
        m.inject(Fault::NackAfter(3));
        m.clear_faults();
        m.write(ADDR_AD0_LOW, &[REG_GYRO_CONFIG, 0x00]).unwrap();
    }

    #[test]
    fn wrong_address_is_an_address_nack() {
        let mut m = MockI2c::new();
        let err = m.write(0x42, &[REG_GYRO_CONFIG, 0x08]).unwrap_err();
        assert_eq!(err, MockError::NoAcknowledge(NoAcknowledgeSource::Address));
        use embedded_hal::i2c::Error as _;
        assert_eq!(
            err.kind(),
            ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)
        );
    }

    #[test]
    fn int_status_clears_on_read() {
        let mut m = MockI2c::new();
        m.set_register(REG_INT_STATUS, INT_FIFO_OFLOW);
        assert_eq!(read1(&mut m, REG_INT_STATUS), INT_FIFO_OFLOW);
        assert_eq!(read1(&mut m, REG_INT_STATUS), 0);
    }

    #[test]
    fn fifo_count_low_byte_is_stale_until_high_byte_is_read() {
        let mut m = MockI2c::new();
        m.push_fifo(&[0u8; 300]);
        // Low byte alone: still the power-on latch (0).
        assert_eq!(read1(&mut m, REG_FIFO_COUNT_L), 0);
        // High byte latches both; a 2-byte burst from H gives the truth.
        let mut two = [0u8; 2];
        m.write_read(ADDR_AD0_LOW, &[REG_FIFO_COUNT_H], &mut two)
            .unwrap();
        assert_eq!(u16::from_be_bytes(two), 300);
    }

    #[test]
    fn fifo_reset_is_ignored_while_fifo_en_is_set() {
        let mut m = MockI2c::new();
        m.push_fifo(&[1, 2, 3]);
        // Reset written together with FIFO_EN: ignored.
        m.write(
            ADDR_AD0_LOW,
            &[REG_USER_CTRL, USER_CTRL_FIFO_EN | USER_CTRL_FIFO_RESET],
        )
        .unwrap();
        assert_eq!(m.fifo_len(), 3);
        // Reset with FIFO_EN clear: takes effect, and the bit self-clears.
        m.write(ADDR_AD0_LOW, &[REG_USER_CTRL, USER_CTRL_FIFO_RESET])
            .unwrap();
        assert_eq!(m.fifo_len(), 0);
        assert_eq!(m.register(REG_USER_CTRL) & USER_CTRL_FIFO_RESET, 0);
    }

    #[test]
    fn device_reset_self_clears_after_the_configured_polls() {
        let mut m = MockI2c::new();
        m.set_reset_poll_count(2);
        m.write(ADDR_AD0_LOW, &[REG_PWR_MGMT_1, PWR_MGMT_1_DEVICE_RESET])
            .unwrap();
        assert_ne!(read1(&mut m, REG_PWR_MGMT_1) & PWR_MGMT_1_DEVICE_RESET, 0);
        assert_ne!(read1(&mut m, REG_PWR_MGMT_1) & PWR_MGMT_1_DEVICE_RESET, 0);
        assert_eq!(read1(&mut m, REG_PWR_MGMT_1) & PWR_MGMT_1_DEVICE_RESET, 0);
        // Reset restored SLEEP, as the datasheet's power-on state does.
        assert_ne!(m.register(REG_PWR_MGMT_1) & PWR_MGMT_1_SLEEP, 0);
    }

    #[test]
    fn fifo_overflow_fills_to_capacity_sets_the_flag_and_misaligns() {
        let mut m = MockI2c::new();
        // accel + temp + gyro: 14-byte packets. 1024 mod 14 = 2.
        m.set_register(
            REG_FIFO_EN,
            FIFO_EN_ACCEL | FIFO_EN_TEMP | FIFO_EN_XG | FIFO_EN_YG | FIFO_EN_ZG,
        );
        assert_eq!(m.packet_len(), 14);
        m.inject(Fault::FifoOverflow);
        assert_eq!(m.fifo_len(), FIFO_CAPACITY_BYTES as usize);
        assert_ne!(m.register(REG_INT_STATUS) & INT_FIFO_OFLOW, 0);
        // The stream is a byte counter; the first retained byte tells us the
        // stream offset the window starts at. It must not be packet-aligned.
        let first = read1(&mut m, REG_FIFO_R_W) as usize;
        let total = (1024 / 14 + 2) * 14;
        let start = total - 1024;
        assert_eq!(first, start & 0xFF);
        assert_ne!(start % 14, 0, "retained window must start mid-packet");
    }

    #[test]
    fn stuck_high_and_wrong_id_faults_shape_reads() {
        let mut m = MockI2c::new();
        m.inject(Fault::WrongDeviceId(0x70));
        assert_eq!(read1(&mut m, REG_WHO_AM_I), 0x70);
        m.clear_faults();
        m.inject(Fault::StuckHigh);
        assert_eq!(read1(&mut m, REG_WHO_AM_I), 0xFF);
        assert_eq!(read1(&mut m, REG_ACCEL_CONFIG), 0xFF);
    }

    #[test]
    fn corrupt_byte_fault_hits_one_byte_of_one_transaction() {
        let mut m = MockI2c::new();
        m.set_registers(REG_GYRO_CONFIG, &[0x08, 0x10]);
        m.inject(Fault::CorruptByte {
            txn: 0,
            index: 1,
            value: 0xEE,
        });
        let mut two = [0u8; 2];
        m.write_read(ADDR_AD0_LOW, &[REG_GYRO_CONFIG], &mut two)
            .unwrap();
        assert_eq!(two, [0x08, 0xEE]);
        m.write_read(ADDR_AD0_LOW, &[REG_GYRO_CONFIG], &mut two)
            .unwrap();
        assert_eq!(
            two,
            [0x08, 0x10],
            "only the indexed transaction is corrupted"
        );
    }
}
