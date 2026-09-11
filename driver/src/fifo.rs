//! The 1024-byte FIFO: configuration, packet arithmetic and draining.
//!
//! The FIFO is where most MPU-6050 drivers go wrong, because four of its
//! behaviours are documented in single sentences that are easy to skim past.
//! Each is quoted in `docs/datasheet-notes.md`; each is enforced here.
//!
//! - **Packet layout is register order**, not the order the enable bits were
//!   set: accelerometer (6 bytes), then temperature (2), then gyroscope
//!   X, Y, Z (2 each, individually gated). [`FifoSample::parse`] decodes in
//!   that order and no other.
//! - **`FIFO_COUNT_L` is a shadow.** Only reading `FIFO_COUNT_H` latches a
//!   fresh count into both bytes. [`Mpu6050::fifo_count`] is one 2-byte burst
//!   from the high byte.
//! - **`INT_STATUS` is clear-on-read.** [`Mpu6050::fifo_overflowed`] consumes
//!   the overflow flag; the driver latches it internally so a caller who
//!   ignores the error is still refused on the next read.
//! - **`FIFO_RESET` is ignored while `FIFO_EN` is set.** [`Mpu6050::fifo_reset`]
//!   disables, resets, re-enables — three writes, in that order.
//!
//! And the one that follows from the datasheet by arithmetic rather than by
//! quotation: on overflow the part drops the *oldest bytes* and keeps
//! writing. It has no notion of packets, and 1024 is not a multiple of 12 or
//! 14, so the retained window starts mid-packet and every later read is
//! byte-misaligned. The values look plausible. [`Mpu6050::fifo_read`] refuses
//! to return anything after an overflow until the FIFO is reset, and refuses
//! any count that is not a whole number of packets.

use embedded_hal::i2c::I2c;

use crate::registers::{
    FIFO_EN_ACCEL, FIFO_EN_TEMP, FIFO_EN_XG, FIFO_EN_YG, FIFO_EN_ZG, INT_FIFO_OFLOW,
    REG_FIFO_COUNT_H, REG_FIFO_EN, REG_FIFO_R_W, REG_INT_STATUS, REG_USER_CTRL, USER_CTRL_FIFO_EN,
    USER_CTRL_FIFO_RESET,
};
use crate::{Error, Mpu6050};

/// Largest single FIFO burst, in bytes. Sized for a stack buffer that holds
/// two whole 14-byte packets; the crate has no allocator to draw on.
pub const MAX_BURST: usize = 32;

/// Which sensor registers are copied into the FIFO on every sample.
///
/// The accelerometer is all-or-nothing (one enable bit covers all six
/// bytes); temperature and each gyroscope axis are gated individually,
/// exactly as `FIFO_EN` (Register 35) is laid out.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FifoConfig {
    /// All three accelerometer axes, 6 bytes.
    pub accel: bool,
    /// Die temperature, 2 bytes.
    pub temp: bool,
    /// Gyroscope X, 2 bytes.
    pub gyro_x: bool,
    /// Gyroscope Y, 2 bytes.
    pub gyro_y: bool,
    /// Gyroscope Z, 2 bytes.
    pub gyro_z: bool,
}

impl FifoConfig {
    /// Nothing enabled. The FIFO receives no data.
    pub const NONE: Self = Self {
        accel: false,
        temp: false,
        gyro_x: false,
        gyro_y: false,
        gyro_z: false,
    };
    /// Accelerometer only: 6-byte packets.
    pub const ACCEL: Self = Self {
        accel: true,
        ..Self::NONE
    };
    /// Accelerometer and all three gyroscope axes: 12-byte packets.
    pub const ACCEL_GYRO: Self = Self {
        accel: true,
        gyro_x: true,
        gyro_y: true,
        gyro_z: true,
        ..Self::NONE
    };
    /// Everything: 14-byte packets.
    pub const ALL: Self = Self {
        temp: true,
        ..Self::ACCEL_GYRO
    };

    /// Bytes per sample with this configuration.
    pub const fn packet_len(self) -> usize {
        6 * self.accel as usize
            + 2 * self.temp as usize
            + 2 * (self.gyro_x as usize + self.gyro_y as usize + self.gyro_z as usize)
    }

    /// The value to write to `FIFO_EN`.
    pub const fn fifo_en_bits(self) -> u8 {
        (if self.accel { FIFO_EN_ACCEL } else { 0 })
            | (if self.temp { FIFO_EN_TEMP } else { 0 })
            | (if self.gyro_x { FIFO_EN_XG } else { 0 })
            | (if self.gyro_y { FIFO_EN_YG } else { 0 })
            | (if self.gyro_z { FIFO_EN_ZG } else { 0 })
    }

    /// True when no sensor is enabled.
    pub const fn is_empty(self) -> bool {
        self.packet_len() == 0
    }
}

/// One decoded FIFO packet. A field is `None` when its sensor was not
/// enabled in the [`FifoConfig`] the packet was read under.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FifoSample {
    /// Raw accelerometer counts, X/Y/Z.
    pub accel: Option<[i16; 3]>,
    /// Raw temperature counts.
    pub temp: Option<i16>,
    /// Raw gyroscope counts, X/Y/Z — each axis independently present.
    pub gyro: [Option<i16>; 3],
}

impl FifoSample {
    /// Decode one packet. `bytes.len()` must equal `cfg.packet_len()`.
    ///
    /// Fields are consumed in register order — accel, temperature, gyro X,
    /// Y, Z — because that is the order the part writes them, regardless of
    /// how the enable bits were set.
    pub fn parse(cfg: FifoConfig, bytes: &[u8]) -> Self {
        debug_assert_eq!(bytes.len(), cfg.packet_len());
        let mut at = 0usize;
        let mut next = || {
            let v = i16::from_be_bytes([bytes[at], bytes[at + 1]]);
            at += 2;
            v
        };
        let accel = cfg.accel.then(|| [next(), next(), next()]);
        let temp = cfg.temp.then(&mut next);
        let gx = cfg.gyro_x.then(&mut next);
        let gy = cfg.gyro_y.then(&mut next);
        let gz = cfg.gyro_z.then(&mut next);
        Self {
            accel,
            temp,
            gyro: [gx, gy, gz],
        }
    }
}

impl<I2C: I2c> Mpu6050<I2C> {
    /// The FIFO configuration the driver last wrote (or [`FifoConfig::NONE`]).
    pub fn fifo_config(&self) -> FifoConfig {
        self.fifo
    }

    /// Choose which sensors the FIFO records. Writes `FIFO_EN` and stores
    /// the configuration so packet length is derived, never assumed.
    pub fn fifo_configure(&mut self, cfg: FifoConfig) -> Result<(), Error<I2C::Error>> {
        self.ensure_init()?;
        self.write_reg(REG_FIFO_EN, cfg.fifo_en_bits())?;
        self.fifo = cfg;
        Ok(())
    }

    /// Set `USER_CTRL.FIFO_EN`. Other bits are preserved.
    pub fn fifo_enable(&mut self) -> Result<(), Error<I2C::Error>> {
        self.ensure_init()?;
        self.modify_reg(REG_USER_CTRL, USER_CTRL_FIFO_EN, USER_CTRL_FIFO_EN)
    }

    /// Clear `USER_CTRL.FIFO_EN`. The FIFO keeps its contents; it just stops
    /// accepting or serving bytes. Other bits are preserved.
    pub fn fifo_disable(&mut self) -> Result<(), Error<I2C::Error>> {
        self.ensure_init()?;
        self.modify_reg(REG_USER_CTRL, USER_CTRL_FIFO_EN, 0)
    }

    /// Empty the FIFO and leave it enabled, aligned and trusted again.
    ///
    /// Register 106: `FIFO_RESET` "resets the FIFO buffer when set to 1
    /// **while FIFO_EN equals 0**". Written to a running FIFO it does nothing,
    /// silently — so this is three writes, not one: clear `FIFO_EN`, set
    /// `FIFO_RESET`, set `FIFO_EN`. Every other bit of `USER_CTRL` is read
    /// first and preserved (`I2C_IF_DIS` must stay 0 on the MPU-6050).
    ///
    /// This is the only way to recover from [`Error::FifoOverflow`].
    pub fn fifo_reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.ensure_init()?;
        let base = self.read_reg(REG_USER_CTRL)? & !(USER_CTRL_FIFO_EN | USER_CTRL_FIFO_RESET);
        self.write_reg(REG_USER_CTRL, base)?;
        self.write_reg(REG_USER_CTRL, base | USER_CTRL_FIFO_RESET)?;
        self.write_reg(REG_USER_CTRL, base | USER_CTRL_FIFO_EN)?;
        self.fifo_poisoned = false;
        Ok(())
    }

    /// Bytes currently in the FIFO, as one 2-byte burst from `FIFO_COUNT_H`.
    ///
    /// Registers 114–115: reading the high byte latches both; the low byte
    /// read alone is stale by design. Never read them separately.
    pub fn fifo_count(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.ensure_init()?;
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(self.addr, &[REG_FIFO_COUNT_H], &mut buf)?;
        Ok(u16::from_be_bytes(buf))
    }

    /// Whether the FIFO has overflowed since the flag was last read.
    ///
    /// **Consumes the flag.** Register 58 clears every bit of `INT_STATUS` on
    /// read, so calling this speculatively destroys the information. The
    /// driver latches a seen overflow internally until [`fifo_reset`](Self::fifo_reset),
    /// so [`fifo_read`](Self::fifo_read) stays refused even if the caller
    /// discards this result.
    pub fn fifo_overflowed(&mut self) -> Result<bool, Error<I2C::Error>> {
        self.ensure_init()?;
        let overflowed = self.read_reg(REG_INT_STATUS)? & INT_FIFO_OFLOW != 0;
        if overflowed {
            self.fifo_poisoned = true;
        }
        Ok(overflowed || self.fifo_poisoned)
    }

    /// Drain whole packets into `out`, returning how many were written.
    ///
    /// Sequence:
    ///
    /// 1. If an overflow has been seen and not reset — now or earlier —
    ///    return [`Error::FifoOverflow`] and read nothing. The contents are
    ///    byte-misaligned and would decode to plausible garbage.
    /// 2. Read `FIFO_COUNT` as one burst.
    /// 3. If the count is not a whole number of packets, return
    ///    [`Error::FifoUnaligned`] and read nothing. Consuming a partial
    ///    packet desynchronises every packet after it.
    /// 4. Read `min(available, out.len())` packets from `FIFO_R_W` in bursts
    ///    of at most [`MAX_BURST`] bytes, and decode each in register order.
    ///
    /// Returns `Ok(0)` if no sensors are configured. Returns
    /// [`Error::BufferTooSmall`] if `out` is empty.
    pub fn fifo_read(&mut self, out: &mut [FifoSample]) -> Result<usize, Error<I2C::Error>> {
        self.ensure_init()?;
        let cfg = self.fifo;
        let plen = cfg.packet_len();
        if plen == 0 {
            return Ok(0);
        }
        if out.is_empty() {
            return Err(Error::BufferTooSmall);
        }
        if self.fifo_overflowed()? {
            return Err(Error::FifoOverflow);
        }

        let count = self.fifo_count()?;
        let count_bytes = usize::from(count);
        if count_bytes % plen != 0 {
            return Err(Error::FifoUnaligned(count));
        }
        let wanted = (count_bytes / plen).min(out.len());

        let per_burst = (MAX_BURST / plen) * plen;
        let mut scratch = [0u8; MAX_BURST];
        let mut done = 0;
        while done < wanted {
            let bytes = ((wanted - done) * plen).min(per_burst);
            self.i2c
                .write_read(self.addr, &[REG_FIFO_R_W], &mut scratch[..bytes])?;
            for packet in scratch[..bytes].chunks_exact(plen) {
                out[done] = FifoSample::parse(cfg, packet);
                done += 1;
            }
        }
        Ok(wanted)
    }
}
