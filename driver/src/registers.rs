//! MPU-6050 register map and configuration values.
//!
//! Register addresses and semantics are taken from the InvenSense *MPU-6000
//! and MPU-6050 Register Map and Descriptions*, document RM-MPU-6000A-00,
//! revision 4.0 (03/09/2012). Every constant here is backed by a quoted line
//! in `docs/datasheet-notes.md`, cited by register number — section numbers
//! are avoided because they move between revisions.

/// Default 7-bit I2C address (AD0 tied low). `0x69` when AD0 is high.
pub const ADDR_AD0_LOW: u8 = 0x68;
/// 7-bit I2C address with AD0 tied high.
pub const ADDR_AD0_HIGH: u8 = 0x69;

/// Value the `WHO_AM_I` register reports on a genuine MPU-6050.
///
/// The register holds the upper six bits of the I2C address and hard-codes
/// bits 0 and 7 to zero, so devices at both `0x68` and `0x69` report the same
/// identity value. An identity check must compare against this constant, not
/// against the bus address.
pub const WHO_AM_I_VALUE: u8 = 0x68;

// --- Register addresses -----------------------------------------------------

/// Register 13 — `SELF_TEST_X`: `XA_TEST[4:2]` in bits 7:5, `XG_TEST[4:0]` in bits 4:0.
pub const REG_SELF_TEST_X: u8 = 0x0D;
/// Register 14 — `SELF_TEST_Y`: `YA_TEST[4:2]` in bits 7:5, `YG_TEST[4:0]` in bits 4:0.
pub const REG_SELF_TEST_Y: u8 = 0x0E;
/// Register 15 — `SELF_TEST_Z`: `ZA_TEST[4:2]` in bits 7:5, `ZG_TEST[4:0]` in bits 4:0.
pub const REG_SELF_TEST_Z: u8 = 0x0F;
/// Register 16 — `SELF_TEST_A`: `XA_TEST[1:0]` in bits 5:4, `YA_TEST[1:0]` in bits 3:2, `ZA_TEST[1:0]` in bits 1:0.
pub const REG_SELF_TEST_A: u8 = 0x10;
/// Register 25 — sample rate divider.
pub const REG_SMPLRT_DIV: u8 = 0x19;
/// Register 26 — external sync and digital low-pass filter configuration.
pub const REG_CONFIG: u8 = 0x1A;
/// Register 27 — gyroscope self-test and full-scale range.
pub const REG_GYRO_CONFIG: u8 = 0x1B;
/// Register 28 — accelerometer self-test and full-scale range.
pub const REG_ACCEL_CONFIG: u8 = 0x1C;
/// Register 35 — which sensor registers are written into the FIFO.
pub const REG_FIFO_EN: u8 = 0x23;
/// Register 56 — interrupt enable.
pub const REG_INT_ENABLE: u8 = 0x38;
/// Register 58 — interrupt status. Every bit clears when the register is read.
pub const REG_INT_STATUS: u8 = 0x3A;
/// Register 59 — `ACCEL_XOUT_H`, first of six accelerometer output bytes.
pub const REG_ACCEL_XOUT_H: u8 = 0x3B;
/// Register 65 — `TEMP_OUT_H`, first of two temperature output bytes.
pub const REG_TEMP_OUT_H: u8 = 0x41;
/// Register 67 — `GYRO_XOUT_H`, first of six gyroscope output bytes.
pub const REG_GYRO_XOUT_H: u8 = 0x43;
/// Register 106 — user control: FIFO enable and reset, I2C master, signal-path reset.
pub const REG_USER_CTRL: u8 = 0x6A;
/// Register 107 — power management: device reset, sleep, clock source.
pub const REG_PWR_MGMT_1: u8 = 0x6B;
/// Register 114 — FIFO byte count, high byte. Reading it latches both count bytes.
pub const REG_FIFO_COUNT_H: u8 = 0x72;
/// Register 115 — FIFO byte count, low byte. Stale unless `FIFO_COUNT_H` was read first.
pub const REG_FIFO_COUNT_L: u8 = 0x73;
/// Register 116 — FIFO read/write port. Each read pops one byte.
pub const REG_FIFO_R_W: u8 = 0x74;
/// Register 117 — device identity.
pub const REG_WHO_AM_I: u8 = 0x75;

// --- PWR_MGMT_1 (Register 107) ---------------------------------------------

/// `PWR_MGMT_1` bit 7 — resets all registers to their defaults. Self-clears when done.
pub const PWR_MGMT_1_DEVICE_RESET: u8 = 0x80;
/// `PWR_MGMT_1` bit 6 — puts the device into sleep mode. Set on power-up.
pub const PWR_MGMT_1_SLEEP: u8 = 0x40;
/// `PWR_MGMT_1` CLKSEL = 1 — use the gyroscope X-axis PLL as the clock source.
///
/// The datasheet recommends a gyroscope PLL over the internal 8 MHz oscillator
/// for stability.
pub const PWR_MGMT_1_CLKSEL_PLL_X: u8 = 0x01;

// --- GYRO_CONFIG / ACCEL_CONFIG self-test bits (Registers 27 and 28) --------

/// `GYRO_CONFIG` bits 7:5 — `XG_ST`, `YG_ST`, `ZG_ST`. Actuates all three gyro axes.
pub const GYRO_CONFIG_ST_ALL: u8 = 0xE0;
/// `ACCEL_CONFIG` bits 7:5 — `XA_ST`, `YA_ST`, `ZA_ST`. Actuates all three accel axes.
pub const ACCEL_CONFIG_ST_ALL: u8 = 0xE0;

// --- FIFO_EN (Register 35) --------------------------------------------------

/// `FIFO_EN` bit 7 — write `TEMP_OUT_H/L` (2 bytes) into the FIFO each sample.
pub const FIFO_EN_TEMP: u8 = 0x80;
/// `FIFO_EN` bit 6 — write `GYRO_XOUT_H/L` (2 bytes) into the FIFO each sample.
pub const FIFO_EN_XG: u8 = 0x40;
/// `FIFO_EN` bit 5 — write `GYRO_YOUT_H/L` (2 bytes) into the FIFO each sample.
pub const FIFO_EN_YG: u8 = 0x20;
/// `FIFO_EN` bit 4 — write `GYRO_ZOUT_H/L` (2 bytes) into the FIFO each sample.
pub const FIFO_EN_ZG: u8 = 0x10;
/// `FIFO_EN` bit 3 — write all six accelerometer bytes into the FIFO each sample.
///
/// Unlike the gyroscope, the accelerometer axes are gated by a single bit.
pub const FIFO_EN_ACCEL: u8 = 0x08;

// --- USER_CTRL (Register 106) -----------------------------------------------

/// `USER_CTRL` bit 6 — enables FIFO operation. The FIFO cannot be read or written while clear.
pub const USER_CTRL_FIFO_EN: u8 = 0x40;
/// `USER_CTRL` bit 4 — selects SPI on the MPU-6000. **Always write 0 on the MPU-6050.**
pub const USER_CTRL_I2C_IF_DIS: u8 = 0x10;
/// `USER_CTRL` bit 2 — resets the FIFO. **Only takes effect while `FIFO_EN` is 0.** Self-clears.
pub const USER_CTRL_FIFO_RESET: u8 = 0x04;
/// `USER_CTRL` bit 1 — resets the auxiliary I2C master. Self-clears. Not used by this driver.
pub const USER_CTRL_I2C_MST_RESET: u8 = 0x02;
/// `USER_CTRL` bit 0 — resets the sensor signal paths and registers. Self-clears.
pub const USER_CTRL_SIG_COND_RESET: u8 = 0x01;

// --- INT_ENABLE / INT_STATUS (Registers 56 and 58) ---------------------------

/// Bit 4 in both `INT_ENABLE` and `INT_STATUS` — FIFO buffer overflow.
pub const INT_FIFO_OFLOW: u8 = 0x10;
/// Bit 0 in both `INT_ENABLE` and `INT_STATUS` — data ready.
pub const INT_DATA_RDY: u8 = 0x01;

/// Size of the on-chip FIFO in bytes.
///
/// Not a multiple of any realistic packet size, which is why an overflow
/// destroys packet alignment — see `docs/datasheet-notes.md`.
pub const FIFO_CAPACITY_BYTES: u16 = 1024;

// --- Configuration enums ----------------------------------------------------

/// Accelerometer full-scale range.
///
/// The discriminant is the value written to `ACCEL_CONFIG[4:3]` (`AFS_SEL`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccelRange {
    /// ±2 g, 16 384 LSB/g.
    G2 = 0,
    /// ±4 g, 8 192 LSB/g.
    G4 = 1,
    /// ±8 g, 4 096 LSB/g.
    G8 = 2,
    /// ±16 g, 2 048 LSB/g.
    G16 = 3,
}

impl AccelRange {
    /// LSB per g, from the Register 59–64 sensitivity table.
    pub const fn sensitivity(self) -> f32 {
        match self {
            AccelRange::G2 => 16_384.0,
            AccelRange::G4 => 8_192.0,
            AccelRange::G8 => 4_096.0,
            AccelRange::G16 => 2_048.0,
        }
    }

    /// The bit pattern for `ACCEL_CONFIG`, already shifted into `AFS_SEL`.
    pub const fn config_bits(self) -> u8 {
        (self as u8) << 3
    }
}

/// Gyroscope full-scale range.
///
/// The discriminant is the value written to `GYRO_CONFIG[4:3]` (`FS_SEL`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GyroRange {
    /// ±250 °/s, 131 LSB/(°/s).
    Dps250 = 0,
    /// ±500 °/s, 65.5 LSB/(°/s).
    Dps500 = 1,
    /// ±1000 °/s, 32.8 LSB/(°/s).
    Dps1000 = 2,
    /// ±2000 °/s, 16.4 LSB/(°/s).
    Dps2000 = 3,
}

impl GyroRange {
    /// LSB per degree per second, from the Register 67–72 sensitivity table.
    pub const fn sensitivity(self) -> f32 {
        match self {
            GyroRange::Dps250 => 131.0,
            GyroRange::Dps500 => 65.5,
            GyroRange::Dps1000 => 32.8,
            GyroRange::Dps2000 => 16.4,
        }
    }

    /// The bit pattern for `GYRO_CONFIG`, already shifted into `FS_SEL`.
    pub const fn config_bits(self) -> u8 {
        (self as u8) << 3
    }
}

/// Digital low-pass filter setting, written to `CONFIG[2:0]` (`DLPF_CFG`).
///
/// Lower bandwidth means more filtering and more group delay. `Hz260`
/// disables the filter, which also raises the gyroscope output rate to
/// 8 kHz. `DLPF_CFG = 7` is marked RESERVED by the datasheet and is
/// deliberately not exposed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DlpfConfig {
    /// Filter off. Accel 260 Hz, gyro 256 Hz bandwidth; gyro output rate 8 kHz.
    Hz260 = 0,
    /// Accel 184 Hz, gyro 188 Hz bandwidth; gyro output rate 1 kHz.
    Hz184 = 1,
    /// Accel 94 Hz, gyro 98 Hz bandwidth; gyro output rate 1 kHz.
    Hz94 = 2,
    /// Accel 44 Hz, gyro 42 Hz bandwidth; gyro output rate 1 kHz.
    Hz44 = 3,
    /// Accel 21 Hz, gyro 20 Hz bandwidth; gyro output rate 1 kHz.
    Hz21 = 4,
    /// Accel 10 Hz, gyro 10 Hz bandwidth; gyro output rate 1 kHz.
    Hz10 = 5,
    /// Accel 5 Hz, gyro 5 Hz bandwidth; gyro output rate 1 kHz.
    Hz5 = 6,
}

impl DlpfConfig {
    /// Gyroscope output rate in Hz for this filter setting.
    ///
    /// This is the base rate that `SMPLRT_DIV` divides down, so it is needed
    /// to compute a target sample rate correctly: 8 kHz with the filter off,
    /// 1 kHz with it on. Register 25.
    pub const fn gyro_output_rate_hz(self) -> u32 {
        match self {
            DlpfConfig::Hz260 => 8_000,
            _ => 1_000,
        }
    }
}
