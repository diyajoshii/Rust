//! An MPU-6050 driver you can test without an MPU-6050.
//!
//! Register-level `no_std` driver for the InvenSense MPU-6050
//! accelerometer/gyroscope, generic over any [`embedded_hal::i2c::I2c`]
//! implementation.
//!
//! # Two builds
//!
//! - **Default** — pure `no_std`. No allocator, no operating system. Builds
//!   for `thumbv7em-none-eabihf` and friends.
//! - **`mock` feature** — adds the `mock` module, a fake I2C bus with a register
//!   backing store, a transaction log and programmable faults. Pulls in
//!   `std`, so the entire driver runs under `cargo test` on a host with no
//!   hardware attached.
//!
//! Everything the driver does is asserted against the mock's transaction
//! log — actual bus traffic, not internal state.
//!
//! # Example
//!
//! ```no_run
//! # fn demo<I2C: embedded_hal::i2c::I2c>(i2c: I2C) -> Result<(), mpu6050_nostd::Error<I2C::Error>> {
//! use mpu6050_nostd::{AccelRange, Mpu6050, registers::ADDR_AD0_LOW};
//!
//! let mut imu = Mpu6050::new(i2c, ADDR_AD0_LOW);
//! imu.init()?;                          // identity check, reset, wake, PLL clock
//! imu.set_accel_range(AccelRange::G8)?; // conversion below now uses ±8 g
//! let [x, y, z] = imu.accel_g()?;       // one 6-byte burst, converted
//! # let _ = (x, y, z);
//! # Ok(()) }
//! ```
#![no_std]
#![warn(missing_docs)]

#[cfg(feature = "mock")]
extern crate std;

pub mod error;
#[cfg(feature = "mock")]
pub mod mock;
pub mod registers;

pub use error::Error;
pub use registers::{AccelRange, DlpfConfig, GyroRange};

use embedded_hal::i2c::I2c;
use registers::{
    PWR_MGMT_1_CLKSEL_PLL_X, PWR_MGMT_1_DEVICE_RESET, REG_ACCEL_CONFIG, REG_ACCEL_XOUT_H,
    REG_CONFIG, REG_GYRO_CONFIG, REG_GYRO_XOUT_H, REG_PWR_MGMT_1, REG_SMPLRT_DIV, REG_TEMP_OUT_H,
    REG_WHO_AM_I, WHO_AM_I_VALUE,
};

/// How many times [`Mpu6050::init`] reads `PWR_MGMT_1` waiting for
/// `DEVICE_RESET` to self-clear before giving up with [`Error::ResetTimeout`].
///
/// The datasheet says the bit "automatically clears to 0 once the reset is
/// done" but gives no duration, so the driver polls rather than sleeping.
/// Polling needs no timer, and it makes the reset path testable on a host.
pub const RESET_POLL_LIMIT: usize = 100;

/// `ACCEL_CONFIG[4:3]` and `GYRO_CONFIG[4:3]` — the range fields.
const RANGE_MASK: u8 = 0b0001_1000;
/// `CONFIG[2:0]` — the DLPF field.
const DLPF_MASK: u8 = 0b0000_0111;

/// Register 65–66: `T = raw / 340 + 36.53`.
const TEMP_LSB_PER_DEG_C: f32 = 340.0;
const TEMP_OFFSET_DEG_C: f32 = 36.53;

/// Driver for one MPU-6050 on an I2C bus.
///
/// The configured ranges and filter setting live in the struct so that unit
/// conversion and sample-rate arithmetic always use what the device is
/// actually set to. A driver that hard-codes ±2 g is wrong the moment the
/// range changes; this one cannot be.
///
/// Construction does not touch the bus. Call [`init`](Self::init) first.
pub struct Mpu6050<I2C> {
    i2c: I2C,
    addr: u8,
    accel_range: AccelRange,
    gyro_range: GyroRange,
    dlpf: DlpfConfig,
    initialised: bool,
}

impl<I2C: I2c> Mpu6050<I2C> {
    /// Wrap a bus and a 7-bit device address. The struct starts in the
    /// chip's power-on configuration (±2 g, ±250 °/s, DLPF off).
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            accel_range: AccelRange::G2,
            gyro_range: GyroRange::Dps250,
            dlpf: DlpfConfig::Hz260,
            initialised: false,
        }
    }

    /// Give the bus back.
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Borrow the underlying bus.
    pub fn bus(&self) -> &I2C {
        &self.i2c
    }

    /// Mutably borrow the underlying bus.
    pub fn bus_mut(&mut self) -> &mut I2C {
        &mut self.i2c
    }

    /// The accelerometer range the driver last wrote (or the power-on default).
    pub fn accel_range(&self) -> AccelRange {
        self.accel_range
    }

    /// The gyroscope range the driver last wrote (or the power-on default).
    pub fn gyro_range(&self) -> GyroRange {
        self.gyro_range
    }

    /// The DLPF setting the driver last wrote (or the power-on default).
    pub fn dlpf(&self) -> DlpfConfig {
        self.dlpf
    }

    // --- Bring-up -----------------------------------------------------------

    /// Read `WHO_AM_I`. Does not require [`init`](Self::init).
    pub fn who_am_i(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_reg(REG_WHO_AM_I)
    }

    /// Bring the device to a known, awake, stably-clocked state.
    ///
    /// Sequence, each step observable on the bus:
    ///
    /// 1. Read `WHO_AM_I`; anything but `0x68` is [`Error::WrongDevice`].
    /// 2. Write `DEVICE_RESET`.
    /// 3. Poll `PWR_MGMT_1` until the reset bit self-clears, up to
    ///    [`RESET_POLL_LIMIT`] reads, else [`Error::ResetTimeout`].
    /// 4. Clear `SLEEP`.
    /// 5. Select the gyroscope X-axis PLL as clock. This is a separate write
    ///    from step 4 because the PLL locks to a running gyroscope.
    /// 6. Write `CONFIG`, `GYRO_CONFIG` and `ACCEL_CONFIG` in one burst
    ///    (they are consecutive registers) so the device matches the ranges
    ///    held in the struct.
    ///
    /// Calling it again repeats the sequence and leaves the same state.
    pub fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        let id = self.who_am_i()?;
        if id != WHO_AM_I_VALUE {
            return Err(Error::WrongDevice(id));
        }

        self.write_reg(REG_PWR_MGMT_1, PWR_MGMT_1_DEVICE_RESET)?;
        let mut cleared = false;
        for _ in 0..RESET_POLL_LIMIT {
            if self.read_reg(REG_PWR_MGMT_1)? & PWR_MGMT_1_DEVICE_RESET == 0 {
                cleared = true;
                break;
            }
        }
        if !cleared {
            return Err(Error::ResetTimeout);
        }

        self.write_reg(REG_PWR_MGMT_1, 0x00)?;
        self.write_reg(REG_PWR_MGMT_1, PWR_MGMT_1_CLKSEL_PLL_X)?;

        self.i2c.write(
            self.addr,
            &[
                REG_CONFIG,
                self.dlpf as u8,
                self.gyro_range.config_bits(),
                self.accel_range.config_bits(),
            ],
        )?;

        self.initialised = true;
        Ok(())
    }

    // --- Configuration ------------------------------------------------------

    /// Set the accelerometer full-scale range. Other bits of `ACCEL_CONFIG`
    /// (the self-test flags) are preserved.
    pub fn set_accel_range(&mut self, range: AccelRange) -> Result<(), Error<I2C::Error>> {
        self.ensure_init()?;
        self.modify_reg(REG_ACCEL_CONFIG, RANGE_MASK, range.config_bits())?;
        self.accel_range = range;
        Ok(())
    }

    /// Set the gyroscope full-scale range. Other bits of `GYRO_CONFIG` are preserved.
    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Error<I2C::Error>> {
        self.ensure_init()?;
        self.modify_reg(REG_GYRO_CONFIG, RANGE_MASK, range.config_bits())?;
        self.gyro_range = range;
        Ok(())
    }

    /// Set the digital low-pass filter. Other bits of `CONFIG` are preserved.
    ///
    /// This also changes the base rate that [`set_sample_rate_hz`](Self::set_sample_rate_hz)
    /// divides down — 8 kHz with the filter off, 1 kHz with it on — so set
    /// the filter before the sample rate.
    pub fn set_dlpf(&mut self, dlpf: DlpfConfig) -> Result<(), Error<I2C::Error>> {
        self.ensure_init()?;
        self.modify_reg(REG_CONFIG, DLPF_MASK, dlpf as u8)?;
        self.dlpf = dlpf;
        Ok(())
    }

    /// Set the sample rate as close as the hardware allows to `hz`, and
    /// return the rate actually achieved.
    ///
    /// Register 25: `rate = base / (1 + SMPLRT_DIV)`, where `base` is the
    /// gyroscope output rate for the current DLPF setting. The divider is
    /// rounded to the nearest value; rates above `base` or below `base / 256`
    /// are [`Error::SampleRateUnreachable`].
    pub fn set_sample_rate_hz(&mut self, hz: u16) -> Result<u16, Error<I2C::Error>> {
        self.ensure_init()?;
        let base = self.dlpf.gyro_output_rate_hz();
        let hz32 = u32::from(hz);
        if hz == 0 || hz32 > base {
            return Err(Error::SampleRateUnreachable(hz));
        }
        let div = (base + hz32 / 2) / hz32 - 1;
        if div > 0xFF {
            return Err(Error::SampleRateUnreachable(hz));
        }
        self.write_reg(REG_SMPLRT_DIV, div as u8)?;
        Ok((base / (1 + div)) as u16)
    }

    // --- Measurement --------------------------------------------------------

    /// Raw accelerometer counts for X, Y, Z in one 6-byte burst.
    ///
    /// One transaction, not six: the device latches all axes together, and
    /// separate reads can straddle a sample update and tear across axes.
    pub fn accel_raw(&mut self) -> Result<[i16; 3], Error<I2C::Error>> {
        self.ensure_init()?;
        self.read_vec3(REG_ACCEL_XOUT_H)
    }

    /// Raw gyroscope counts for X, Y, Z in one 6-byte burst.
    pub fn gyro_raw(&mut self) -> Result<[i16; 3], Error<I2C::Error>> {
        self.ensure_init()?;
        self.read_vec3(REG_GYRO_XOUT_H)
    }

    /// Acceleration in g, scaled by the configured range.
    pub fn accel_g(&mut self) -> Result<[f32; 3], Error<I2C::Error>> {
        let raw = self.accel_raw()?;
        let s = self.accel_range.sensitivity();
        Ok(raw.map(|v| f32::from(v) / s))
    }

    /// Angular rate in degrees per second, scaled by the configured range.
    pub fn gyro_dps(&mut self) -> Result<[f32; 3], Error<I2C::Error>> {
        let raw = self.gyro_raw()?;
        let s = self.gyro_range.sensitivity();
        Ok(raw.map(|v| f32::from(v) / s))
    }

    /// Die temperature in °C: `raw / 340 + 36.53`.
    pub fn temperature_c(&mut self) -> Result<f32, Error<I2C::Error>> {
        self.ensure_init()?;
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(self.addr, &[REG_TEMP_OUT_H], &mut buf)?;
        let raw = i16::from_be_bytes(buf);
        Ok(f32::from(raw) / TEMP_LSB_PER_DEG_C + TEMP_OFFSET_DEG_C)
    }

    // --- Register access ----------------------------------------------------

    fn ensure_init(&self) -> Result<(), Error<I2C::Error>> {
        if self.initialised {
            Ok(())
        } else {
            Err(Error::NotInitialised)
        }
    }

    fn read_reg(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0u8];
        self.i2c.write_read(self.addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.addr, &[reg, value])?;
        Ok(())
    }

    /// Read-modify-write: replace the bits under `mask` with `bits`, leave the rest.
    fn modify_reg(&mut self, reg: u8, mask: u8, bits: u8) -> Result<(), Error<I2C::Error>> {
        let current = self.read_reg(reg)?;
        self.write_reg(reg, (current & !mask) | (bits & mask))
    }

    fn read_vec3(&mut self, start: u8) -> Result<[i16; 3], Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.i2c.write_read(self.addr, &[start], &mut buf)?;
        Ok([
            i16::from_be_bytes([buf[0], buf[1]]),
            i16::from_be_bytes([buf[2], buf[3]]),
            i16::from_be_bytes([buf[4], buf[5]]),
        ])
    }
}
