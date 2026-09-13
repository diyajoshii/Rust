//! Hardware self-test — Registers 13–16, with the actuation bits in
//! Registers 27 and 28.
//!
//! The part can drive each sensor's proof mass electrically and report the
//! resulting output. Comparing that *self-test response* (STR) against a
//! per-part *factory trim* (FT) value burned into Registers 13–16 tells you
//! whether the mechanical and electrical path is still within spec — without
//! moving the board.
//!
//! Everything here is quoted in `docs/datasheet-notes.md`. In short:
//!
//! - `STR = output with self-test enabled − output with self-test disabled`
//! - `deviation % = (STR − FT) / FT × 100`
//! - Gyro FT: `±25 · 131 · 1.046^(n−1)` at **±250 dps**, negative on Y.
//! - Accel FT: `4096 · 0.34 · (0.92/0.34)^((n−1)/30)` at **±8 g**.
//! - `n` is the 5-bit test value; `n = 0` means FT is undefined.
//!
//! The pass limit is not in the register map — it defers to the Product
//! Specification — so the tolerance is a parameter, with a documented default.
//!
//! `core` has no `powf` and `n` is a 5-bit integer, so the two exponentials
//! are 31-entry tables computed offline and checked against `f32::powf` in a
//! host test.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::registers::{
    ACCEL_CONFIG_ST_ALL, GYRO_CONFIG_ST_ALL, REG_ACCEL_CONFIG, REG_ACCEL_XOUT_H, REG_GYRO_CONFIG,
    REG_GYRO_XOUT_H, REG_SELF_TEST_X,
};
use crate::{AccelRange, Error, GyroRange, Mpu6050};

/// `(gyro, accel)` per-axis values in LSB.
type Vec3Pair = ([i32; 3], [i32; 3]);

/// Pass/fail tolerance the driver suggests when the caller has no better
/// figure. **This is a default, not a datasheet quotation** — the register
/// map defers the limit to the Product Specification. Confirm it there
/// before treating a pass as authoritative.
pub const DEFAULT_SELF_TEST_TOLERANCE_PERCENT: f32 = 14.0;

/// Settling time after toggling the self-test bits. Not specified in the
/// register map; 50 ms is conservative.
pub const SELF_TEST_SETTLE_MS: u32 = 50;

/// Samples averaged for each of the two measurements.
pub const SELF_TEST_SAMPLES: usize = 8;

/// `25 · 131`: gyro FT at `n = 1`, ±250 dps sensitivity.
const GYRO_FT_BASE: f32 = 25.0 * 131.0;
/// `4096 · 0.34`: accel FT at `n = 1`, ±8 g sensitivity.
const ACCEL_FT_BASE: f32 = 4096.0 * 0.34;

/// `1.046^(n−1)` for `n = 1..=31`. Index with `n − 1`.
pub const GYRO_TRIM_TABLE: [f32; 31] = [
    1.0, 1.046, 1.094116, 1.1444453, 1.1970898, 1.2521559, 1.3097551, 1.3700038, 1.433024,
    1.4989431, 1.5678946, 1.6400176, 1.7154585, 1.7943696, 1.8769106, 1.9632485, 2.0535579,
    2.1480215, 2.2468305, 2.3501847, 2.4582932, 2.5713747, 2.689658, 2.8133821, 2.942798,
    3.0781665, 3.219762, 3.3678713, 3.5227933, 3.6848419, 3.8543446,
];

/// `(0.92 / 0.34)^((n−1) / 30)` for `n = 1..=31`. Index with `n − 1`.
pub const ACCEL_TRIM_TABLE: [f32; 31] = [
    1.0, 1.0337375, 1.0686133, 1.1046658, 1.1419345, 1.1804606, 1.2202865, 1.2614559, 1.3040143,
    1.3480086, 1.3934872, 1.4405, 1.489099, 1.5393375, 1.591271, 1.6449566, 1.7004535, 1.7578226,
    1.8171272, 1.8784328, 1.9418064, 2.0073183, 2.0750403, 2.1450472, 2.2174158, 2.2922258,
    2.36956, 2.4495032, 2.5321436, 2.6175718, 2.7058823,
];

/// Gyroscope factory-trim magnitude for a 5-bit test value, in LSB at ±250 dps.
/// Returns `0.0` for `n = 0`, where the datasheet defines no value. The Y axis
/// is negative on the part; see [`SelfTestReport`].
pub fn gyro_factory_trim(test: u8) -> f32 {
    let n = usize::from(test & 0x1F);
    if n == 0 {
        0.0
    } else {
        GYRO_FT_BASE * GYRO_TRIM_TABLE[n - 1]
    }
}

/// Accelerometer factory trim for a 5-bit test value, in LSB at ±8 g.
/// Returns `0.0` for `n = 0`.
pub fn accel_factory_trim(test: u8) -> f32 {
    let n = usize::from(test & 0x1F);
    if n == 0 {
        0.0
    } else {
        ACCEL_FT_BASE * ACCEL_TRIM_TABLE[n - 1]
    }
}

/// The six 5-bit test values unpacked from Registers 13–16.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SelfTestValues {
    /// `XG_TEST`, `YG_TEST`, `ZG_TEST` — the low five bits of Registers 13–15.
    pub gyro: [u8; 3],
    /// `XA_TEST`, `YA_TEST`, `ZA_TEST` — bits 7:5 of Registers 13–15
    /// concatenated with the matching 2-bit field of Register 16.
    pub accel: [u8; 3],
}

impl SelfTestValues {
    /// Unpack `[SELF_TEST_X, SELF_TEST_Y, SELF_TEST_Z, SELF_TEST_A]`.
    pub fn decode(regs: [u8; 4]) -> Self {
        let [x, y, z, a] = regs;
        Self {
            gyro: [x & 0x1F, y & 0x1F, z & 0x1F],
            accel: [
                ((x >> 5) << 2) | ((a >> 4) & 0x03),
                ((y >> 5) << 2) | ((a >> 2) & 0x03),
                ((z >> 5) << 2) | (a & 0x03),
            ],
        }
    }
}

/// Everything measured and derived during one self-test run.
///
/// Deviations are `None` where the part's test value is 0, for which the
/// datasheet defines no factory trim and therefore no criterion; such an
/// axis counts as failed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SelfTestReport {
    /// Unpacked test values from Registers 13–16.
    pub values: SelfTestValues,
    /// Gyro STR per axis: mean output with actuation minus mean without, LSB at ±250 dps.
    pub gyro_response: [i32; 3],
    /// Accel STR per axis, LSB at ±8 g.
    pub accel_response: [i32; 3],
    /// Gyro FT per axis, sign included (Y is negative).
    pub gyro_factory_trim: [f32; 3],
    /// Accel FT per axis.
    pub accel_factory_trim: [f32; 3],
    /// `(STR − FT) / FT × 100` per gyro axis.
    pub gyro_deviation_percent: [Option<f32>; 3],
    /// `(STR − FT) / FT × 100` per accel axis.
    pub accel_deviation_percent: [Option<f32>; 3],
    /// The tolerance the pass/fail flags were judged against.
    pub tolerance_percent: f32,
}

impl SelfTestReport {
    fn within(dev: Option<f32>, tol: f32) -> bool {
        matches!(dev, Some(d) if d.abs() <= tol)
    }

    /// Per-axis gyro pass flags.
    pub fn gyro_passed(&self) -> [bool; 3] {
        self.gyro_deviation_percent
            .map(|d| Self::within(d, self.tolerance_percent))
    }

    /// Per-axis accel pass flags.
    pub fn accel_passed(&self) -> [bool; 3] {
        self.accel_deviation_percent
            .map(|d| Self::within(d, self.tolerance_percent))
    }

    /// True only if all six axes are within tolerance.
    pub fn passed(&self) -> bool {
        self.gyro_passed().iter().all(|p| *p) && self.accel_passed().iter().all(|p| *p)
    }
}

fn deviation(response: i32, trim: f32) -> Option<f32> {
    if trim == 0.0 {
        None
    } else {
        Some((response as f32 - trim) / trim * 100.0)
    }
}

impl<I2C: I2c> Mpu6050<I2C> {
    /// Run the hardware self-test on all six axes.
    ///
    /// Sequence, each step visible on the bus:
    ///
    /// 1. Set ±250 dps and ±8 g (the ranges the FT formulas assume), self-test off.
    /// 2. Wait [`SELF_TEST_SETTLE_MS`]; average [`SELF_TEST_SAMPLES`] raw reads.
    /// 3. Set all six self-test bits; wait; average again.
    /// 4. Restore the ranges held in the struct, self-test off. **This step
    ///    runs even if an earlier one failed**, so a bus error mid-test does
    ///    not leave the part actuating.
    /// 5. Read Registers 13–16 in one burst; compute FT and deviation.
    ///
    /// `tolerance_percent` is the pass limit — see
    /// [`DEFAULT_SELF_TEST_TOLERANCE_PERCENT`] for why it is an argument.
    /// `delay` is used only here; nothing else in the driver needs a timer.
    pub fn self_test<D: DelayNs>(
        &mut self,
        delay: &mut D,
        tolerance_percent: f32,
    ) -> Result<SelfTestReport, Error<I2C::Error>> {
        self.ensure_init()?;
        let measured = self.self_test_measure(delay);
        let restored = self.self_test_restore();
        let (gyro_response, accel_response) = measured?;
        restored?;

        let mut regs = [0u8; 4];
        self.i2c
            .write_read(self.addr, &[REG_SELF_TEST_X], &mut regs)?;
        let values = SelfTestValues::decode(regs);

        let gyro_factory_trim = [
            gyro_factory_trim(values.gyro[0]),
            -gyro_factory_trim(values.gyro[1]),
            gyro_factory_trim(values.gyro[2]),
        ];
        let accel_factory_trim = values.accel.map(accel_factory_trim);

        let mut gyro_deviation_percent = [None; 3];
        let mut accel_deviation_percent = [None; 3];
        for i in 0..3 {
            gyro_deviation_percent[i] = deviation(gyro_response[i], gyro_factory_trim[i]);
            accel_deviation_percent[i] = deviation(accel_response[i], accel_factory_trim[i]);
        }

        Ok(SelfTestReport {
            values,
            gyro_response,
            accel_response,
            gyro_factory_trim,
            accel_factory_trim,
            gyro_deviation_percent,
            accel_deviation_percent,
            tolerance_percent,
        })
    }

    /// Steps 1–3: returns (gyro STR, accel STR).
    fn self_test_measure<D: DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<Vec3Pair, Error<I2C::Error>> {
        const CONFIG_MASK: u8 = 0b1111_1000; // self-test bits + range field
        let gyro_off = GyroRange::Dps250.config_bits();
        let accel_off = AccelRange::G8.config_bits();

        self.modify_reg(REG_GYRO_CONFIG, CONFIG_MASK, gyro_off)?;
        self.modify_reg(REG_ACCEL_CONFIG, CONFIG_MASK, accel_off)?;
        delay.delay_ms(SELF_TEST_SETTLE_MS);
        let (g0, a0) = self.average_raw()?;

        self.modify_reg(REG_GYRO_CONFIG, CONFIG_MASK, GYRO_CONFIG_ST_ALL | gyro_off)?;
        self.modify_reg(
            REG_ACCEL_CONFIG,
            CONFIG_MASK,
            ACCEL_CONFIG_ST_ALL | accel_off,
        )?;
        delay.delay_ms(SELF_TEST_SETTLE_MS);
        let (g1, a1) = self.average_raw()?;

        let mut gyro = [0i32; 3];
        let mut accel = [0i32; 3];
        for i in 0..3 {
            gyro[i] = g1[i] - g0[i];
            accel[i] = a1[i] - a0[i];
        }
        Ok((gyro, accel))
    }

    /// Step 4: self-test bits off, ranges back to what the struct says.
    fn self_test_restore(&mut self) -> Result<(), Error<I2C::Error>> {
        const CONFIG_MASK: u8 = 0b1111_1000;
        self.modify_reg(REG_GYRO_CONFIG, CONFIG_MASK, self.gyro_range.config_bits())?;
        self.modify_reg(
            REG_ACCEL_CONFIG,
            CONFIG_MASK,
            self.accel_range.config_bits(),
        )
    }

    /// Mean of [`SELF_TEST_SAMPLES`] gyro and accel bursts, in LSB.
    fn average_raw(&mut self) -> Result<Vec3Pair, Error<I2C::Error>> {
        let mut g = [0i32; 3];
        let mut a = [0i32; 3];
        for _ in 0..SELF_TEST_SAMPLES {
            let gs = self.read_vec3(REG_GYRO_XOUT_H)?;
            let as_ = self.read_vec3(REG_ACCEL_XOUT_H)?;
            for i in 0..3 {
                g[i] += i32::from(gs[i]);
                a[i] += i32::from(as_[i]);
            }
        }
        let n = SELF_TEST_SAMPLES as i32;
        Ok((g.map(|v| v / n), a.map(|v| v / n)))
    }
}
