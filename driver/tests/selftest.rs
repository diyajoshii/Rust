//! Hardware self-test: factory-trim arithmetic and the measurement sequence.
#![cfg(feature = "mock")]

mod common;
use common::*;

use mpu6050_nostd::mock::{Fault, MockDelay, MockI2c};
use mpu6050_nostd::registers::*;
use mpu6050_nostd::selftest::{
    ACCEL_TRIM_TABLE, DEFAULT_SELF_TEST_TOLERANCE_PERCENT, GYRO_TRIM_TABLE, SELF_TEST_SAMPLES,
    SELF_TEST_SETTLE_MS, SelfTestValues, accel_factory_trim, gyro_factory_trim,
};
use mpu6050_nostd::{AccelRange, Error, GyroRange, Mpu6050};

const TOL: f32 = DEFAULT_SELF_TEST_TOLERANCE_PERCENT;

/// Program the part's test values: gyro n = (gx, gy, gz), accel n = (ax, ay, az).
fn program_trim(imu: &mut Mpu6050<MockI2c>, g: [u8; 3], a: [u8; 3]) {
    let regs = [
        ((a[0] >> 2) << 5) | (g[0] & 0x1F),
        ((a[1] >> 2) << 5) | (g[1] & 0x1F),
        ((a[2] >> 2) << 5) | (g[2] & 0x1F),
        ((a[0] & 3) << 4) | ((a[1] & 3) << 2) | (a[2] & 3),
    ];
    imu.bus_mut().set_registers(REG_SELF_TEST_X, &regs);
}

/// Make the mock's actuation produce exactly the factory-trim response.
fn respond_at_trim(imu: &mut Mpu6050<MockI2c>, g: [u8; 3], a: [u8; 3]) {
    let gr = [
        gyro_factory_trim(g[0]).round() as i16,
        -(gyro_factory_trim(g[1]).round() as i16),
        gyro_factory_trim(g[2]).round() as i16,
    ];
    let ar = a.map(|n| accel_factory_trim(n).round() as i16);
    imu.bus_mut().set_self_test_response(gr, ar);
}

// --- Arithmetic -------------------------------------------------------------

#[test]
fn trim_tables_match_the_datasheet_exponentials() {
    for n in 1..=31usize {
        let g_expected = 1.046f32.powf((n - 1) as f32);
        let a_expected = (0.92f32 / 0.34).powf((n - 1) as f32 / 30.0);
        let g = GYRO_TRIM_TABLE[n - 1];
        let a = ACCEL_TRIM_TABLE[n - 1];
        assert!(
            (g - g_expected).abs() / g_expected < 1e-5,
            "gyro n={n}: {g} vs {g_expected}"
        );
        assert!(
            (a - a_expected).abs() / a_expected < 1e-5,
            "accel n={n}: {a} vs {a_expected}"
        );
    }
}

#[test]
fn factory_trim_endpoints_match_the_formulas() {
    assert_close(gyro_factory_trim(1), 25.0 * 131.0);
    assert_close(gyro_factory_trim(31), 25.0 * 131.0 * 1.046f32.powf(30.0));
    assert_close(accel_factory_trim(1), 4096.0 * 0.34);
    // (0.92/0.34)^(30/30) = 0.92/0.34, so FT = 4096 * 0.92.
    assert_close(accel_factory_trim(31), 4096.0 * 0.92);
    assert_eq!(gyro_factory_trim(0), 0.0);
    assert_eq!(accel_factory_trim(0), 0.0);
    // Only five bits are meaningful.
    assert_eq!(gyro_factory_trim(0xE0 | 5), gyro_factory_trim(5));
}

#[test]
fn test_values_decode_by_concatenating_the_split_accel_fields() {
    // X: XA_TEST = 0b10110 (22) → top bits 101 in reg 13[7:5], low bits 10 in reg 16[5:4]
    // Y: YA_TEST = 0b00001 (1)  → top 000, low 01 in reg 16[3:2]
    // Z: ZA_TEST = 0b11111 (31) → top 111, low 11 in reg 16[1:0]
    // gyro: 7, 0, 31
    let regs = [
        0b101_00111, // XA hi=101, XG=7
        0b000_00000, // YA hi=000, YG=0
        0b111_11111, // ZA hi=111, ZG=31
        0b00_10_01_11,
    ];
    let v = SelfTestValues::decode(regs);
    assert_eq!(v.gyro, [7, 0, 31]);
    assert_eq!(v.accel, [22, 1, 31]);
}

// --- Sequence ---------------------------------------------------------------

#[test]
fn self_test_uses_the_datasheet_ranges_and_toggles_actuation_on_then_off() {
    let mut imu = ready();
    imu.set_gyro_range(GyroRange::Dps1000).unwrap();
    imu.set_accel_range(AccelRange::G16).unwrap();
    program_trim(&mut imu, [10, 10, 10], [10, 10, 10]);
    respond_at_trim(&mut imu, [10, 10, 10], [10, 10, 10]);
    imu.bus_mut().take_log();

    let mut d = MockDelay::default();
    imu.self_test(&mut d, TOL).unwrap();

    let gyro_writes = writes_to(imu.bus().log(), REG_GYRO_CONFIG);
    let accel_writes = writes_to(imu.bus().log(), REG_ACCEL_CONFIG);
    // 1: ±250 / ±8 g, ST off. 2: ST on. 3: restore Dps1000 / G16, ST off.
    assert_eq!(
        gyro_writes,
        vec![
            vec![GyroRange::Dps250.config_bits()],
            vec![GYRO_CONFIG_ST_ALL | GyroRange::Dps250.config_bits()],
            vec![GyroRange::Dps1000.config_bits()],
        ]
    );
    assert_eq!(
        accel_writes,
        vec![
            vec![AccelRange::G8.config_bits()],
            vec![ACCEL_CONFIG_ST_ALL | AccelRange::G8.config_bits()],
            vec![AccelRange::G16.config_bits()],
        ]
    );
    // Struct state untouched; device matches it.
    assert_eq!(imu.gyro_range(), GyroRange::Dps1000);
    assert_eq!(imu.accel_range(), AccelRange::G16);
    assert_eq!(imu.bus().register(REG_GYRO_CONFIG) & 0xE0, 0);
    assert_eq!(imu.bus().register(REG_ACCEL_CONFIG) & 0xE0, 0);
}

#[test]
fn self_test_waits_after_each_toggle_and_averages_the_configured_samples() {
    let mut imu = ready();
    program_trim(&mut imu, [5, 5, 5], [5, 5, 5]);
    respond_at_trim(&mut imu, [5, 5, 5], [5, 5, 5]);
    imu.bus_mut().take_log();
    let mut d = MockDelay::default();
    imu.self_test(&mut d, TOL).unwrap();
    assert_eq!(d.calls, 2);
    assert_eq!(d.total_ns, 2 * u64::from(SELF_TEST_SETTLE_MS) * 1_000_000);
    // Two measurement phases × SELF_TEST_SAMPLES × (gyro burst + accel burst).
    assert_eq!(
        reads_of(imu.bus().log(), REG_GYRO_XOUT_H),
        2 * SELF_TEST_SAMPLES
    );
    assert_eq!(
        reads_of(imu.bus().log(), REG_ACCEL_XOUT_H),
        2 * SELF_TEST_SAMPLES
    );
    // Trim registers read once, as one 4-byte burst.
    assert_eq!(reads_of(imu.bus().log(), REG_SELF_TEST_X), 1);
}

#[test]
fn a_part_responding_at_factory_trim_passes_with_near_zero_deviation() {
    let mut imu = ready();
    let g = [12, 20, 3];
    let a = [17, 1, 31];
    program_trim(&mut imu, g, a);
    respond_at_trim(&mut imu, g, a);
    let mut d = MockDelay::default();
    let r = imu.self_test(&mut d, TOL).unwrap();

    assert!(r.passed(), "{r:?}");
    assert_eq!(r.values.gyro, g);
    assert_eq!(r.values.accel, a);
    // Y gyro trim is negative on the part, and the response was negative too.
    assert!(r.gyro_factory_trim[1] < 0.0);
    assert!(r.gyro_response[1] < 0);
    for dev in r
        .gyro_deviation_percent
        .iter()
        .chain(&r.accel_deviation_percent)
    {
        let d = dev.expect("all trims defined");
        assert!(d.abs() < 0.1, "rounding only: {d}");
    }
}

#[test]
fn a_part_thirty_percent_off_fails_on_exactly_the_bad_axes() {
    let mut imu = ready();
    let g = [8, 8, 8];
    let a = [8, 8, 8];
    program_trim(&mut imu, g, a);
    let ft_g = gyro_factory_trim(8);
    let ft_a = accel_factory_trim(8);
    // Gyro Z and accel X respond 30 % low; everything else is on trim.
    imu.bus_mut().set_self_test_response(
        [ft_g as i16, -(ft_g as i16), (ft_g * 0.7) as i16],
        [(ft_a * 0.7) as i16, ft_a as i16, ft_a as i16],
    );
    let mut d = MockDelay::default();
    let r = imu.self_test(&mut d, TOL).unwrap();
    assert!(!r.passed());
    assert_eq!(r.gyro_passed(), [true, true, false]);
    assert_eq!(r.accel_passed(), [false, true, true]);
    assert!(r.gyro_deviation_percent[2].unwrap() < -25.0);
    assert!(r.accel_deviation_percent[0].unwrap() < -25.0);
}

#[test]
fn a_zero_test_value_has_no_criterion_and_counts_as_failed() {
    let mut imu = ready();
    program_trim(&mut imu, [0, 9, 9], [9, 9, 0]);
    respond_at_trim(&mut imu, [0, 9, 9], [9, 9, 0]);
    let mut d = MockDelay::default();
    let r = imu.self_test(&mut d, TOL).unwrap();
    assert_eq!(r.gyro_deviation_percent[0], None);
    assert_eq!(r.accel_deviation_percent[2], None);
    assert_eq!(r.gyro_passed(), [false, true, true]);
    assert_eq!(r.accel_passed(), [true, true, false]);
    assert!(!r.passed());
}

#[test]
fn tolerance_is_the_callers_and_is_reported_back() {
    let mut imu = ready();
    program_trim(&mut imu, [8, 8, 8], [8, 8, 8]);
    let ft_g = gyro_factory_trim(8);
    let ft_a = accel_factory_trim(8);
    // Everything 10 % high.
    imu.bus_mut().set_self_test_response(
        [
            (ft_g * 1.1) as i16,
            -((ft_g * 1.1) as i16),
            (ft_g * 1.1) as i16,
        ],
        [
            (ft_a * 1.1) as i16,
            (ft_a * 1.1) as i16,
            (ft_a * 1.1) as i16,
        ],
    );
    let mut d = MockDelay::default();
    let strict = imu.self_test(&mut d, 5.0).unwrap();
    let lenient = imu.self_test(&mut d, 14.0).unwrap();
    assert!(!strict.passed());
    assert!(lenient.passed());
    assert_eq!(strict.tolerance_percent, 5.0);
    assert_eq!(lenient.tolerance_percent, 14.0);
}

#[test]
fn self_test_requires_init() {
    let mut imu = Mpu6050::new(MockI2c::new(), ADDR_AD0_LOW);
    let mut d = MockDelay::default();
    assert_eq!(imu.self_test(&mut d, TOL), Err(Error::NotInitialised));
    assert!(imu.bus().log().is_empty());
}

#[test]
fn a_bus_error_mid_test_still_switches_actuation_off_and_restores_ranges() {
    let mut imu = ready();
    imu.set_gyro_range(GyroRange::Dps500).unwrap();
    program_trim(&mut imu, [8, 8, 8], [8, 8, 8]);
    respond_at_trim(&mut imu, [8, 8, 8], [8, 8, 8]);
    // Fail one of the reads in the second (actuated) measurement phase:
    // 4 RMW config txns + 2*SAMPLES reads + 4 RMW + a few reads in.
    let next = imu.bus().transactions_seen();
    let fail_at = next + 4 + 2 * SELF_TEST_SAMPLES + 4 + 3;
    imu.bus_mut().inject(Fault::NackAfter(fail_at));
    let mut d = MockDelay::default();
    let r = imu.self_test(&mut d, TOL);
    assert!(matches!(r, Err(Error::Bus(_))), "{r:?}");
    // Actuation bits are off and the struct's range is back on the device.
    assert_eq!(imu.bus().register(REG_GYRO_CONFIG) & 0xE0, 0);
    assert_eq!(imu.bus().register(REG_ACCEL_CONFIG) & 0xE0, 0);
    assert_eq!(
        imu.bus().register(REG_GYRO_CONFIG) & 0x18,
        GyroRange::Dps500.config_bits()
    );
    assert_eq!(imu.gyro_range(), GyroRange::Dps500);
}
