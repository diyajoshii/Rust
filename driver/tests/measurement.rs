//! Raw reads and unit conversion.
#![cfg(feature = "mock")]

mod common;
use common::*;

use mpu6050_nostd::mock::Transaction;
use mpu6050_nostd::registers::*;
use mpu6050_nostd::{AccelRange, DlpfConfig, Error, GyroRange};

const ACCEL_RANGES: [AccelRange; 4] = [
    AccelRange::G2,
    AccelRange::G4,
    AccelRange::G8,
    AccelRange::G16,
];
const GYRO_RANGES: [GyroRange; 4] = [
    GyroRange::Dps250,
    GyroRange::Dps500,
    GyroRange::Dps1000,
    GyroRange::Dps2000,
];

#[test]
fn accel_conversion_matches_the_sensitivity_table_at_every_range() {
    // 16384 counts is exactly 1 g at ±2 g, 2 g at ±4 g, 4 g at ±8 g, 8 g at ±16 g.
    for (range, expected_g) in ACCEL_RANGES.iter().zip([1.0f32, 2.0, 4.0, 8.0]) {
        let mut imu = ready();
        imu.set_accel_range(*range).unwrap();
        imu.bus_mut()
            .set_registers(REG_ACCEL_XOUT_H, &be6(16_384, -16_384, 8_192));
        let [x, y, z] = imu.accel_g().unwrap();
        assert_close(x, expected_g);
        assert_close(y, -expected_g);
        assert_close(z, expected_g / 2.0);
    }
}

#[test]
fn gyro_conversion_matches_the_sensitivity_table_at_every_range() {
    // 131 counts is 1 °/s at ±250; sensitivities halve with each range.
    let sens = [131.0f32, 65.5, 32.8, 16.4];
    for (range, s) in GYRO_RANGES.iter().zip(sens) {
        let mut imu = ready();
        imu.set_gyro_range(*range).unwrap();
        imu.bus_mut()
            .set_registers(REG_GYRO_XOUT_H, &be6(131, -262, 1_310));
        let [x, y, z] = imu.gyro_dps().unwrap();
        assert_close(x, 131.0 / s);
        assert_close(y, -262.0 / s);
        assert_close(z, 1_310.0 / s);
    }
}

#[test]
fn twos_complement_edges_decode_correctly() {
    let mut imu = ready();
    imu.bus_mut()
        .set_registers(REG_ACCEL_XOUT_H, &[0xFF, 0xFF, 0x80, 0x00, 0x7F, 0xFF]);
    assert_eq!(imu.accel_raw().unwrap(), [-1, i16::MIN, i16::MAX]);
    imu.bus_mut()
        .set_registers(REG_GYRO_XOUT_H, &[0x00, 0x00, 0x00, 0x01, 0xFF, 0xFE]);
    assert_eq!(imu.gyro_raw().unwrap(), [0, 1, -2]);
}

#[test]
fn temperature_follows_the_datasheet_formula() {
    let mut imu = ready();
    // raw = 0 → 36.53 °C
    imu.bus_mut().set_registers(REG_TEMP_OUT_H, &[0x00, 0x00]);
    assert_close(imu.temperature_c().unwrap(), 36.53);
    // raw = 340 (0x0154) → 37.53 °C
    imu.bus_mut().set_registers(REG_TEMP_OUT_H, &[0x01, 0x54]);
    assert_close(imu.temperature_c().unwrap(), 37.53);
    // raw = -3400 → 26.53 °C
    imu.bus_mut()
        .set_registers(REG_TEMP_OUT_H, &(-3400i16).to_be_bytes());
    assert_close(imu.temperature_c().unwrap(), 26.53);
}

#[test]
fn changing_range_changes_the_scale_applied_to_the_same_counts() {
    let mut imu = ready();
    imu.bus_mut()
        .set_registers(REG_ACCEL_XOUT_H, &be6(4_096, 0, 0));
    let [before, ..] = imu.accel_g().unwrap();
    imu.set_accel_range(AccelRange::G8).unwrap();
    let [after, ..] = imu.accel_g().unwrap();
    assert_close(before, 0.25);
    assert_close(after, 1.0);
}

#[test]
fn accel_raw_is_exactly_one_six_byte_burst() {
    let mut imu = ready();
    imu.accel_raw().unwrap();
    let log = imu.bus().log();
    assert_eq!(log.len(), 1, "expected a single transaction, got {log:?}");
    assert!(matches!(
        &log[0],
        Transaction::WriteRead { sent, received, .. }
            if sent.as_slice() == [REG_ACCEL_XOUT_H] && received.len() == 6
    ));
}

#[test]
fn gyro_raw_is_exactly_one_six_byte_burst() {
    let mut imu = ready();
    imu.gyro_raw().unwrap();
    let log = imu.bus().log();
    assert_eq!(log.len(), 1);
    assert!(matches!(
        &log[0],
        Transaction::WriteRead { sent, received, .. }
            if sent.as_slice() == [REG_GYRO_XOUT_H] && received.len() == 6
    ));
}

#[test]
fn temperature_is_exactly_one_two_byte_burst() {
    let mut imu = ready();
    imu.temperature_c().unwrap();
    let log = imu.bus().log();
    assert_eq!(log.len(), 1);
    assert!(matches!(
        &log[0],
        Transaction::WriteRead { sent, received, .. }
            if sent.as_slice() == [REG_TEMP_OUT_H] && received.len() == 2
    ));
}

#[test]
fn sample_rate_divider_depends_on_dlpf_state() {
    // DLPF off: base 8 kHz → 100 Hz needs SMPLRT_DIV = 79.
    let mut off = ready();
    assert_eq!(off.dlpf(), DlpfConfig::Hz260);
    assert_eq!(off.set_sample_rate_hz(100), Ok(100));
    assert_eq!(off.bus().register(REG_SMPLRT_DIV), 79);

    // DLPF on: base 1 kHz → 100 Hz needs SMPLRT_DIV = 9. Eight times smaller.
    let mut on = ready();
    on.set_dlpf(DlpfConfig::Hz44).unwrap();
    assert_eq!(on.set_sample_rate_hz(100), Ok(100));
    assert_eq!(on.bus().register(REG_SMPLRT_DIV), 9);
}

#[test]
fn sample_rate_returns_the_rate_actually_achieved() {
    let mut imu = ready();
    imu.set_dlpf(DlpfConfig::Hz94).unwrap();
    // 1000 / 300 is not integral; nearest divider is 2 → 333 Hz.
    assert_eq!(imu.set_sample_rate_hz(300), Ok(333));
    assert_eq!(imu.bus().register(REG_SMPLRT_DIV), 2);
    assert_eq!(imu.set_sample_rate_hz(1000), Ok(1000));
    assert_eq!(imu.bus().register(REG_SMPLRT_DIV), 0);
}

#[test]
fn unreachable_sample_rates_are_refused_without_touching_the_bus() {
    let mut imu = ready();
    imu.set_dlpf(DlpfConfig::Hz44).unwrap();
    imu.bus_mut().take_log();
    // Above the 1 kHz base.
    assert_eq!(
        imu.set_sample_rate_hz(2000),
        Err(Error::SampleRateUnreachable(2000))
    );
    // Below base / 256.
    assert_eq!(
        imu.set_sample_rate_hz(1),
        Err(Error::SampleRateUnreachable(1))
    );
    assert_eq!(
        imu.set_sample_rate_hz(0),
        Err(Error::SampleRateUnreachable(0))
    );
    assert!(imu.bus().log().is_empty());
}

#[test]
fn eight_khz_base_reaches_rates_the_filtered_base_cannot() {
    let mut imu = ready(); // DLPF off → 8 kHz base
    assert_eq!(imu.set_sample_rate_hz(4000), Ok(4000));
    assert_eq!(imu.bus().register(REG_SMPLRT_DIV), 1);
    assert_eq!(imu.set_sample_rate_hz(8000), Ok(8000));
    assert_eq!(imu.bus().register(REG_SMPLRT_DIV), 0);
}
