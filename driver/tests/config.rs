//! Range, filter and sample-rate configuration — what lands in which register.
#![cfg(feature = "mock")]

mod common;
use common::*;

use mpu6050_nostd::mock::MockI2c;
use mpu6050_nostd::registers::*;
use mpu6050_nostd::{AccelRange, DlpfConfig, GyroRange, Mpu6050};

#[test]
fn each_accel_range_writes_afs_sel() {
    for (range, bits) in [
        (AccelRange::G2, 0b00_000),
        (AccelRange::G4, 0b01_000),
        (AccelRange::G8, 0b10_000),
        (AccelRange::G16, 0b11_000),
    ] {
        let mut imu = ready();
        imu.set_accel_range(range).unwrap();
        assert_eq!(imu.bus().register(REG_ACCEL_CONFIG), bits, "{range:?}");
        assert_eq!(
            writes_to(imu.bus().log(), REG_ACCEL_CONFIG),
            vec![vec![bits]]
        );
    }
}

#[test]
fn each_gyro_range_writes_fs_sel() {
    for (range, bits) in [
        (GyroRange::Dps250, 0b00_000),
        (GyroRange::Dps500, 0b01_000),
        (GyroRange::Dps1000, 0b10_000),
        (GyroRange::Dps2000, 0b11_000),
    ] {
        let mut imu = ready();
        imu.set_gyro_range(range).unwrap();
        assert_eq!(imu.bus().register(REG_GYRO_CONFIG), bits, "{range:?}");
    }
}

#[test]
fn each_dlpf_setting_writes_dlpf_cfg() {
    for dlpf in [
        DlpfConfig::Hz260,
        DlpfConfig::Hz184,
        DlpfConfig::Hz94,
        DlpfConfig::Hz44,
        DlpfConfig::Hz21,
        DlpfConfig::Hz10,
        DlpfConfig::Hz5,
    ] {
        let mut imu = ready();
        imu.set_dlpf(dlpf).unwrap();
        assert_eq!(
            imu.bus().register(REG_CONFIG) & 0x07,
            dlpf as u8,
            "{dlpf:?}"
        );
    }
}

#[test]
fn setters_preserve_neighbouring_bits() {
    let mut imu = ready();
    // Self-test bits [7:5] set out of band, as if something else owned them.
    imu.bus_mut().set_register(REG_ACCEL_CONFIG, 0b1110_0000);
    imu.bus_mut().set_register(REG_GYRO_CONFIG, 0b1110_0000);
    // EXT_SYNC_SET [5:3] set out of band.
    imu.bus_mut().set_register(REG_CONFIG, 0b0010_1000);

    imu.set_accel_range(AccelRange::G16).unwrap();
    imu.set_gyro_range(GyroRange::Dps1000).unwrap();
    imu.set_dlpf(DlpfConfig::Hz44).unwrap();

    assert_eq!(imu.bus().register(REG_ACCEL_CONFIG), 0b1111_1000);
    assert_eq!(imu.bus().register(REG_GYRO_CONFIG), 0b1111_0000);
    assert_eq!(imu.bus().register(REG_CONFIG), 0b0010_1011);
}

#[test]
fn configured_state_survives_a_setter_round_trip() {
    let mut imu = ready();
    imu.set_accel_range(AccelRange::G8).unwrap();
    imu.set_gyro_range(GyroRange::Dps2000).unwrap();
    imu.set_dlpf(DlpfConfig::Hz21).unwrap();
    assert_eq!(imu.accel_range(), AccelRange::G8);
    assert_eq!(imu.gyro_range(), GyroRange::Dps2000);
    assert_eq!(imu.dlpf(), DlpfConfig::Hz21);
}

#[test]
fn config_bits_shift_into_position_exactly_once() {
    assert_eq!(AccelRange::G2.config_bits(), 0x00);
    assert_eq!(AccelRange::G16.config_bits(), 0x18);
    assert_eq!(GyroRange::Dps250.config_bits(), 0x00);
    assert_eq!(GyroRange::Dps2000.config_bits(), 0x18);
}

#[test]
fn device_at_ad0_high_reports_the_same_identity() {
    let mut imu = Mpu6050::new(MockI2c::with_address(ADDR_AD0_HIGH), ADDR_AD0_HIGH);
    assert_eq!(imu.who_am_i(), Ok(WHO_AM_I_VALUE));
    imu.init().unwrap();
}

#[test]
fn init_reapplies_ranges_held_in_the_struct() {
    // A driver constructed and configured, then re-initialised, must push the
    // struct's ranges back to a freshly reset device.
    let mut imu = ready();
    imu.set_accel_range(AccelRange::G4).unwrap();
    imu.set_gyro_range(GyroRange::Dps500).unwrap();
    imu.set_dlpf(DlpfConfig::Hz94).unwrap();
    imu.bus_mut().take_log();
    imu.init().unwrap();
    assert_eq!(
        writes_to(imu.bus().log(), REG_CONFIG),
        vec![vec![
            DlpfConfig::Hz94 as u8,
            GyroRange::Dps500.config_bits(),
            AccelRange::G4.config_bits()
        ]]
    );
}
