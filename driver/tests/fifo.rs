//! FIFO configuration, counting, draining and overflow handling.
#![cfg(feature = "mock")]

mod common;
use common::*;

use mpu6050_nostd::fifo::MAX_BURST;
use mpu6050_nostd::mock::{Fault, MockI2c, Transaction};
use mpu6050_nostd::registers::*;
use mpu6050_nostd::{Error, FifoConfig, FifoSample, Mpu6050};

/// A ready driver with the FIFO configured and enabled, log drained.
fn fifo_ready(cfg: FifoConfig) -> Mpu6050<MockI2c> {
    let mut imu = ready();
    imu.fifo_configure(cfg).unwrap();
    imu.fifo_enable().unwrap();
    imu.bus_mut().take_log();
    imu
}

/// One 14-byte packet in register order: accel, temp, gyro.
fn packet_all(ax: i16, ay: i16, az: i16, t: i16, gx: i16, gy: i16, gz: i16) -> [u8; 14] {
    let mut p = [0u8; 14];
    p[0..6].copy_from_slice(&be6(ax, ay, az));
    p[6..8].copy_from_slice(&t.to_be_bytes());
    p[8..14].copy_from_slice(&be6(gx, gy, gz));
    p
}

// --- Packet arithmetic ------------------------------------------------------

#[test]
fn packet_len_accel_only_is_six() {
    assert_eq!(FifoConfig::ACCEL.packet_len(), 6);
}

#[test]
fn packet_len_accel_gyro_is_twelve() {
    assert_eq!(FifoConfig::ACCEL_GYRO.packet_len(), 12);
}

#[test]
fn packet_len_everything_is_fourteen() {
    assert_eq!(FifoConfig::ALL.packet_len(), 14);
    assert_eq!(FifoConfig::NONE.packet_len(), 0);
    assert!(FifoConfig::NONE.is_empty());
}

#[test]
fn gyro_axes_are_gated_individually() {
    let z_only = FifoConfig {
        gyro_z: true,
        ..FifoConfig::NONE
    };
    assert_eq!(z_only.packet_len(), 2);
    assert_eq!(z_only.fifo_en_bits(), FIFO_EN_ZG);
    let s = FifoSample::parse(z_only, &(-7i16).to_be_bytes());
    assert_eq!(s.gyro, [None, None, Some(-7)]);
    assert_eq!(s.accel, None);
    assert_eq!(s.temp, None);
}

// --- Configuration and control ----------------------------------------------

#[test]
fn fifo_configure_writes_fifo_en_bits_matching_the_config() {
    let mut imu = ready();
    imu.fifo_configure(FifoConfig::ALL).unwrap();
    assert_eq!(
        imu.bus().register(REG_FIFO_EN),
        FIFO_EN_ACCEL | FIFO_EN_TEMP | FIFO_EN_XG | FIFO_EN_YG | FIFO_EN_ZG
    );
    imu.fifo_configure(FifoConfig::ACCEL).unwrap();
    assert_eq!(imu.bus().register(REG_FIFO_EN), FIFO_EN_ACCEL);
    assert_eq!(imu.fifo_config(), FifoConfig::ACCEL);
}

#[test]
fn fifo_enable_sets_user_ctrl_fifo_en_and_preserves_other_bits() {
    let mut imu = ready();
    imu.bus_mut().set_register(REG_USER_CTRL, 0b0010_0000); // I2C_MST_EN owned elsewhere
    imu.fifo_enable().unwrap();
    assert_eq!(imu.bus().register(REG_USER_CTRL), 0b0110_0000);
    imu.fifo_disable().unwrap();
    assert_eq!(imu.bus().register(REG_USER_CTRL), 0b0010_0000);
}

#[test]
fn fifo_reset_disables_then_resets_then_re_enables_in_that_order() {
    let mut imu = fifo_ready(FifoConfig::ALL);
    imu.fifo_reset().unwrap();
    assert_eq!(
        writes_to(imu.bus().log(), REG_USER_CTRL),
        vec![
            vec![0x00],                 // FIFO_EN = 0
            vec![USER_CTRL_FIFO_RESET], // reset takes effect now
            vec![USER_CTRL_FIFO_EN],    // back on
        ]
    );
    // The mock ignores a reset written with FIFO_EN set; the FIFO being
    // empty afterwards proves the sequence was the working one.
    imu.bus_mut().push_fifo(&[0u8; 28]);
    imu.fifo_reset().unwrap();
    assert_eq!(imu.bus().fifo_len(), 0);
    assert_ne!(imu.bus().register(REG_USER_CTRL) & USER_CTRL_FIFO_EN, 0);
}

#[test]
fn fifo_reset_preserves_unrelated_user_ctrl_bits() {
    let mut imu = fifo_ready(FifoConfig::ALL);
    imu.bus_mut()
        .set_register(REG_USER_CTRL, USER_CTRL_FIFO_EN | 0b0010_0000);
    imu.fifo_reset().unwrap();
    for w in writes_to(imu.bus().log(), REG_USER_CTRL) {
        assert_ne!(w[0] & 0b0010_0000, 0, "I2C_MST_EN dropped in write {w:?}");
        assert_eq!(w[0] & USER_CTRL_I2C_IF_DIS, 0, "I2C_IF_DIS must stay 0");
    }
}

// --- Counting ---------------------------------------------------------------

#[test]
fn fifo_count_is_one_two_byte_burst_from_the_high_byte() {
    let mut imu = fifo_ready(FifoConfig::ALL);
    imu.bus_mut().push_fifo(&[0u8; 700]);
    assert_eq!(imu.fifo_count(), Ok(700));
    let log = imu.bus().log();
    assert_eq!(log.len(), 1);
    assert!(matches!(
        &log[0],
        Transaction::WriteRead { sent, received, .. }
            if sent.as_slice() == [REG_FIFO_COUNT_H] && received.len() == 2
    ));
}

// --- Draining ---------------------------------------------------------------

#[test]
fn packets_decode_in_register_order_accel_then_temp_then_gyro() {
    let mut imu = fifo_ready(FifoConfig::ALL);
    imu.bus_mut()
        .push_fifo(&packet_all(100, -200, 300, 340, -1000, 2000, -3000));
    let mut out = [FifoSample::default(); 1];
    assert_eq!(imu.fifo_read(&mut out), Ok(1));
    assert_eq!(out[0].accel, Some([100, -200, 300]));
    assert_eq!(out[0].temp, Some(340));
    assert_eq!(out[0].gyro, [Some(-1000), Some(2000), Some(-3000)]);
}

#[test]
fn a_config_without_temperature_shifts_gyro_forward() {
    // With temp disabled, gyro follows accel directly. A driver that assumed
    // a fixed 14-byte layout would read temperature bytes as gyro X.
    let mut imu = fifo_ready(FifoConfig::ACCEL_GYRO);
    let mut p = [0u8; 12];
    p[0..6].copy_from_slice(&be6(1, 2, 3));
    p[6..12].copy_from_slice(&be6(4, 5, 6));
    imu.bus_mut().push_fifo(&p);
    let mut out = [FifoSample::default(); 1];
    imu.fifo_read(&mut out).unwrap();
    assert_eq!(out[0].accel, Some([1, 2, 3]));
    assert_eq!(out[0].temp, None);
    assert_eq!(out[0].gyro, [Some(4), Some(5), Some(6)]);
}

#[test]
fn partial_packet_count_is_refused_and_nothing_is_read() {
    let mut imu = fifo_ready(FifoConfig::ALL);
    imu.bus_mut().push_fifo(&[0u8; 13]);
    let mut out = [FifoSample::default(); 2];
    assert_eq!(imu.fifo_read(&mut out), Err(Error::FifoUnaligned(13)));
    assert_eq!(
        imu.bus().fifo_len(),
        13,
        "unaligned FIFO must be left untouched"
    );
    assert_eq!(reads_of(imu.bus().log(), REG_FIFO_R_W), 0);
}

#[test]
fn undersized_output_reads_only_what_fits_and_leaves_the_rest() {
    let mut imu = fifo_ready(FifoConfig::ACCEL);
    for i in 0..5i16 {
        imu.bus_mut().push_fifo(&be6(i, i, i));
    }
    let mut out = [FifoSample::default(); 2];
    assert_eq!(imu.fifo_read(&mut out), Ok(2));
    assert_eq!(out[0].accel, Some([0, 0, 0]));
    assert_eq!(out[1].accel, Some([1, 1, 1]));
    assert_eq!(imu.bus().fifo_len(), 18, "three packets remain");
    // Next call continues from where it left off.
    assert_eq!(imu.fifo_read(&mut out), Ok(2));
    assert_eq!(out[0].accel, Some([2, 2, 2]));
}

#[test]
fn drain_bursts_are_bounded_by_max_burst_and_cover_everything() {
    let mut imu = fifo_ready(FifoConfig::ALL);
    for i in 0..5i16 {
        imu.bus_mut().push_fifo(&packet_all(i, 0, 0, 0, 0, 0, i));
    }
    let mut out = [FifoSample::default(); 8];
    assert_eq!(imu.fifo_read(&mut out), Ok(5));
    assert_eq!(imu.bus().fifo_len(), 0);
    let bursts: Vec<usize> = imu
        .bus()
        .log()
        .iter()
        .filter_map(|t| match t {
            Transaction::WriteRead { sent, received, .. } if sent[0] == REG_FIFO_R_W => {
                Some(received.len())
            }
            _ => None,
        })
        .collect();
    // 70 bytes in 14-byte packets, at most 32 bytes (two packets) per burst.
    assert_eq!(bursts, vec![28, 28, 14]);
    assert!(bursts.iter().all(|b| *b <= MAX_BURST && b % 14 == 0));
    assert_eq!(out[4].accel, Some([4, 0, 0]));
    assert_eq!(out[4].gyro[2], Some(4));
}

#[test]
fn empty_output_buffer_and_empty_config_are_handled_without_bus_traffic() {
    let mut imu = fifo_ready(FifoConfig::ALL);
    let mut none: [FifoSample; 0] = [];
    assert_eq!(imu.fifo_read(&mut none), Err(Error::BufferTooSmall));

    let mut unconfigured = ready();
    let mut out = [FifoSample::default(); 1];
    assert_eq!(unconfigured.fifo_read(&mut out), Ok(0));
    assert!(unconfigured.bus().log().is_empty());
}

#[test]
fn fifo_operations_require_init() {
    let mut imu = Mpu6050::new(MockI2c::new(), ADDR_AD0_LOW);
    let mut out = [FifoSample::default(); 1];
    assert_eq!(
        imu.fifo_configure(FifoConfig::ALL),
        Err(Error::NotInitialised)
    );
    assert_eq!(imu.fifo_enable(), Err(Error::NotInitialised));
    assert_eq!(imu.fifo_reset(), Err(Error::NotInitialised));
    assert_eq!(imu.fifo_count(), Err(Error::NotInitialised));
    assert_eq!(imu.fifo_read(&mut out), Err(Error::NotInitialised));
}

// --- Overflow ---------------------------------------------------------------

#[test]
fn overflow_is_reported_and_nothing_is_read() {
    let mut imu = fifo_ready(FifoConfig::ALL);
    imu.bus_mut().inject(Fault::FifoOverflow);
    let mut out = [FifoSample::default(); 4];
    assert_eq!(imu.fifo_read(&mut out), Err(Error::FifoOverflow));
    assert_eq!(imu.bus().fifo_len(), FIFO_CAPACITY_BYTES as usize);
    assert_eq!(reads_of(imu.bus().log(), REG_FIFO_R_W), 0);
    assert_eq!(out, [FifoSample::default(); 4]);
}

#[test]
fn overflow_flag_is_clear_on_read_but_the_driver_latches_it() {
    let mut imu = fifo_ready(FifoConfig::ALL);
    imu.bus_mut().set_register(REG_INT_STATUS, INT_FIFO_OFLOW);
    assert_eq!(imu.fifo_overflowed(), Ok(true));
    // The hardware bit is gone after one read...
    assert_eq!(imu.bus().register(REG_INT_STATUS) & INT_FIFO_OFLOW, 0);
    // ...but the driver still says so, until reset.
    assert_eq!(imu.fifo_overflowed(), Ok(true));
    imu.fifo_reset().unwrap();
    assert_eq!(imu.fifo_overflowed(), Ok(false));
}

#[test]
fn ignoring_an_overflow_error_does_not_get_you_data_on_the_next_call() {
    let mut imu = fifo_ready(FifoConfig::ALL);
    imu.bus_mut().inject(Fault::FifoOverflow);
    let mut out = [FifoSample::default(); 1];
    assert_eq!(imu.fifo_read(&mut out), Err(Error::FifoOverflow));
    imu.bus_mut().take_log();
    // Second call: hardware flag already consumed, but the driver remembers.
    assert_eq!(imu.fifo_read(&mut out), Err(Error::FifoOverflow));
    assert_eq!(reads_of(imu.bus().log(), REG_FIFO_R_W), 0);
}

#[test]
fn reset_recovers_from_overflow_and_reading_resumes() {
    let mut imu = fifo_ready(FifoConfig::ALL);
    imu.bus_mut().inject(Fault::FifoOverflow);
    let mut out = [FifoSample::default(); 1];
    assert_eq!(imu.fifo_read(&mut out), Err(Error::FifoOverflow));
    imu.fifo_reset().unwrap();
    assert_eq!(imu.bus().fifo_len(), 0);
    assert_eq!(imu.fifo_read(&mut out), Ok(0));
    imu.bus_mut().push_fifo(&packet_all(9, 9, 9, 9, 9, 9, 9));
    assert_eq!(imu.fifo_read(&mut out), Ok(1));
    assert_eq!(out[0].accel, Some([9, 9, 9]));
}

#[test]
fn post_overflow_stream_is_misaligned_and_refused_even_if_the_flag_was_missed() {
    // Model a driver that lost the flag some other way: a fresh driver
    // instance attached to a bus whose FIFO already overflowed and whose
    // INT_STATUS was consumed by someone else.
    let mut imu = fifo_ready(FifoConfig::ALL);
    imu.bus_mut().inject(Fault::FifoOverflow);
    imu.bus_mut().set_register(REG_INT_STATUS, 0); // flag gone
    let mut out = [FifoSample::default(); 1];
    // 1024 mod 14 = 2: the count itself betrays the misalignment.
    assert_eq!(imu.fifo_read(&mut out), Err(Error::FifoUnaligned(1024)));
    assert_eq!(imu.bus().fifo_len(), 1024);
}

#[test]
fn twelve_byte_packets_are_also_misaligned_after_overflow() {
    let mut imu = fifo_ready(FifoConfig::ACCEL_GYRO);
    imu.bus_mut().inject(Fault::FifoOverflow);
    imu.bus_mut().set_register(REG_INT_STATUS, 0);
    let mut out = [FifoSample::default(); 1];
    // 1024 mod 12 = 4.
    assert_eq!(imu.fifo_read(&mut out), Err(Error::FifoUnaligned(1024)));
}
