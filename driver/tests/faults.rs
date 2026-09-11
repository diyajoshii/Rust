//! Fault injection: what the driver does when the bus misbehaves.
//!
//! Most drivers test the happy path. These tests inject NACKs, corrupt
//! bytes, wrong identities, a floating bus and a FIFO overflow, and check
//! that the driver fails loudly, leaves consistent state, and never panics.
#![cfg(feature = "mock")]

mod common;
use common::*;

use embedded_hal::i2c::{Error as _, ErrorKind, NoAcknowledgeSource};
use mpu6050_nostd::mock::{Fault, MockError, MockI2c};
use mpu6050_nostd::registers::*;
use mpu6050_nostd::{AccelRange, Error, FifoConfig, FifoSample, Mpu6050};

const DATA_NACK: Error<MockError> = Error::Bus(MockError::NoAcknowledge(NoAcknowledgeSource::Data));

fn fresh() -> Mpu6050<MockI2c> {
    Mpu6050::new(MockI2c::new(), ADDR_AD0_LOW)
}

#[test]
fn nack_on_the_very_first_transaction_surfaces_as_a_bus_error() {
    let mut imu = fresh();
    imu.bus_mut().inject(Fault::NackAfter(0));
    assert_eq!(imu.init(), Err(DATA_NACK));
    assert_eq!(imu.bus().nack_count(), 1);
    // The device was never touched beyond the failed read.
    assert!(imu.bus().log().is_empty());
}

#[test]
fn nack_midway_through_init_leaves_an_error_not_a_half_configured_device() {
    // init's transaction order: 0 WHO_AM_I, 1 reset write, 2 poll, 3 poll,
    // 4 wake write. Fail the wake.
    let mut imu = fresh();
    imu.bus_mut().inject(Fault::NackAfter(4));
    assert_eq!(imu.init(), Err(DATA_NACK));
    // Device is still asleep — the wake never landed...
    assert_ne!(imu.bus().register(REG_PWR_MGMT_1) & PWR_MGMT_1_SLEEP, 0);
    // ...the clock and ranges were never written...
    assert!(writes_to(imu.bus().log(), REG_CONFIG).is_empty());
    // ...and the driver refuses to pretend otherwise.
    assert_eq!(imu.accel_raw(), Err(Error::NotInitialised));
}

#[test]
fn stuck_high_bus_reads_as_0xff_and_init_rejects_it() {
    let mut imu = fresh();
    imu.bus_mut().inject(Fault::StuckHigh);
    assert_eq!(imu.who_am_i(), Ok(0xFF));
    assert_eq!(imu.init(), Err(Error::WrongDevice(0xFF)));
}

#[test]
fn wrong_device_error_carries_the_byte_actually_observed() {
    // A corrupt identity read is indistinguishable from a wrong part; the
    // driver reports what it saw so the caller can tell 0x00 (held low)
    // from 0xFF (floating) from 0x70 (an MPU-9250 on the same address).
    let mut imu = fresh();
    imu.bus_mut().inject(Fault::CorruptByte {
        txn: 0,
        index: 0,
        value: 0x00,
    });
    assert_eq!(imu.init(), Err(Error::WrongDevice(0x00)));
}

#[test]
fn corrupt_byte_in_a_burst_is_returned_as_is_because_there_is_no_crc() {
    // The MPU-6050 has no checksum on I2C data. A flipped byte on the wire
    // is undetectable at this layer; the driver's contract is to not panic
    // and to leave detection to the caller (range checks, plausibility,
    // redundancy). This test documents that limit rather than hiding it.
    let mut imu = ready();
    imu.bus_mut()
        .set_registers(REG_ACCEL_XOUT_H, &be6(1000, 2000, 3000));
    let next = imu.bus().transactions_seen();
    imu.bus_mut().inject(Fault::CorruptByte {
        txn: next,
        index: 2, // high byte of Y
        value: 0x80,
    });
    let raw = imu.accel_raw().expect("no error is possible here");
    assert_eq!(raw[0], 1000);
    assert_eq!(raw[1], i16::from_be_bytes([0x80, (2000u16 & 0xFF) as u8]));
    assert_eq!(raw[2], 3000);
    // Subsequent reads are clean.
    assert_eq!(imu.accel_raw(), Ok([1000, 2000, 3000]));
}

#[test]
fn nack_on_a_setter_leaves_stored_and_device_state_consistent() {
    let mut imu = ready();
    // set_accel_range is read-modify-write: fail the write (second txn).
    let next = imu.bus().transactions_seen();
    imu.bus_mut().inject(Fault::NackAfter(next + 1));
    assert_eq!(imu.set_accel_range(AccelRange::G16), Err(DATA_NACK));
    assert_eq!(
        imu.accel_range(),
        AccelRange::G2,
        "struct must not run ahead of the device"
    );
    assert_eq!(imu.bus().register(REG_ACCEL_CONFIG), 0);
    // Conversion therefore still uses ±2 g, matching the part.
    imu.bus_mut()
        .set_registers(REG_ACCEL_XOUT_H, &be6(16_384, 0, 0));
    let [x, ..] = imu.accel_g().unwrap();
    assert_close(x, 1.0);
}

#[test]
fn nack_during_a_fifo_drain_is_a_bus_error_and_poisons_nothing() {
    let mut imu = ready();
    imu.fifo_configure(FifoConfig::ALL).unwrap();
    imu.fifo_enable().unwrap();
    imu.bus_mut().push_fifo(&[0u8; 28]);
    // fifo_read: INT_STATUS read, count read, then the FIFO_R_W burst.
    let next = imu.bus().transactions_seen();
    imu.bus_mut().inject(Fault::NackAfter(next + 2));
    let mut out = [FifoSample::default(); 2];
    assert_eq!(imu.fifo_read(&mut out), Err(DATA_NACK));
    assert_eq!(imu.fifo_config(), FifoConfig::ALL);
    assert_eq!(
        imu.fifo_overflowed(),
        Ok(false),
        "a NACK is not an overflow"
    );
    // Retry succeeds: the FIFO was not consumed by the failed burst.
    imu.bus_mut().clear_faults();
    assert_eq!(imu.fifo_read(&mut out), Ok(2));
}

#[test]
fn fifo_overflow_fault_is_visible_through_the_driver_api() {
    let mut imu = ready();
    imu.fifo_configure(FifoConfig::ALL).unwrap();
    imu.fifo_enable().unwrap();
    imu.bus_mut().inject(Fault::FifoOverflow);
    assert_eq!(imu.fifo_count(), Ok(FIFO_CAPACITY_BYTES));
    assert_eq!(imu.fifo_overflowed(), Ok(true));
}

#[test]
fn bus_errors_keep_their_embedded_hal_error_kind() {
    // Error::Bus(E) passes the HAL's error through untouched, so a caller
    // can still classify it the way embedded-hal intends.
    let mut imu = fresh();
    imu.bus_mut().inject(Fault::NackAfter(0));
    match imu.who_am_i() {
        Err(Error::Bus(e)) => assert_eq!(
            e.kind(),
            ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data)
        ),
        other => panic!("expected a bus error, got {other:?}"),
    }

    // Talking to an address nobody answers is an *address* NACK, and the
    // driver distinguishes it from a data NACK.
    let mut wrong = Mpu6050::new(MockI2c::new(), ADDR_AD0_HIGH);
    match wrong.who_am_i() {
        Err(Error::Bus(e)) => assert_eq!(
            e.kind(),
            ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)
        ),
        other => panic!("expected an address NACK, got {other:?}"),
    }
}

#[test]
fn several_faults_can_be_armed_and_cleared_together() {
    let mut imu = fresh();
    imu.bus_mut().inject(Fault::WrongDeviceId(0x69));
    imu.bus_mut().inject(Fault::NackAfter(1));
    assert_eq!(imu.init(), Err(Error::WrongDevice(0x69)));
    imu.bus_mut().clear_faults();
    imu.init().unwrap();
}
