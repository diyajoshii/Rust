//! `init()` — the bring-up sequence, asserted against bus traffic.
#![cfg(feature = "mock")]

mod common;
use common::*;

use mpu6050_nostd::mock::{Fault, MockI2c, Transaction};
use mpu6050_nostd::registers::*;
use mpu6050_nostd::{Error, Mpu6050, RESET_POLL_LIMIT};

fn fresh() -> Mpu6050<MockI2c> {
    Mpu6050::new(MockI2c::new(), ADDR_AD0_LOW)
}

#[test]
fn init_produces_exactly_the_documented_transaction_sequence() {
    let mut imu = fresh();
    imu.init().unwrap();
    let a = ADDR_AD0_LOW;
    let expected = vec![
        Transaction::WriteRead {
            addr: a,
            sent: vec![REG_WHO_AM_I],
            received: vec![WHO_AM_I_VALUE],
        },
        Transaction::Write {
            addr: a,
            bytes: vec![REG_PWR_MGMT_1, PWR_MGMT_1_DEVICE_RESET],
        },
        // First poll: reset still in progress (mock default: one poll).
        Transaction::WriteRead {
            addr: a,
            sent: vec![REG_PWR_MGMT_1],
            received: vec![PWR_MGMT_1_DEVICE_RESET | PWR_MGMT_1_SLEEP],
        },
        // Second poll: cleared, device asleep as after power-on.
        Transaction::WriteRead {
            addr: a,
            sent: vec![REG_PWR_MGMT_1],
            received: vec![PWR_MGMT_1_SLEEP],
        },
        Transaction::Write {
            addr: a,
            bytes: vec![REG_PWR_MGMT_1, 0x00],
        },
        Transaction::Write {
            addr: a,
            bytes: vec![REG_PWR_MGMT_1, PWR_MGMT_1_CLKSEL_PLL_X],
        },
        // CONFIG, GYRO_CONFIG, ACCEL_CONFIG in one burst at power-on values.
        Transaction::Write {
            addr: a,
            bytes: vec![REG_CONFIG, 0x00, 0x00, 0x00],
        },
    ];
    assert_eq!(imu.bus().log(), expected.as_slice());
}

#[test]
fn reset_is_written_before_anything_else_is_configured() {
    let mut imu = fresh();
    imu.init().unwrap();
    let writes: Vec<&Transaction> = imu
        .bus()
        .log()
        .iter()
        .filter(|t| matches!(t, Transaction::Write { .. }))
        .collect();
    assert_eq!(
        writes[0],
        &Transaction::Write {
            addr: ADDR_AD0_LOW,
            bytes: vec![REG_PWR_MGMT_1, PWR_MGMT_1_DEVICE_RESET]
        }
    );
}

#[test]
fn sleep_is_cleared_only_after_reset_has_completed() {
    let mut imu = fresh();
    imu.bus_mut().set_reset_poll_count(3);
    imu.init().unwrap();
    let log = imu.bus().log();
    let last_poll = log
        .iter()
        .rposition(
            |t| matches!(t, Transaction::WriteRead { sent, .. } if sent[0] == REG_PWR_MGMT_1),
        )
        .unwrap();
    let wake = log
        .iter()
        .position(|t| matches!(t, Transaction::Write { bytes, .. } if bytes[..] == [REG_PWR_MGMT_1, 0x00]))
        .unwrap();
    assert!(
        wake > last_poll,
        "wake at {wake} must follow last poll at {last_poll}"
    );
    // And the last poll really did observe the bit clear.
    assert!(matches!(
        &log[last_poll],
        Transaction::WriteRead { received, .. } if received[0] & PWR_MGMT_1_DEVICE_RESET == 0
    ));
}

#[test]
fn clock_is_selected_after_wake_not_before() {
    let mut imu = fresh();
    imu.init().unwrap();
    let pm1 = writes_to(imu.bus().log(), REG_PWR_MGMT_1);
    assert_eq!(
        pm1,
        vec![
            vec![PWR_MGMT_1_DEVICE_RESET],
            vec![0x00],
            vec![PWR_MGMT_1_CLKSEL_PLL_X]
        ]
    );
}

#[test]
fn init_leaves_the_device_awake_on_the_gyro_pll() {
    let mut imu = fresh();
    imu.init().unwrap();
    assert_eq!(imu.bus().register(REG_PWR_MGMT_1), PWR_MGMT_1_CLKSEL_PLL_X);
}

#[test]
fn wrong_who_am_i_is_rejected_with_the_observed_value() {
    let mut imu = fresh();
    imu.bus_mut().inject(Fault::WrongDeviceId(0x70));
    assert_eq!(imu.init(), Err(Error::WrongDevice(0x70)));
    // Nothing was written to a device we did not recognise.
    assert!(
        imu.bus()
            .log()
            .iter()
            .all(|t| matches!(t, Transaction::WriteRead { .. }))
    );
}

#[test]
fn init_is_idempotent() {
    let mut imu = fresh();
    imu.init().unwrap();
    let first: Vec<u8> = (0..=255u8).map(|r| imu.bus().register(r)).collect();
    imu.init().unwrap();
    let second: Vec<u8> = (0..=255u8).map(|r| imu.bus().register(r)).collect();
    assert_eq!(first, second);
}

#[test]
fn reset_polling_continues_until_the_bit_clears() {
    let mut imu = fresh();
    imu.bus_mut().set_reset_poll_count(5);
    imu.init().unwrap();
    // Five polls see the bit set, the sixth sees it clear.
    assert_eq!(reads_of(imu.bus().log(), REG_PWR_MGMT_1), 6);
}

#[test]
fn reset_that_never_clears_times_out_within_the_poll_budget() {
    let mut imu = fresh();
    imu.bus_mut().set_reset_poll_count(u32::MAX);
    assert_eq!(imu.init(), Err(Error::ResetTimeout));
    assert_eq!(reads_of(imu.bus().log(), REG_PWR_MGMT_1), RESET_POLL_LIMIT);
    // Device left asleep and unconfigured — no wake, no clock, no ranges.
    assert!(writes_to(imu.bus().log(), REG_PWR_MGMT_1).len() == 1);
    assert!(writes_to(imu.bus().log(), REG_CONFIG).is_empty());
}

#[test]
fn measurement_before_init_is_refused() {
    let mut imu = fresh();
    assert_eq!(imu.accel_raw(), Err(Error::NotInitialised));
    assert_eq!(imu.gyro_raw(), Err(Error::NotInitialised));
    assert_eq!(imu.temperature_c(), Err(Error::NotInitialised));
    assert!(
        imu.bus().log().is_empty(),
        "refused calls must not touch the bus"
    );
}

#[test]
fn who_am_i_works_before_init() {
    let mut imu = fresh();
    assert_eq!(imu.who_am_i(), Ok(WHO_AM_I_VALUE));
}
