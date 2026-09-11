//! Shared helpers for the integration tests. Every test runs the real
//! driver against the mock bus and asserts on the traffic it produced.
#![allow(dead_code)]

use mpu6050_nostd::Mpu6050;
use mpu6050_nostd::mock::{MockI2c, Transaction};
use mpu6050_nostd::registers::ADDR_AD0_LOW;

/// A driver that has completed `init()`, with the bring-up traffic drained
/// from the log so tests see only what they cause.
pub fn ready() -> Mpu6050<MockI2c> {
    let mut imu = Mpu6050::new(MockI2c::new(), ADDR_AD0_LOW);
    imu.init().expect("init against a healthy mock");
    imu.bus_mut().take_log();
    imu
}

/// The data bytes of every `Write` transaction that targeted `reg`, in order.
pub fn writes_to(log: &[Transaction], reg: u8) -> Vec<Vec<u8>> {
    log.iter()
        .filter_map(|t| match t {
            Transaction::Write { bytes, .. } if bytes.first() == Some(&reg) => {
                Some(bytes[1..].to_vec())
            }
            _ => None,
        })
        .collect()
}

/// Number of `WriteRead` transactions that started at `reg`.
pub fn reads_of(log: &[Transaction], reg: u8) -> usize {
    log.iter()
        .filter(|t| matches!(t, Transaction::WriteRead { sent, .. } if sent.as_slice() == [reg]))
        .count()
}

/// Big-endian bytes for three signed 16-bit values, as the sensor stores them.
pub fn be6(x: i16, y: i16, z: i16) -> [u8; 6] {
    let [xh, xl] = x.to_be_bytes();
    let [yh, yl] = y.to_be_bytes();
    let [zh, zl] = z.to_be_bytes();
    [xh, xl, yh, yl, zh, zl]
}

pub fn assert_close(actual: f32, expected: f32) {
    let tol = 1e-4 * expected.abs().max(1.0);
    assert!(
        (actual - expected).abs() <= tol,
        "expected {expected}, got {actual}"
    );
}
