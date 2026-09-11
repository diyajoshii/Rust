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
