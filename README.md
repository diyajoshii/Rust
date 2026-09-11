# mpu6050-nostd

**An MPU-6050 driver you can test without an MPU-6050.**

Register-level `no_std` Rust for the InvenSense MPU-6050 accelerometer/gyroscope, generic over [`embedded-hal`](https://crates.io/crates/embedded-hal) 1.0, with a fault-injecting mock I2C bus so the whole driver — including bus failures — runs under `cargo test` on a laptop with no hardware attached.

[![CI](https://github.com/diyajoshii/Rust/actions/workflows/ci.yml/badge.svg)](https://github.com/diyajoshii/Rust/actions/workflows/ci.yml)

> **Status:** under construction. Register map and datasheet notes are in; driver, mock bus and FIFO are being built in the open. Nothing is published to crates.io yet.

## Why a mock bus

`no_std` code can't use Rust's test runner — the runner needs `std`. The usual answer is "test on the target", which means every test needs a board plugged in and none of it runs in CI.

This crate takes the other route: a feature-gated `std` build. With `--features mock` the crate gains a fake I2C bus that implements the same `embedded_hal::i2c::I2c` trait the real HAL does, so the driver cannot tell the difference. The mock holds a 256-byte register file seeded with the chip's power-on defaults, records every transaction that crosses it, and can be told to fail on demand — NACK the third write, corrupt a byte in a burst read, hold every line high the way a bus with missing pull-ups does, or overflow the FIFO.

Tests then assert on **observable bus traffic**, not internal state: that `init()` cleared `SLEEP` before selecting the clock, that a six-byte read was one transaction and not six, that `FIFO_COUNT` was read high-byte-first because the datasheet says the low byte is a shadow.

Without the feature, the crate is pure `no_std` — no allocator, no OS — and builds for `thumbv7em-none-eabihf`. CI builds both.

## Where the register semantics come from

Every register constant traces to a quoted line of the InvenSense register map in [`docs/datasheet-notes.md`](docs/datasheet-notes.md), cited by register number. The gotchas that most amateur drivers get wrong — the FIFO reset that is silently ignored unless the FIFO is disabled first, the count register whose low byte goes stale, the interrupt status that clears itself when you look at it — are documented there with the sentence that says so.

## Licence

Dual-licensed under MIT or Apache-2.0, at your option.
