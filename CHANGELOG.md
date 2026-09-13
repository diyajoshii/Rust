# Changelog

All notable changes to `mpu6050-nostd`. Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow [SemVer](https://semver.org/).

## [0.2.0] — 2026-09-13

### Added

- **Hardware self-test.** `Mpu6050::self_test(&mut delay, tolerance_percent)` runs the six-axis actuation sequence and returns a `SelfTestReport` with per-axis response, factory trim, deviation and pass flags. Factory trim follows Registers 13–16 exactly: gyro `±25·131·1.046^(n−1)` at ±250 dps (Y negative), accel `4096·0.34·(0.92/0.34)^((n−1)/30)` at ±8 g, with the accel test values reassembled from the split fields of Registers 13–16. Exponentials are 31-entry tables verified against `f32::powf`. Ranges are restored and actuation switched off even if a bus error interrupts the run.
- `selftest::{gyro_factory_trim, accel_factory_trim, SelfTestValues, GYRO_TRIM_TABLE, ACCEL_TRIM_TABLE, DEFAULT_SELF_TEST_TOLERANCE_PERCENT, SELF_TEST_SETTLE_MS, SELF_TEST_SAMPLES}`.
- `registers::{REG_SELF_TEST_X/Y/Z/A, GYRO_CONFIG_ST_ALL, ACCEL_CONFIG_ST_ALL}`.
- Mock: `MockI2c::set_self_test_response()` models the actuators; `MockDelay` records requested delays without sleeping.

11 new tests (87 total).

## [0.1.0] — 2026-09-13

First release. Host-tested; not yet validated on silicon (see README → Hardware).

### Driver

- `Mpu6050<I2C>` generic over `embedded_hal::i2c::I2c` 1.0. Pure `no_std`, no `alloc`.
- `init()`: identity check, `DEVICE_RESET` polled until it self-clears (no delay provider needed), wake, gyro-X PLL clock selected in a separate write, then `CONFIG`/`GYRO_CONFIG`/`ACCEL_CONFIG` in one burst.
- Range, DLPF and FIFO configuration stored in the struct; conversion, sample-rate divider and packet length are derived from it, never assumed.
- `set_sample_rate_hz()` accounts for the DLPF-dependent base rate (8 kHz off, 1 kHz on) and returns the rate actually achieved.
- Accelerometer and gyroscope reads are single 6-byte bursts.
- `Error<E>` carries the HAL's error through unchanged; `WrongDevice(u8)` carries the byte observed.

### FIFO

- `FifoConfig` mirrors `FIFO_EN` gating; `FifoSample::parse` decodes in register order (accel, temperature, gyro).
- `fifo_count()` is one 2-byte burst from `FIFO_COUNT_H` (the low byte is a latched shadow).
- `fifo_reset()` is three writes — clear `FIFO_EN`, set `FIFO_RESET`, set `FIFO_EN` — because `FIFO_RESET` is ignored while the FIFO is enabled.
- `fifo_read()` refuses after an overflow (latched in the driver until reset, since `INT_STATUS` is clear-on-read) and refuses counts that are not a whole number of packets. Bursts bounded by a 32-byte stack buffer.

### Mock (`--features mock`)

- `MockI2c` implements `I2c` via `transaction`, classifies operation slices into a `Transaction` log, and models the device behaviours above: pointer auto-increment, reset self-clear after *N* polls, clear-on-read status, shadowed count, conditional FIFO reset.
- Five faults: `NackAfter`, `CorruptByte`, `WrongDeviceId`, `StuckHigh`, `FifoOverflow` (fills to 1024 bytes from a packet stream so the retained window is mid-packet).

### Verification

- 76 tests; 98.2% line coverage including `mock.rs`, published by CI to the `coverage` branch.
- CI builds for `thumbv7em-none-eabihf` with `--no-default-features` to prove `no_std`.
- Demo binary for the TM4C123G LaunchPad builds at 10,608 bytes of flash; hand-written `embedded-hal` 0.2 → 1.0 bridge.

[0.2.0]: https://github.com/diyajoshii/Rust/releases/tag/v0.2.0
[0.1.0]: https://github.com/diyajoshii/Rust/releases/tag/v0.1.0
