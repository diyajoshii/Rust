# Changelog

All notable changes to `mpu6050-nostd`. Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow [SemVer](https://semver.org/).

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

[0.1.0]: https://github.com/diyajoshii/Rust/releases/tag/v0.1.0
