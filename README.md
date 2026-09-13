# mpu6050-nostd

**An MPU-6050 driver you can test without an MPU-6050.**

Register-level `no_std` Rust for the InvenSense MPU-6050 accelerometer/gyroscope, generic over [`embedded-hal`](https://crates.io/crates/embedded-hal) 1.0, with a fault-injecting mock I2C bus so the whole driver — including the FIFO, including bus failures — runs under `cargo test` on a laptop with no hardware attached.

[![crates.io](https://img.shields.io/crates/v/mpu6050-nostd.svg)](https://crates.io/crates/mpu6050-nostd)
[![docs.rs](https://img.shields.io/docsrs/mpu6050-nostd)](https://docs.rs/mpu6050-nostd)
[![CI](https://github.com/diyajoshii/Rust/actions/workflows/ci.yml/badge.svg)](https://github.com/diyajoshii/Rust/actions/workflows/ci.yml)
[![line coverage](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/diyajoshii/Rust/coverage/badge.json)](https://github.com/diyajoshii/Rust/blob/coverage/coverage.json)

```toml
[dependencies]
mpu6050-nostd = "0.2"
```

**87 tests.** The coverage badge is written by CI on every push to `main` — `cargo llvm-cov` output, including `mock.rs`, committed as [`badge.json`](https://github.com/diyajoshii/Rust/blob/coverage/badge.json) on the `coverage` branch. It cannot go stale and nobody types it in.

> **Status:** 0.2.0 is published. Driver, mock bus and FIFO are complete and under test. The on-target demo binary builds for the TM4C123G at **10,608 bytes of flash** (release, size-optimised — measured by CI). Validation on real silicon is the next milestone and will ship as 0.3.0 with the logic-analyser capture; the hardware sections below are marked accordingly.

---

## Why a mock bus

`no_std` code can't use Rust's test runner — the runner needs `std`. The usual answer is "test on the target", which means every test needs a board plugged in, none of it runs in CI, and bus failures get tested by unplugging wires and hoping.

This crate takes the other route, one I first used to get a `no_std` Ethernet stack under test in production: **feature-gate a `std` build and stand in for the world at the boundary.** With `--features mock` the crate gains [`mock::MockI2c`](driver/src/mock.rs), which implements the same `embedded_hal::i2c::I2c` trait a real HAL does. The driver cannot tell the difference.

The mock is not a register array with a trait on top. It models the parts of the device that bite:

| Behaviour | Why it matters |
|---|---|
| Register pointer auto-increments; stays put at `FIFO_R_W` | Burst reads work like the part; FIFO pops byte by byte |
| `DEVICE_RESET` self-clears after *N* polls (configurable) | The reset path — including the timeout — is testable with no clock |
| `INT_STATUS` clears on read | A speculative read destroys the overflow flag, as on the part |
| `FIFO_COUNT_L` is a stale shadow until `FIFO_COUNT_H` is read | A driver that reads the low byte alone gets caught |
| `FIFO_RESET` is silently ignored while `FIFO_EN` is set | A driver that resets without disabling first gets caught |
| Overflow keeps the newest 1024 bytes of a packet stream | The retained window starts mid-packet, exactly as the arithmetic predicts |

Every transaction is recorded. Tests assert on **bus traffic**, not internal state — that `init()` cleared `SLEEP` before selecting the clock, that a six-byte read was one transaction and not six, that the count was read high-byte-first.

And it fails on demand. Five `Fault`s: NACK the *n*th transaction, corrupt one byte of one read, report the wrong identity, hold every line high the way a bus with missing pull-ups does, or overflow the FIFO.

Without the feature the crate is pure `no_std` — no allocator, no OS — and builds for `thumbv7em-none-eabihf`. CI builds both.

---

## Usage

```rust
use mpu6050_nostd::{AccelRange, DlpfConfig, FifoConfig, FifoSample, Mpu6050, registers::ADDR_AD0_LOW};

let mut imu = Mpu6050::new(i2c, ADDR_AD0_LOW);   // any embedded_hal::i2c::I2c
imu.init()?;                                     // identity, reset (polled), wake, gyro PLL clock
imu.set_accel_range(AccelRange::G8)?;            // stored; conversion below uses it
imu.set_dlpf(DlpfConfig::Hz44)?;                 // base rate is now 1 kHz, not 8 kHz
let achieved = imu.set_sample_rate_hz(100)?;     // Ok(100) — divider computed from the real base

let [x, y, z] = imu.accel_g()?;                  // one 6-byte burst, scaled to ±8 g
let t = imu.temperature_c()?;

imu.fifo_configure(FifoConfig::ACCEL_GYRO)?;     // 12-byte packets, derived not assumed
imu.fifo_enable()?;
let mut buf = [FifoSample::default(); 16];
match imu.fifo_read(&mut buf) {
    Ok(n) => { /* buf[..n] are whole packets in register order */ }
    Err(mpu6050_nostd::Error::FifoOverflow) => imu.fifo_reset()?,   // the only recovery
    Err(e) => return Err(e),
}
```

The same code against the mock — this is what the test suite does:

```rust
use mpu6050_nostd::mock::{Fault, MockI2c, Transaction};

let mut imu = Mpu6050::new(MockI2c::new(), ADDR_AD0_LOW);
imu.bus_mut().inject(Fault::StuckHigh);          // missing pull-ups
assert_eq!(imu.who_am_i(), Ok(0xFF));
assert_eq!(imu.init(), Err(Error::WrongDevice(0xFF)));

imu.bus_mut().clear_faults();
imu.init()?;
imu.accel_raw()?;
let log = imu.bus().log();                       // assert on what actually crossed the bus
assert!(matches!(log.last(), Some(Transaction::WriteRead { received, .. }) if received.len() == 6));
```

Errors are `Error<E>` where `E` is the HAL's own error type, passed through untouched in `Error::Bus(E)` — a NACK on address is still distinguishable from a NACK on data. `WrongDevice(u8)` carries the byte actually observed: `0xFF` is a floating bus, `0x00` a held-low line, `0x70` an MPU-9250 on the same address.

---

## Architecture

```
mpu6050-nostd/
├── driver/            the published crate (this README's subject)
│   ├── src/lib.rs        Mpu6050<I2C>: init, config, measurement
│   ├── src/fifo.rs       FifoConfig, FifoSample, drain state machine
│   ├── src/registers.rs  every address and bit, cited by register number
│   ├── src/error.rs      Error<E>
│   ├── src/mock.rs       feature-gated: MockI2c, Transaction, Fault
│   └── tests/            init, config, measurement, fifo, faults
├── demo-tm4c123g/     on-target binary — separate package, never built for the host
└── docs/datasheet-notes.md   the quote behind every constant
```

**Two builds, one crate.** `default = []` is `no_std` with no `alloc`; `mock` pulls in `std` for exactly one module. `mock.rs` is the only file permitted to use `std`, and it is `pub` — the mock is part of the crate's public API, not scaffolding hidden in `tests/`, so anyone building on this driver can test their own code against it.

**Why the demo is a separate package.** A Cortex-M binary needs a linker script, `cortex-m-rt` and a panic handler; it cannot build for the host, and `cargo clippy --all-targets` would try. The workspace keeps the driver host-testable and target-buildable while the demo stays target-only.

**Why the demo has its own HAL bridge.** `tm4c123x-hal` predates `embedded-hal` 1.0 and implements the 0.2 traits — `Write`, `Read`, `WriteRead`, nothing more. The driver targets 1.0 and stays that way; the adaptation lives entirely in [`demo-tm4c123g/src/bridge.rs`](demo-tm4c123g/src/bridge.rs). The off-the-shelf `embedded-hal-compat` shim was tried first and rejected by the compiler: its blanket impl also demands `WriteIter`, `Transactional` and friends, which the HAL does not provide. The hand-written bridge needs exactly what the HAL has, coalesces a `Write`+`Read` operation pair into one `write_read` so the repeated start survives, and is forty lines.

**The driver holds state on purpose.** Accelerometer range, gyroscope range, DLPF setting and FIFO configuration all live in the struct. Unit conversion, the sample-rate divider and the FIFO packet length are *derived* from them. A driver that hard-codes ±2 g, or a 1 kHz base rate, or a 14-byte packet, is wrong the moment any of those is changed.

**Reset is polled, not slept.** `DEVICE_RESET` "automatically clears to 0 once the reset is done" (Register 107). The driver reads it until it does, bounded by `RESET_POLL_LIMIT`. No `DelayNs` generic, no magic constant, and the timeout path has a test.

---

## Seven things the datasheet says that most drivers miss

Each is a quoted line in [`docs/datasheet-notes.md`](docs/datasheet-notes.md), enforced in code, and covered by a test that fails if the enforcement is removed.

1. **Read all six accelerometer bytes in one burst.** The axes are latched together; six single-byte reads can straddle a sample update and tear across axes.
2. **The sample-rate base depends on the filter.** `rate = base / (1 + SMPLRT_DIV)` where `base` is 8 kHz with the DLPF off and 1 kHz with it on. Miss it and you are off by 8×.
3. **Store the configured range.** Conversion that assumes ±2 g is silently wrong after `set_accel_range`.
4. **FIFO packet order is register order** — accelerometer, then *temperature*, then gyroscope — not the order you set the enable bits. Wrong only when temperature is enabled, which is how it survives casual testing.
5. **`FIFO_COUNT_L` is a shadow.** "Reading only FIFO_COUNT_L will not update the registers to the current sample count." Read both bytes in one burst starting at the high byte.
6. **`INT_STATUS` clears on read.** A diagnostic peek consumes the overflow flag. The driver latches it internally so a caller who ignores the error is still refused next time.
7. **`FIFO_RESET` only works while `FIFO_EN` is 0.** Written to a running FIFO it is ignored — silently. Since reset is the only recovery from overflow, a driver that gets this wrong can never recover. `fifo_reset()` is three writes: disable, reset, enable.

---

## What happens when the FIFO overflows

Register 116, verbatim:

> "When the FIFO buffer has overflowed, the oldest data will be lost and new data will be written to the FIFO."

Oldest, not newest. (The MPU-6500 generation added a `FIFO_MODE` bit to choose; the MPU-6050 has no such bit.) That sentence, plus arithmetic, predicts something the datasheet does not spell out:

The FIFO drops the oldest **bytes**. It has no notion of packets — it is a flat byte stream written in register order. The buffer is 1024 bytes, and 1024 is not a multiple of any realistic packet size:

| Enabled | Packet | `1024 mod packet` |
|---|---|---|
| Accel only | 6 | 4 |
| Accel + gyro | 12 | 4 |
| Accel + temp + gyro | 14 | 2 |

So once overflow begins, the retained window starts mid-packet, and **every subsequent read is byte-misaligned** — the high byte of X pairs with the low byte of the previous sample's Z. The values have plausible magnitudes and a plausible sign distribution. Nothing about them looks wrong. A driver that ignores the overflow flag returns silently corrupt data indefinitely.

This driver refuses. `fifo_read()` returns `Error::FifoOverflow` and reads nothing; it keeps refusing until `fifo_reset()`; and independently, it refuses any count that is not a whole number of packets — so even a caller who somehow lost the flag is caught when `1024 mod packet ≠ 0`. The mock reproduces the misalignment from a real packet stream, and the tests prove both refusals for 12- and 14-byte packets.

**Honest limit:** with an 8-byte packet (accelerometer plus one gyro axis) `1024 mod 8 = 0`, the count looks aligned, and only the flag can save you. That is why the flag is latched.

**Status of the prediction:** the sentence is quoted; the misalignment is derived. It will be confirmed or refuted on hardware in the on-target milestone, and this section will say which. The test suite is unaffected either way — the mock encodes a defined contract.

---

## Self-test

The part can actuate each sensor's proof mass electrically and report the response, and each part carries a factory-trim value for that response in Registers 13–16. Comparing the two checks the mechanical and electrical path without moving the board.

```rust
let report = imu.self_test(&mut delay, DEFAULT_SELF_TEST_TOLERANCE_PERCENT)?;
if !report.passed() {
    // report.gyro_passed(), report.accel_passed(): per-axis
    // report.gyro_deviation_percent[i]: Some((STR − FT) / FT × 100), or None if FT is undefined
}
```

What the driver does, all of it visible on the bus: sets ±250 dps and ±8 g (the ranges the trim formulas assume), averages eight reads, enables all six `*_ST` bits, waits, averages again, then restores your ranges with actuation off — **even if a bus error interrupted the measurement** — and finally reads the four trim registers in one burst.

Three datasheet details it gets right that are easy to miss:

- **The gyro Y trim is negative.** `FT[Yg] = −25 · 131 · 1.046^(n−1)`; X and Z are positive.
- **Accel test values are split across registers.** `XA_TEST` is bits 7:5 of Register 13 concatenated with bits 5:4 of Register 16 — five bits from two places.
- **The pass limit is not in the register map.** It defers to the Product Specification, so the tolerance is an argument you supply; `DEFAULT_SELF_TEST_TOLERANCE_PERCENT` is labelled as a default, not a quotation.

`core` has no `powf`, and the exponent is a 5-bit integer, so the two exponentials are 31-entry tables computed offline and verified against `f32::powf` by a host test. `self_test()` is the only method that takes a delay — nothing else in the driver needs a timer.

---

## Roadmap

### 0.3.0 — on-target validation

The driver is complete and host-tested. What no mock can prove is what the *part* does, so 0.3.0 is the release that runs the demo on silicon and records the answers. The runbook is [`demo-tm4c123g/README.md`](demo-tm4c123g/README.md).

- [ ] `WHO_AM_I` reads `0x68` over a real bus
- [ ] Z ≈ +1 g at rest, X and Y ≈ 0; axes swap correctly on rotation
- [ ] The `init()` sequence on a logic analyser matches what `tests/init.rs` asserts — capture committed as `docs/scope-capture.png`
- [ ] The HAL's `write_read` issues a repeated START, not a STOP, between register address and data
- [ ] The overflow prediction below — `flag = true, FIFO_COUNT = 1024, count mod 14 = 2` — confirmed or refuted, and this README updated with whichever it was

### After that

- **Motion-detection interrupt** — `MOT_THR`, `MOT_DUR`, `INT_ENABLE.MOT_EN`. Documented, small.
- **`embedded-hal-async`** — the same driver over the async I2C trait, with an async mock.
- **Fuzzing `FifoSample::parse`** with `cargo fuzz` — `no_std` parsing code under a fuzzer, no hardware required.
- **Low-power cycle mode** — `LP_WAKE_CTRL`.

**Not planned: the DMP.** The Digital Motion Processor runs an undocumented firmware blob. Supporting it would mean porting InvenSense's binary the way every other DMP driver does, which is the opposite of what this crate is for.

### Target hardware

*Wiring is from the TM4C123GH6PM and GY-521 documentation; not yet exercised on silicon.*

**Target:** TI EK-TM4C123GXL LaunchPad (Cortex-M4F, `thumbv7em-none-eabihf`), GY-521 MPU-6050 breakout.

| GY-521 | LaunchPad | Note |
|---|---|---|
| VCC | 3.3 V | The GY-521 has an on-board regulator; 5 V also works, but the logic is 3.3 V either way |
| GND | GND | |
| SCL | PB2 (I2C0SCL) | |
| SDA | PB3 (I2C0SDA) | |
| AD0 | GND (or leave floating — pulled down on most boards) | Address `0x68`; tie to 3.3 V for `0x69` |

**Pull-ups:** the GY-521 usually carries 2.2 kΩ pull-ups on SCL and SDA. If yours does not, add 4.7 kΩ to 3.3 V on each line. A bus with no pull-ups reads `0xFF` on every byte — which is exactly what `Fault::StuckHigh` reproduces, and why `init()` rejects `WHO_AM_I = 0xFF` as `WrongDevice(0xFF)` instead of hanging.

**Flash footprint of the demo:** 10,608 bytes — `.vector_table` 1,024 + `.text` 7,888 + `.rodata` 1,696 — for `cargo build -p demo-tm4c123g --release --target thumbv7em-none-eabihf` with `opt-level = "s"` and LTO. That includes `init`, register reads, the full FIFO path with overflow recovery, and UART output with integer formatting only. `cargo size -- -A` also prints a `Total` line several times larger: that is the ELF with debug info, not what goes on the chip.

**Logic-analyser capture:** arrives with 0.3.0.

---

## Running the tests

```bash
# Host: the full suite against the mock bus
cargo test -p mpu6050-nostd --features mock

# Prove it is really no_std: build for a Cortex-M4F with no std to fall back on
rustup target add thumbv7em-none-eabihf
cargo build -p mpu6050-nostd --no-default-features --target thumbv7em-none-eabihf

# Lints, both modes
cargo clippy -p mpu6050-nostd --all-targets --features mock -- -D warnings
cargo clippy -p mpu6050-nostd --no-default-features --target thumbv7em-none-eabihf -- -D warnings

# Line coverage (Linux/macOS, or Windows with the MSVC toolchain — the GNU
# toolchain ships without the profiler runtime)
cargo install cargo-llvm-cov
cargo llvm-cov -p mpu6050-nostd --features mock --summary-only
```

`mock.rs` is included in the coverage measurement. It is shipped, public code.

---

## Licence

Dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.

Register semantics are taken from the InvenSense *MPU-6000 and MPU-6050 Register Map and Descriptions*, RM-MPU-6000A-00 rev 4.0. The document itself is not redistributed here; the lines this crate depends on are quoted, with register numbers, in [`docs/datasheet-notes.md`](docs/datasheet-notes.md).
