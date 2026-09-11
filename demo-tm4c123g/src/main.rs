//! On-target demo: `mpu6050-nostd` on a TI EK-TM4C123GXL LaunchPad.
//!
//! Wiring: GY-521 SCL → PB2, SDA → PB3, VCC → 3.3 V, GND → GND. Output is
//! on UART0 (the LaunchPad's ICDI virtual COM port), 115200 8N1.
//!
//! What it does, in order:
//!
//! 1. `init()` — prints `WHO_AM_I` or the error and stops.
//! 2. Ten register reads: raw accel/gyro counts and integer milli-g.
//! 3. FIFO at 100 Hz, 14-byte packets, drained every ~50 ms.
//! 4. Every 20 drains it deliberately stops reading for two seconds so the
//!    FIFO overflows (100 Hz × 14 B = 1400 B/s into 1024 B), then reports
//!    exactly what the driver saw: the overflow flag, the count, and
//!    `count mod 14` — the on-silicon check of the misalignment prediction.
//!    Then it resets and carries on.
//!
//! Everything printed is an integer. Float formatting alone would cost
//! several KiB of flash, and the point of this binary is to be measured.
#![no_std]
#![no_main]

use core::fmt::Write;

use cortex_m_rt::entry;
use panic_halt as _;
use tm4c123x_hal::gpio::{AF1, AF3, Floating};
use tm4c123x_hal::i2c::I2c;
use tm4c123x_hal::serial::{NewlineMode, Serial};
use tm4c123x_hal::sysctl::{CrystalFrequency, Oscillator, PllOutputFrequency, SystemClock};
use tm4c123x_hal::{self as hal, prelude::*};

use mpu6050_nostd::registers::ADDR_AD0_LOW;
use mpu6050_nostd::{DlpfConfig, Error, FifoConfig, FifoSample, Mpu6050};

mod bridge;
use bridge::Bridge;

/// 80 MHz core: this many cycles is ~1 ms.
const CYCLES_PER_MS: u32 = 80_000;

fn delay_ms(ms: u32) {
    cortex_m::asm::delay(ms * CYCLES_PER_MS);
}

/// Name an error without needing `Debug` on the bus error type (or `core::fmt`
/// float support). Keeps the binary small and the output greppable.
fn describe<E>(e: &Error<E>) -> &'static str {
    match e {
        Error::Bus(_) => "bus error (NACK / arbitration / timeout)",
        Error::WrongDevice(_) => "wrong device id",
        Error::NotInitialised => "not initialised",
        Error::ResetTimeout => "reset did not complete",
        Error::SampleRateUnreachable(_) => "sample rate unreachable",
        Error::FifoOverflow => "FIFO overflow",
        Error::FifoUnaligned(_) => "FIFO count not packet-aligned",
        Error::BufferTooSmall => "buffer too small",
    }
}

#[entry]
fn main() -> ! {
    let p = hal::Peripherals::take().unwrap();
    let mut sc = p.SYSCTL.constrain();
    sc.clock_setup.oscillator = Oscillator::Main(
        CrystalFrequency::_16mhz,
        SystemClock::UsePll(PllOutputFrequency::_80_00mhz),
    );
    let clocks = sc.clock_setup.freeze();

    // UART0 on PA0 (RX) / PA1 (TX) → the debugger's virtual COM port.
    let mut porta = p.GPIO_PORTA.split(&sc.power_control);
    let mut uart = Serial::uart0(
        p.UART0,
        porta.pa1.into_af_push_pull::<AF1>(&mut porta.control),
        porta.pa0.into_af_push_pull::<AF1>(&mut porta.control),
        (),
        (),
        115_200_u32.bps(),
        NewlineMode::SwapLFtoCRLF,
        &clocks,
        &sc.power_control,
    );

    // I2C0 on PB2 (SCL) / PB3 (SDA), 100 kHz standard mode.
    let mut portb = p.GPIO_PORTB.split(&sc.power_control);
    let i2c = I2c::i2c0(
        p.I2C0,
        (
            portb.pb2.into_af_push_pull::<AF3>(&mut portb.control),
            portb
                .pb3
                .into_af_open_drain::<AF3, Floating>(&mut portb.control),
        ),
        100_000_u32.hz(),
        &clocks,
        &sc.power_control,
    );

    // 0.2 HAL → the 1.0 trait the driver is generic over.
    let mut imu = Mpu6050::new(Bridge(i2c), ADDR_AD0_LOW);

    writeln!(uart, "\nmpu6050-nostd demo on TM4C123G").ok();

    match imu.who_am_i() {
        Ok(id) => writeln!(uart, "WHO_AM_I = 0x{id:02X}").ok(),
        Err(e) => writeln!(uart, "WHO_AM_I read failed: {}", describe(&e)).ok(),
    };

    if let Err(e) = imu.init() {
        writeln!(uart, "init failed: {}", describe(&e)).ok();
        if let Error::WrongDevice(seen) = e {
            writeln!(
                uart,
                "  saw 0x{seen:02X} (0xFF = floating bus / no pull-ups, 0x00 = line held low)"
            )
            .ok();
        }
        loop {
            cortex_m::asm::wfi();
        }
    }
    writeln!(uart, "init ok: awake, gyro-X PLL clock, +/-2 g, +/-250 dps").ok();

    // --- Plain register reads ---------------------------------------------
    for _ in 0..10 {
        delay_ms(100);
        let (a, g) = match (imu.accel_raw(), imu.gyro_raw()) {
            (Ok(a), Ok(g)) => (a, g),
            (Err(e), _) | (_, Err(e)) => {
                writeln!(uart, "read failed: {}", describe(&e)).ok();
                continue;
            }
        };
        // ±2 g: 16384 LSB/g → milli-g = raw * 1000 / 16384, integer maths only.
        let mg = a.map(|v| i32::from(v) * 1000 / 16_384);
        writeln!(
            uart,
            "accel raw [{:6} {:6} {:6}]  = [{:5} {:5} {:5}] mg   gyro raw [{:6} {:6} {:6}]",
            a[0], a[1], a[2], mg[0], mg[1], mg[2], g[0], g[1], g[2]
        )
        .ok();
    }

    // --- FIFO -------------------------------------------------------------
    let setup = imu
        .set_dlpf(DlpfConfig::Hz44)
        .and_then(|_| imu.set_sample_rate_hz(100))
        .and_then(|rate| {
            writeln!(uart, "sample rate {rate} Hz (DLPF on, 1 kHz base)").ok();
            imu.fifo_configure(FifoConfig::ALL)
        })
        .and_then(|_| imu.fifo_reset()); // disable → reset → enable: starts aligned
    if let Err(e) = setup {
        writeln!(uart, "FIFO setup failed: {}", describe(&e)).ok();
    }

    let mut buf = [FifoSample::default(); 8];
    let mut drains: u32 = 0;
    loop {
        drains += 1;

        if drains % 20 == 0 {
            // Deliberately overflow: 1400 B/s into 1024 B for two seconds.
            writeln!(uart, "--- provoking overflow: not reading for 2 s ---").ok();
            delay_ms(2000);
            // Read the flag and count *before* fifo_read, so both are visible.
            let flag = imu.fifo_overflowed();
            let count = imu.fifo_count();
            match (flag, count) {
                (Ok(f), Ok(c)) => {
                    writeln!(
                        uart,
                        "overflow flag = {f}, FIFO_COUNT = {c}, count mod 14 = {}",
                        c % 14
                    )
                    .ok();
                    writeln!(
                        uart,
                        "  predicted: flag = true, count = 1024, mod = 2 (window starts mid-packet)"
                    )
                    .ok();
                }
                (Err(e), _) | (_, Err(e)) => {
                    writeln!(uart, "overflow probe failed: {}", describe(&e)).ok();
                }
            }
            // The driver has latched the overflow; a read must refuse.
            match imu.fifo_read(&mut buf) {
                Err(Error::FifoOverflow) => {
                    writeln!(uart, "fifo_read refused: FIFO overflow (as designed)").ok()
                }
                Err(e) => writeln!(uart, "fifo_read refused: {}", describe(&e)).ok(),
                Ok(n) => writeln!(
                    uart,
                    "UNEXPECTED: fifo_read returned {n} packets after overflow"
                )
                .ok(),
            };
            match imu.fifo_reset() {
                Ok(()) => writeln!(uart, "fifo_reset ok — resuming").ok(),
                Err(e) => writeln!(uart, "fifo_reset failed: {}", describe(&e)).ok(),
            };
            continue;
        }

        delay_ms(50);
        match imu.fifo_read(&mut buf) {
            Ok(0) => {}
            Ok(n) => {
                let s = buf[0];
                let a = s.accel.unwrap_or([0; 3]);
                let t = s.temp.unwrap_or(0);
                let gz = s.gyro[2].unwrap_or(0);
                // Temperature: (raw / 340 + 36.53) °C, printed as centi-degrees in integer maths.
                let centi_c = i32::from(t) * 100 / 340 + 3653;
                writeln!(
                    uart,
                    "fifo: {n} packets  first: accel [{:6} {:6} {:6}]  temp {}.{:02} C  gyro z {:6}",
                    a[0], a[1], a[2], centi_c / 100, (centi_c % 100).unsigned_abs(), gz
                )
                .ok();
            }
            Err(Error::FifoOverflow) => {
                writeln!(uart, "unplanned overflow — resetting").ok();
                imu.fifo_reset().ok();
            }
            Err(e) => {
                writeln!(uart, "fifo_read failed: {}", describe(&e)).ok();
            }
        }
    }
}
