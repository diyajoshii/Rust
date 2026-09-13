# demo-tm4c123g

On-target proof for [`mpu6050-nostd`](../README.md) on a TI EK-TM4C123GXL LaunchPad. Not published; not built for the host.

## What you need

- EK-TM4C123GXL LaunchPad (TM4C123GH6PM, Cortex-M4F)
- GY-521 MPU-6050 breakout
- 4 jumper wires; 2 × 4.7 kΩ resistors only if your breakout lacks pull-ups (most GY-521s have 2.2 kΩ on board)
- Optional but strongly recommended: any 8-channel USB logic analyser + [PulseView](https://sigrok.org/wiki/PulseView)

## Wiring

| GY-521 | LaunchPad | |
|---|---|---|
| VCC | 3.3 V | (J1-1) |
| GND | GND | |
| SCL | **PB2** | I2C0SCL |
| SDA | **PB3** | I2C0SDA |
| AD0 | GND or floating | address `0x68` |

Leave the LaunchPad's `PWR SELECT` on **DEBUG** so the ICDI USB port powers it and exposes the UART.

## Build

```bash
rustup target add thumbv7em-none-eabihf
cargo build -p demo-tm4c123g --release --target thumbv7em-none-eabihf
```

Output: `target/thumbv7em-none-eabihf/release/demo-tm4c123g` (ELF).

## Flash

Any one of these. The ICDI debugger on the board is supported by all three.

**lm4flash** (simplest; part of `lm4tools`):

```bash
rustup component add llvm-tools-preview
"$(find "$(rustc --print sysroot)" -name llvm-objcopy -type f | head -1)" -O binary \
  target/thumbv7em-none-eabihf/release/demo-tm4c123g demo.bin
lm4flash demo.bin
```

**OpenOCD:**

```bash
openocd -f board/ek-tm4c123gxl.cfg \
  -c "program target/thumbv7em-none-eabihf/release/demo-tm4c123g verify reset exit"
```

**probe-rs** (needs ICDI support in your probe-rs version, or an external CMSIS-DAP/J-Link on the debug header):

```bash
probe-rs run --chip TM4C123GH6PM target/thumbv7em-none-eabihf/release/demo-tm4c123g
```

## Watch

The ICDI enumerates a virtual COM port. 115200 8N1.

```bash
# Linux/macOS
screen /dev/ttyACM0 115200
# Windows: PuTTY / Tera Term on the "Stellaris Virtual Serial Port" COMx
```

Expected output, healthy board at rest, Z up:

```
mpu6050-nostd demo on TM4C123G
WHO_AM_I = 0x68
init ok: awake, gyro-X PLL clock, +/-2 g, +/-250 dps
accel raw [   -12    244  16388]  = [    0     14   1000] mg   gyro raw [   -35     12     -8]
...
sample rate 100 Hz (DLPF on, 1 kHz base)
fifo: 5 packets  first: accel [   -10    240  16390]  temp 31.12 C  gyro z     -7
...
--- provoking overflow: not reading for 2 s ---
overflow flag = true, FIFO_COUNT = 1024, count mod 14 = 2
  predicted: flag = true, count = 1024, mod = 2 (window starts mid-packet)
fifo_read refused: FIFO overflow (as designed)
fifo_reset ok — resuming
```

## The five checks (brief §8)

1. **`WHO_AM_I = 0x68`.** If you see `0xFF` and `init failed: wrong device id` — pull-ups are missing or a wire is off. If `0x00` — SDA is shorted low. The demo prints which.
2. **Z ≈ +1000 mg at rest, X and Y ≈ 0.** Within ±50 mg is normal for an uncalibrated part.
3. **Rotate 90°** — the ~1000 mg moves to whichever axis now points down, with the sign matching the silkscreen arrow.
4. **Logic-analyser capture.** Probe SCL (PB2) and SDA (PB3), 1 MHz+ sample rate, I²C decoder in PulseView, address `0x68`. Trigger on the first activity after reset and you will see the `init()` sequence exactly as `tests/init.rs` asserts it: `[W 0x75][R 1]`, `[W 0x6B 0x80]`, `[W 0x6B][R 1]` repeated until bit 7 clears, `[W 0x6B 0x00]`, `[W 0x6B 0x01]`, `[W 0x1A 0x00 0x00 0x00]`. Screenshot → `docs/scope-capture.png`, and reference it from the root README's Hardware section.
5. **The overflow prediction.** Every 20 drains the demo provokes an overflow and prints `overflow flag`, `FIFO_COUNT`, and `count mod 14`. The derivation predicts `true, 1024, 2`. Record what the part actually printed — either way — in `docs/datasheet-notes.md` under *Open items*, and in the root README's overflow section.

## If the bridge misbehaves

`src/bridge.rs` adapts `tm4c123x-hal`'s `embedded-hal` 0.2 I2C to the 1.0 trait the driver uses. If reads return garbage but `WHO_AM_I` is right, suspect the coalescing of `Write`+`Read` into `write_read` — check the analyser for a repeated START (no STOP) between the register-address write and the data read. If there is a STOP, the HAL's `write_read` is not doing a repeated start and the fix is at the HAL layer, not here.
