# Datasheet notes

Every register constant and every behavioural assumption in this crate, with the line from the datasheet it came from.

**Source:** InvenSense *MPU-6000 and MPU-6050 Register Map and Descriptions*, document `RM-MPU-6000A-00`, **revision 4.0**, release date 03/09/2012.

**Citation convention:** by register number and name, never by section number. Section numbers move between revisions — `FIFO_R_W` is §4.39 in rev 3.2 and §4.33 in rev 4.0, same register, same text. Register numbers are stable.

Quotes are verbatim; `[...]` marks elision. Whitespace inside quotes is normalised (the PDF text layer inserts spurious spaces).

---

## Self-test

### Registers 13–16 (`SELF_TEST_X/Y/Z/A`) — `0x0D`–`0x10`

Layout, verbatim from the register table:

| Reg | Bits 7:5 | Bits 4:0 |
|---|---|---|
| 13 `SELF_TEST_X` | `XA_TEST[4:2]` | `XG_TEST[4:0]` |
| 14 `SELF_TEST_Y` | `YA_TEST[4:2]` | `YG_TEST[4:0]` |
| 15 `SELF_TEST_Z` | `ZA_TEST[4:2]` | `ZG_TEST[4:0]` |
| 16 `SELF_TEST_A` | reserved (7:6) · `XA_TEST[1:0]` (5:4) · `YA_TEST[1:0]` (3:2) · `ZA_TEST[1:0]` (1:0) |

> "the factory trim values for the accel should be in decimal format, and they are determined by concatenating the upper accelerometer self test bits (bits 4-2) with the lower accelerometer self test bits (bits 1-0)."

So `XA_TEST = (SELF_TEST_X[7:5] << 2) | SELF_TEST_A[5:4]`, and similarly for Y (`A[3:2]`) and Z (`A[1:0]`). Gyro values are the low five bits directly.

**Procedure and pass criterion:**

> "Self-Test Response = Gyroscope Output with Self-Test Enabled − Gyroscope Output with Self-Test Disabled"

> "Change from Factory Trim of the Self-Test Response (%) = (STR − FT) / FT"

> "This change from factory trim of the self-test response must be within the limits provided in the MPU-6000/MPU-6050 Product Specification document for the part to pass self-test."

The register map **does not state the limit**; it defers to the Product Specification (PS-MPU-6000A). The driver therefore takes the tolerance as a parameter. `DEFAULT_SELF_TEST_TOLERANCE_PERCENT = 14.0` is the figure that document is widely reported to give — it is a default, not a quotation, and is labelled as such in the code. Confirm against the Product Specification before relying on it for a production pass/fail.

**Gyroscope factory trim** — "When performing self test for the gyroscope, the full-scale range should be set to ±250dps."

> `FT[Xg] = 25 ∗ 131 ∗ 1.046^(XG_TEST−1)  if XG_TEST ≠ 0;  FT[Xg] = 0 if XG_TEST = 0`
> `FT[Yg] = −25 ∗ 131 ∗ 1.046^(YG_TEST−1) if YG_TEST ≠ 0;  FT[Yg] = 0 if YG_TEST = 0`
> `FT[Zg] = 25 ∗ 131 ∗ 1.046^(ZG_TEST−1)  if ZG_TEST ≠ 0;  FT[Zg] = 0 if ZG_TEST = 0`

Note the sign on Y. 131 is the ±250 dps sensitivity (Register 27).

**Accelerometer factory trim** — "When performing accelerometer self test, the full-scale range should be set to ±8g."

> `FT[Xa] = 4096 ∗ 0.34 ∗ (0.92/0.34)^((XA_TEST−1)/(2^5−2))  if XA_TEST ≠ 0;  FT[Xa] = 0 if XA_TEST = 0`

Same for Y and Z. 4096 is the ±8 g sensitivity (Register 28); `2^5 − 2 = 30`. At `n = 31` the exponent is 1 and FT = 4096 × 0.92.

**Implementation note.** `core` has no `powf`, and the exponent is a 5-bit integer, so the driver carries two 31-entry `f32` tables — `1.046^(n−1)` and `(0.92/0.34)^((n−1)/30)` for `n = 1..=31` — computed offline and checked against `f32::powf` in a host test. `FT = 0` (test value 0) has no defined pass criterion; the driver reports that axis as `None` and the overall result as failed.

**Settling time is not specified** in the register map. The driver waits `SELF_TEST_SETTLE_MS` (50 ms) after toggling the self-test bits, via a caller-supplied `embedded_hal::delay::DelayNs`. This is the only place the driver needs a delay, which is why it is an argument to `self_test()` rather than a field of the driver.

## Identity

### Register 117 (`WHO_AM_I`) — `0x75`

> "This register is used to verify the identity of the device. The contents of WHO_AM_I are the upper 6 bits of the MPU-60X0's 7-bit I2C address. The least significant bit of the MPU-60X0's I2C address is determined by the value of the AD0 pin. The value of the AD0 pin is not reflected in this register. The default value of the register is 0x68. Bits 0 and 7 are reserved. (Hard coded to 0)"

Consequences encoded in `registers.rs`:

- `WHO_AM_I_VALUE = 0x68`.
- Both `0x68` (AD0 low) and `0x69` (AD0 high) devices report the same identity value, because bit 0 is masked off. An identity check must not simply compare against the bus address.

---

## Power and clocking

### Register 107 (`PWR_MGMT_1`) — `0x6B`

> "DEVICE_RESET — When set to 1, this bit resets all internal registers to their default values. **The bit automatically clears to 0 once the reset is done.**"

> "SLEEP — When set to 1, this bit puts the MPU-60X0 into sleep mode."

This is the justification for polling rather than sleeping in `init()`. The reset bit is self-clearing and readable, so completion is *observable* — there is no need to guess a delay constant, and the reset path stays testable on a host with no clock.

`CLKSEL` table:

| Value | Clock source |
|---|---|
| 0 | Internal 8 MHz oscillator |
| **1** | **PLL with X axis gyroscope reference** |
| 2 | PLL with Y axis gyroscope reference |
| 3 | PLL with Z axis gyroscope reference |
| 4 | PLL with external 32.768 kHz reference |
| 5 | PLL with external 19.2 MHz reference |
| 6 | Reserved |
| 7 | Stops the clock and keeps the timing generator in reset |

`PWR_MGMT_1_CLKSEL_PLL_X = 0x01` selects row 1.

Constants: `DEVICE_RESET = 0x80` (bit 7), `SLEEP = 0x40` (bit 6), `CLKSEL_PLL_X = 0x01`.

---

## Sample rate and filtering

### Register 25 (`SMPLRT_DIV`) — `0x19`

> "Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7), and 1kHz when the DLPF is enabled (see Register 26)."

The base rate is **not** constant. Computing a divider without knowing the current `DLPF_CFG` is wrong by a factor of 8. This is why the driver stores the configured DLPF setting.

### Register 26 (`CONFIG`) — `0x1A`

Layout: `[7:6]` reserved, `[5:3]` `EXT_SYNC_SET`, `[2:0]` `DLPF_CFG`.

| `DLPF_CFG` | Accel BW (Hz) | Gyro BW (Hz) | Gyro output rate (kHz) |
|---|---|---|---|
| 0 | 260 | 256 | **8** |
| 1 | 184 | 188 | 1 |
| 2 | 94 | 98 | 1 |
| 3 | 44 | 42 | 1 |
| 4 | 21 | 20 | 1 |
| 5 | 10 | 10 | 1 |
| 6 | 5 | 5 | 1 |
| 7 | RESERVED | RESERVED | 8 |

`DlpfConfig` in `registers.rs` exposes values 0–6 only. **Value 7 is deliberately not exposed** — it is marked RESERVED, and offering it in a safe API would invite a configuration the datasheet does not define. Its 8 kHz base rate is noted here only so the omission is a recorded decision rather than an oversight.

`gyro_output_rate_hz()` therefore returns 8000 for `Hz260` (`DLPF_CFG = 0`) and 1000 for all exposed others. Correct for the exposed set.

**Note the MPU-6050 has no `FIFO_MODE` bit.** `CONFIG` bit 6 is reserved here. The FIFO-full-behaviour control bit at `CONFIG[6]` exists only on the MPU-6500/6555 generation. See the overflow section below.

---

## Ranges and sensitivity

### Register 27 (`GYRO_CONFIG`) — `0x1B`

`FS_SEL` occupies bits `[4:3]`.

> "FS_SEL | Full Scale Range | LSB Sensitivity — 0 | ± 250 °/s | 131 LSB/°/s [...]"

| `FS_SEL` | Range | Sensitivity |
|---|---|---|
| 0 | ± 250 °/s | 131 LSB/°/s |
| 1 | ± 500 °/s | 65.5 LSB/°/s |
| 2 | ± 1000 °/s | 32.8 LSB/°/s |
| 3 | ± 2000 °/s | 16.4 LSB/°/s |

### Register 28 (`ACCEL_CONFIG`) — `0x1C`

`AFS_SEL` occupies bits `[4:3]`.

> "AFS_SEL | Full Scale Range | LSB Sensitivity — 0 | ±2g | 16384 LSB/g — 1 | ±4g | 8192 LSB/g — 2 | ±8g | 4096 LSB/g — 3 | ±16g | 2048 LSB/g"

Both enums store the raw `FS_SEL`/`AFS_SEL` value as the discriminant and expose `config_bits()` = `(self as u8) << 3`, so the shift into position happens in exactly one place.

---

## Measurement registers

### Registers 59–64 (`ACCEL_*OUT_H/L`) — `0x3B`–`0x40`

> "ACCEL_XOUT — 16-bit 2's complement value."

Six consecutive bytes, high byte first, X/Y/Z. Read as **one** burst starting at `0x3B`: the axes are latched together, and separate single-byte reads can straddle a sample update and tear across axes.

### Registers 65–66 (`TEMP_OUT_H/L`) — `0x41`–`0x42`

> "The temperature in degrees C for a given register value may be computed as: Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53"

> "TEMP_OUT — 16-bit signed value."

### Registers 67–72 (`GYRO_*OUT_H/L`) — `0x43`–`0x48`

Same shape as the accelerometer block: six bytes, high byte first, X/Y/Z, 2's complement.

---

## Interrupts

### Register 56 (`INT_ENABLE`) — `0x38`

> "FIFO_OFLOW_EN — When set to 1, this bit enables a FIFO buffer overflow to generate an interrupt."

`FIFO_OFLOW_EN` = bit 4 = `0x10`.

### Register 58 (`INT_STATUS`) — `0x3A`

> "This register shows the interrupt status of each interrupt generation source. **Each bit will clear after the register is read.**"

> "FIFO_OFLOW_INT — This bit automatically sets to 1 when a FIFO buffer overflow interrupt has been generated. The bit clears to 0 after the register has been read."

**Clear-on-read.** The driver must never read `INT_STATUS` speculatively: a diagnostic read consumes the overflow flag, and the next genuine check reports a healthy device. `fifo_overflowed()` is documented as consuming the flag.

Bit layout: `FIFO_OFLOW_INT` = bit 4 = `0x10`, `DATA_RDY_INT` = bit 0 = `0x01`. Bits 2 and 1 reserved.

---

## FIFO

### Register 35 (`FIFO_EN`) — `0x23`

Per-sensor enables, each quoted from its own parameter description:

> "TEMP_FIFO_EN — When set to 1, this bit enables TEMP_OUT_H and TEMP_OUT_L (Registers 65 and 66) to be written into the FIFO buffer."

> "XG_FIFO_EN — [...] GYRO_XOUT_H and GYRO_XOUT_L (Registers 67 and 68) [...]"

> "ACCEL_FIFO_EN — When set to 1, this bit enables ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L (Registers 59 to 64) to be written into the FIFO buffer."

| Bit | Constant | Bytes contributed per sample |
|---|---|---|
| 7 | `TEMP_FIFO_EN` = `0x80` | 2 |
| 6 | `XG_FIFO_EN` = `0x40` | 2 |
| 5 | `YG_FIFO_EN` = `0x20` | 2 |
| 4 | `ZG_FIFO_EN` = `0x10` | 2 |
| 3 | `ACCEL_FIFO_EN` = `0x08` | **6** (all three axes as one unit) |
| 2–0 | `SLV2`/`SLV1`/`SLV0` | out of scope |

Accel is all-or-nothing — one bit gates all six bytes. Gyro axes are individually gated. This is what makes `packet_len` config-derived rather than constant.

### Register 106 (`USER_CTRL`) — `0x6A`

Layout: bit 6 `FIFO_EN`, bit 5 `I2C_MST_EN`, bit 4 `I2C_IF_DIS`, bit 2 `FIFO_RESET`, bit 1 `I2C_MST_RESET`, bit 0 `SIG_COND_RESET`. Bits 7 and 3 reserved.

> "FIFO_EN — When set to 1, this bit enables FIFO operations. When this bit is cleared to 0, the FIFO buffer is disabled. The FIFO buffer cannot be written to or read from while disabled."

> "**FIFO_RESET — This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0.** This bit automatically clears to 0 after the reset has been triggered."

> "When the reset bits (FIFO_RESET, I2C_MST_RESET, and SIG_COND_RESET) are set to 1, these reset bits will trigger a reset and then clear to 0."

**This is a trap, and it is the correction that came out of writing this file.** `FIFO_RESET` is conditional: it only takes effect while `FIFO_EN` is **0**. Writing `FIFO_RESET` to a running FIFO does nothing at all — silently. Since resetting the FIFO is the *only* recovery from an overflow, a driver that gets this wrong can never clear an overflow: it calls reset, the reset is ignored, the overflow flag latches again on the next sample, and the device appears permanently broken.

Correct sequence, which `fifo_reset()` must implement as three writes:

1. Clear `FIFO_EN` in `USER_CTRL`
2. Set `FIFO_RESET`
3. Set `FIFO_EN` again

`MPU-6050:` also note "Always write 0 to `I2C_IF_DIS`" — that bit selects SPI on the MPU-6000 and must not be disturbed. Read-modify-write `USER_CTRL`; do not blind-write it.

Also worth recording: "The FIFO buffer's state does not change unless the MPU-60X0 is power [cycled]" when `FIFO_EN` is cleared — disabling is not the same as resetting.

### Registers 114–115 (`FIFO_COUNT_H` / `FIFO_COUNT_L`) — `0x72` / `0x73`

> "These registers shadow the FIFO Count value. Both registers are loaded with the current sample count when FIFO_COUNT_H (Register 72) is read. **Note: Reading only FIFO_COUNT_L will not update the registers to the current sample count. FIFO_COUNT_H must be accessed first** to update the contents of both these registers. FIFO_COUNT should always be read in high-low order in order to guarantee that the most current FIFO Count value is read."

> "FIFO_COUNT — 16-bit unsigned value. Indicates the number of bytes stored in the FIFO buffer."

The low byte is a shadow latched by reading the high byte. This is a *hardware-mandated access pattern*, not merely a tear-avoidance preference: reading `FIFO_COUNT_L` alone returns stale data by design. Read as one 2-byte burst starting at `0x72`.

Note also that the count is in **bytes**, not samples — dividing by `packet_len` is the caller's job.

### Register 116 (`FIFO_R_W`) — `0x74`

> "Data is written to the FIFO in order of register number (from lowest to highest)."

This fixes packet field order as **accel (59–64) → temperature (65–66) → gyro (67–72)**, regardless of the order the enable bits were set. A driver that assumes enable order produces data that is wrong only when temperature is enabled — a bug that survives casual testing.

> "If the FIFO buffer has overflowed, the status bit FIFO_OFLOW_INT is automatically set to 1. This bit is located in INT_STATUS (Register 58). **When the FIFO buffer has overflowed, the oldest data will be lost and new data will be written to the FIFO.**"

> "If the FIFO buffer is empty, reading this register will return the last byte that was previously read from the FIFO until new data is available. The user should check FIFO_COUNT to ensure that the FIFO buffer is not read when empty."

Two things follow.

**Overflow drops the oldest data, not the newest.** Verified identical in revision 3.2 (11/14/2011) and revision 4.0 (03/09/2012). The opposite behaviour — retaining old contents and discarding new samples — is the MPU-6500/6555, which gained a `FIFO_MODE` bit at `CONFIG[6]` to select it. The MPU-6050 has no such bit.

**Overflow destroys packet alignment.** *(Derived, not quoted — flagged as a prediction until confirmed on hardware.)* The FIFO drops the oldest **bytes**; it has no packet awareness, being a flat byte stream written in register order. The buffer is 1024 bytes, and 1024 is not a multiple of any realistic packet size:

| Enabled sensors | `packet_len` | `1024 mod packet_len` |
|---|---|---|
| Accel only | 6 | 4 |
| Accel + gyro | 12 | 4 |
| Accel + temp + gyro | 14 | 2 |

So the retained window begins mid-packet, and every subsequent read is byte-misaligned — the high byte of X pairing with the low byte of the previous sample's Z. The resulting values have plausible magnitudes and a plausible sign distribution. Nothing about them looks wrong. Recovery requires `FIFO_RESET` per Register 106 above, including the `FIFO_EN = 0` precondition.

Empty-FIFO behaviour is equally worth guarding: reading an empty FIFO silently repeats the last byte rather than erroring, so `FIFO_COUNT` must be checked first.

**`FIFO_CAPACITY_BYTES = 1024.`**

---

## Open items

- [ ] Revision **4.2** was not obtainable while writing this file — the InvenSense URL now redirects to a marketing page. Rev 4.0 is pinned because it is the copy actually read. Rev 3.2 was cross-checked for the FIFO overflow text and is word-for-word identical, so that behaviour is stable across revisions. If a 4.2 copy surfaces, re-confirm and update the header above. `src/registers.rs` currently cites 4.2 and must be reconciled to whichever revision is actually verified.
- [ ] The packet-misalignment consequence above is an inference from quoted text plus arithmetic, not a quoted claim. Confirm on hardware and record the measured outcome here either way.
