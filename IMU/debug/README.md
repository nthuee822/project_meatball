# ICM-20948 Debug Scripts

These scripts diagnose I2C communication problems with the ICM-20948 on Raspberry Pi 5.
Run them **in order** — each step tells you whether to continue or what to fix before moving on.

---

## Recovery: Hard Power Reset

> **If writes fail at any step, do this first before continuing.**

The chip can enter a locked state after a failed driver initialization, where writes
silently fail or throw `OSError 121 (Remote I/O)`. Soft reset via `DEVICE_RESET` does
**not** recover it — only a hard power reset does.

```bash
# 1. Disconnect power (USB-C or GPIO 5 V rail) from the Raspberry Pi
# 2. Wait at least 5 seconds
# 3. Reconnect power
# 4. Restart from Step 1
```

To quickly verify the chip is writable after a power cycle:

```bash
python -c "
from icm20948 import open_i2c_bus, sleep_ms
bus = open_i2c_bus(1)
bus.write_byte_data(0x68, 0x06, 0x01)
sleep_ms(100)
pwr = bus.read_byte_data(0x68, 0x06)
print(f'PWR_MGMT_1: 0x{pwr:02X}')
print('OK' if pwr == 0x01 else 'STILL LOCKED')
bus.close()
"
```

---

## Step 1 — Can we reach the chip at all?

```bash
python debug_addresses.py
```

Scans `0x68` and `0x69`. Reads `WHO_AM_I`, then does a write/read round-trip on
`PWR_MGMT_1` to confirm writes actually stick.

| Output | Meaning | What to do |
|--------|---------|------------|
| `WHO_AM_I: 0xEA ✓` and write succeeds | Chip found and writable | Proceed to Step 2 |
| `WHO_AM_I read failed` on both addresses | Chip not found at all | Check wiring, 3.3 V power, and I2C enabled (`raspi-config`) |
| `WHO_AM_I: 0xEA` but write throws `OSError 121` | Chip alive but locked | **Hard power reset**, then retry Step 1 |
| `WHO_AM_I: 0x??` (not `0xEA`) | Wrong chip or address collision | Check `AD0` pin level — low = `0x68`, high = `0x69` |

---

## Step 2 — Can we wake the chip from SLEEP mode?

```bash
python debug_pwr.py
```

Reads the initial `PWR_MGMT_1` state, writes `0x01` (clear SLEEP, use PLL), and checks
whether the SLEEP bit (bit 6) clears. Also tries a soft reset sequence.

| Output | Meaning | What to do |
|--------|---------|------------|
| `SLEEP bit = 0` after write | Chip awake and writable | Proceed to Step 3 |
| `SLEEP bit = 1` after all writes | Writes silently ignored | **Hard power reset**, then retry from Step 1 |
| `OSError` on any write | Chip locked | **Hard power reset**, then retry from Step 1 |
| `After reset: 0x41` (SLEEP bit still set) | Soft reset not clearing SLEEP | **Hard power reset** — soft reset is ineffective in this state |

---

## Step 3 — Is the sensor actually capturing data?

```bash
python debug_minimal.py
```

Does a manual reset + wake sequence at raw I2C level (no driver), then reads
`ACCEL_XOUT_H` 10 times and checks whether values are changing.

| Output | Meaning | What to do |
|--------|---------|------------|
| Values differ across reads | Sensor is alive and updating | Proceed to Step 4 |
| `WARNING: All values are identical` | Sensor awake but data frozen | Write `PWR_MGMT_2 = 0x00` to enable all sensors; recheck |
| All reads return `0` | Sensor still in sleep/disabled state | Go back to Step 2; chip did not wake properly |

---

## Step 4 — Does bank switching work?

```bash
python debug_banks.py
```

Selects bank 2 via `REG_BANK_SEL`, reads it back to confirm the switch, then writes
`0x11` to `GYRO_CONFIG_1` and verifies the read-back.

| Output | Meaning | What to do |
|--------|---------|------------|
| `REG_BANK_SEL reads back as: 0x20` and `GYRO_CONFIG_1: 0x11` | Bank switching and writes work | Proceed to Step 5 |
| `REG_BANK_SEL reads back as: 0x??` (not `0x20`) | Bank select write failed | I2C bus instability — check pull-up resistors and cable length |
| Bank select OK but `GYRO_CONFIG_1` stays at wrong value | Register write ignored | **Hard power reset**, then retry from Step 1 |

---

## Step 5 — Is USER_CTRL blocking register access?

```bash
python debug_user_ctrl.py
```

Reads `PWR_MGMT_1` and `USER_CTRL` bit-by-bit, writes `0x00` to `USER_CTRL` to disable
all potentially blocking features, then retries a `GYRO_CONFIG_1` write.

| Output | Meaning | What to do |
|--------|---------|------------|
| `GYRO_CONFIG_1 after write: 0x11` | USER_CTRL was the blocker; now clear | Proceed to Step 6 |
| `USER_CTRL after write` unchanged | USER_CTRL write itself failed | **Hard power reset** — chip fully locked |
| `I2C_IF_DIS` bit set (`Bit 5: 1`) | I2C interface disabled (SPI mode active) | Check SPI/I2C mode pin; this board may be in SPI-only mode |

---

## Step 6 — Which SMBus write method works?

```bash
python debug_write_methods.py
```

Tries five different SMBus write methods on `PWR_MGMT_1` and reports which (if any)
result in a changed register value.

| Output | Meaning | What to do |
|--------|---------|------------|
| Any method shows register value changed | That method works | Proceed to Step 7 |
| `write_word_data` avoids errors but value unchanged | Chip locked at I2C level | **Hard power reset**, then retry from Step 1 |
| All methods throw `OSError 121` | Chip completely locked | **Hard power reset** |
| All methods throw `OSError 5 (I/O error)` | I2C bus error | Check wiring; try `i2cdetect -y 1` to confirm bus is functional |

---

## Step 7 — Do config registers accept writes through the driver?

```bash
python debug_registers.py
```

Initializes the chip through the full `ICM20948` driver, then reads back `ACCEL_CONFIG`,
`GYRO_CONFIG_1`, and `PWR_MGMT_2` to verify the driver's init sequence actually took.

| Output | Meaning | What to do |
|--------|---------|------------|
| Register values match what was written | Driver init successful | Run `sensor_test.py` to confirm end-to-end |
| Registers stuck at `0x00` or `0x01` | Init sequence ran but writes were ignored | **Hard power reset**, then check if Steps 1–6 pass cleanly |
| Sensor data rows all zeros | Sensors not enabled | `PWR_MGMT_2` likely not `0x00` — driver init failed partway through |

---

## Final check — End-to-end sensor read

```bash
python sensor_test.py
```

Initializes through the full driver, auto-detects the address, and reads 5 samples of
accel, gyro, and temperature, plus one magnetometer read. Confirms all sensors are
responding with non-zero, changing data.

| Output | Meaning | What to do |
|--------|---------|------------|
| Accel/gyro values vary across rows, `All sensors responding OK` | Everything working | Done — use `../icm20948_test.py` for a full streaming run |
| `WARNING: accel Z is near zero` | Sensor connected but data not updating | Check `PWR_MGMT_2 = 0x00`; retry after power reset |
| `Mag: no data` on every run | Magnetometer init failed | Magnetometer issue only — accel/gyro still usable |
| `FAIL: ICM-20948 not found` | Chip not detected by driver | Go back to Step 1 |

---

## See Also

- Main driver: [../icm20948.py](../icm20948.py)
- Full streaming test with calibration: [../icm20948_test.py](../icm20948_test.py)
- Accel calibration: [../accel_calib.py](../accel_calib.py)
