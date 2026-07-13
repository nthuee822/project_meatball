# ICM-20948 IMU on Raspberry Pi 5

This project ports IMU scripts to standard Python on Raspberry Pi 5 using Linux I2C (`/dev/i2c-1`). It communicates with an ICM-20948 9-axis sensor (accelerometer, gyroscope, magnetometer) and provides calibration and streaming capabilities.

---

## 1. First-Time Setup

### Hardware Wiring

Connect your ICM-20948 module to the Raspberry Pi 5 using the 40-pin GPIO header in I2C mode:

| IMU Pin | Raspberry Pi 5 Pin | Physical Pin |
|---------|---------------------|--------------|
| VCC | 3V3 Power | Pin 1 or 17 |
| GND | Ground | Pin 6, 9, 14, 20, 25, 30, 34, or 39 |
| SDA | GPIO2 (SDA1) | Pin 3 |
| SCL | GPIO3 (SCL1) | Pin 5 |
| NCS / CS | 3V3 Power | Pin 1 or 17 |
| ADO / SDO | GND (for address 0x68) or 3V3 (for 0x69) | Any GND or 3V3 pin |

Leave `EEDA`, `ECL`, `INT`, and `FSYNC` unconnected unless you need those features.

**⚠️ Critical wiring notes:**
- Use **3.3V only**—do NOT connect 5V to the IMU.
- `NCS` (chip select) must be tied high to 3.3V, or the chip will stay in SPI mode.
- Most bare modules default to address `0x68` (which requires `ADO/SDO -> GND`).

### Enable I2C on Raspberry Pi

1. Run the configuration utility:
   ```bash
   sudo raspi-config
   ```
2. Navigate to **Interface Options** → **I2C** and enable it.
3. Reboot if prompted:
   ```bash
   sudo reboot
   ```

### Install Required Packages

Update your system and install I2C tools and Python3 I2C support:

```bash
sudo apt update
sudo apt install -y python3-smbus i2c-tools
```

Alternatively, if you prefer to use `smbus2` from pip instead of the system `smbus`:

```bash
python3 -m pip install smbus2
```

Both backends work with these scripts.

### Verify the Sensor is Connected

Check that the IMU is detected on the I2C bus:

```bash
sudo i2cdetect -y 1
```

**Expected output** should show either `68` or `69` (depending on your `ADO/SDO` wiring):

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

**If nothing appears**, troubleshoot:
- Double-check that `SDA` and `SCL` are connected to pins 3 and 5.
- Confirm `NCS/CS` is tied to 3.3V (not floating or grounded).
- Verify `ADO/SDO` wiring matches the address you expect.
- Ensure the board is powered from 3.3V, not 5V.

---

## 2. Running Code on Raspberry Pi 5 from VS Code

Basically, we can directly run a python script without any VSCode extensions:

### Run a Python Script

1. Open the integrated terminal (Ctrl+`).
2. Navigate to your IMU project folder (if not already there):
   ```bash
   cd ~/path/to/your/IMU/project
   ```
3. Run any Python script, for example:
   ```bash
   python3 icm20948_test.py
   ```
4. Output will appear directly in the VS Code terminal.

---

## 3. Example Run & Expected Output

### Quick Test Run

To verify everything is working, run the main streaming example:

```bash
python3 icm20948_test.py
```

### Example Output

```
Detected ICM-20948 address(es): 0x68
Using /dev/i2c-1, address 0x68
ICM-20948 detected (WHO_AM_I = 0xEA)
Gyro calibration: keep the board completely still...
Bias: X=0.1100  Y=-0.0600  Z=0.0200 deg/s

Streaming data; press Ctrl-C to stop.
A: X= 0.012 Y=-0.003 Z= 1.001 g  |  G: X=   0.12 Y=  -0.08 Z=   0.02 deg/s  |  M: X=  23.40 Y=  -5.10 Z=  42.30 uT  |  28.3 C
A: X= 0.015 Y=-0.001 Z= 0.998 g  |  G: X=   0.08 Y=  -0.05 Z=   0.01 deg/s  |  M: X=  23.38 Y=  -5.12 Z=  42.28 uT  |  28.3 C
A: X= 0.010 Y=-0.004 Z= 1.002 g  |  G: X=   0.10 Y=  -0.07 Z=   0.03 deg/s  |  M: X=  23.42 Y=  -5.08 Z=  42.32 uT  |  28.3 C
```

**What each column means:**
- **A** (Accelerometer): measured in g (gravitational units). Z ≈ 1.0 g when the board is flat is expected.
- **G** (Gyroscope): measured in deg/s. Should be near zero when stationary.
- **M** (Magnetometer): measured in µT (microtesla).
- **Temperature**: in °C.

**Notes:**
- The first few magnetometer reads may show `(no data yet)` — this is normal.
- Press **Ctrl-C** to stop the stream.

### Advanced Calibration

For more accurate gyro calibration (recommended for stationary applications):

```bash
python3 icm20948_test.py \
  --warmup-seconds 45 \
  --calibration-samples 2500 \
  --calibration-interval 0.01 \
  --calibration-outlier-threshold 0.8
```

**Flags:**
- `--warmup-seconds`: Time to wait before collecting calibration data (allows sensor to stabilize).
- `--calibration-samples`: Number of samples to average for gyro bias.
- `--calibration-interval`: Delay (in seconds) between samples.
- `--calibration-outlier-threshold`: Threshold for rejecting outliers during calibration.
- `--noise-window`: Rolling window size for printing gyro noise sigma.
- `--gyro-average-window`: Moving-average window for displayed gyro values.
- `--address 0x69`: Use this if your module is wired for address 0x69.
- `--bus 1`: Explicitly specify I2C bus (usually 1 on Pi 5).

### Other Scripts

- **`debug.py`**: Raw accelerometer debug loop for troubleshooting.
- **`accel_calib.py`**: Interactive six-position accelerometer calibration.
- **`imu.py`** and **`icm20948.py`**: Reusable driver modules for the IMU.

---

## Troubleshooting

### Quick Diagnostics

**I2C not detected:**
- Verify `SDA` (pin 3) and `SCL` (pin 5) connections.
- Confirm `NCS/CS` is tied to 3.3V.
- Check power voltage—must be 3.3V only.

**Script fails with permission error:**
- I2C access may require `sudo`:
  ```bash
  sudo python3 icm20948_test.py
  ```

**Remote SSH connection fails:**
- Ensure SSH is enabled on the Raspberry Pi.
- Confirm your Pi is on the same network as your development machine.
- Test SSH connection from terminal first:
  ```bash
  ssh pi@<your-pi-ip-address>
  ```

### Common Sensor Issues

#### Problem: All Data Reads as Zero (Accel = 0, Gyro = 0, Mag = "no data yet")

**Likely causes:**
1. **Sensor in SLEEP mode** — not waking up properly after initialization
2. **I2C communication issue** — chip not responding to configuration writes
3. **Power supply unstable** — insufficient current during sensor operation
4. **Pull-up resistor problem** — weak or missing I2C pull-ups

**Diagnosis steps:**

**Step 1: Check if sensor responds at all**
```bash
i2cdump -y 1 0x68
```

Look for the `ea` value at address `0x00` (WHO_AM_I). If you see it:
- ✓ Sensor detected and readable
- ✗ Move to Step 2

If you see all `ff` or `00`:
- ✗ **Wiring issue**: Double-check SDA/SCL connections
- ✗ **Power issue**: Check 3.3V supply voltage

**Step 2: Check I2C voltage levels**

With sensor powered but **no Python code running**:
- Use a multimeter to measure voltage on **SDA line** (pin 3) → should be ~3.2–3.3V
- Use a multimeter to measure voltage on **SCL line** (pin 5) → should be ~3.2–3.3V
- Measure **VDD pin** on sensor → should be exactly 3.3V

If voltages are too low (< 3.0V):
- ✗ **Pull-up resistors too weak** or missing
- ✗ **Power supply sagging** — check current capacity

If voltages are correct:
- ✓ Proceed to Step 3

**Step 3: Test I2C write capability**

After power-cycling the sensor (disconnect/reconnect power), run:
```bash
python3 -c "
from icm20948 import open_i2c_bus, sleep_ms
bus = open_i2c_bus(1)
bus.write_byte_data(0x68, 0x06, 0x01)  # Write to PWR_MGMT_1
sleep_ms(100)
pwr = bus.read_byte_data(0x68, 0x06)
print(f'PWR_MGMT_1 after write: 0x{pwr:02X}')
print('SUCCESS: I2C writes working!' if pwr == 0x01 else 'FAILED: I2C writes not responding')
bus.close()
"
```

If successful:
- ✓ I2C communication healthy
- ✗ Driver may have an initialization issue — try running `icm20948_test.py`

If it fails or prints `0x41`:
- ✗ **I2C write failure** — see "I2C Write Lockup" section below

#### Problem: I2C Write Lockup (After First Run)

**Symptom**: First run works fine, but second run (without power cycle) produces all-zero data.

**Root cause**: The driver leaves the sensor in a locked state where I2C writes are rejected.

**Solution**: **Power-cycle the sensor**

1. Disconnect power (unplug USB or PSU from Raspberry Pi)
2. Wait 5 seconds
3. Reconnect power
4. Run your script again

The sensor should now respond normally.

**Why this happens:**
- After initialization completes, the sensor enters SLEEP mode with a register lock
- Soft reset (via `DEVICE_RESET` bit) does not recover from this state
- Only a hardware power-on-reset clears the lock

**Workaround for development**: If you need to restart scripts without unplugging:
- Modify the driver to issue a hard reset earlier in initialization
- Or add a GPIO-controlled power switch to the sensor

**For debugging**: See the `debug/` folder for detailed diagnostic scripts that were used to identify this issue:
```bash
cd debug/
python3 debug_minimal.py    # Minimal I2C test
python3 debug_pwr.py        # Power management tests
cat README.md               # Full debugging guide
```

#### Problem: Magnetometer Shows "(no data yet)" Indefinitely

**Likely causes:**
1. Magnetometer bus not initialized
2. Magnetometer chip unresponsive
3. Sensor still booting

**Quick fix**: Wait longer during initialization
```bash
python3 icm20948_test.py --warmup-seconds 60
```

If magnetometer still shows no data after 30 seconds of streaming:
- Check sensor power: `i2cdump -y 1 0x68` should show varying values around offset `0x3B`
- The magnetometer bus requires the I2C master to be enabled—ensure the driver initialized correctly

#### Problem: Gyro Noise is Very High

**Common cause**: Mechanical vibration from the Raspberry Pi or nearby equipment.

**Mitigation:**
- Mount the sensor on an isolated surface (damping foam, rubber feet)
- Reduce fan speed on the Raspberry Pi if it's vibrating
- Collect longer calibration windows:
  ```bash
  python3 icm20948_test.py \
    --warmup-seconds 45 \
    --calibration-samples 3000 \
    --calibration-outlier-threshold 0.5
  ```

For more details on gyro calibration, see `/memories/imu-calibration.md` (if available).

### Additional Help

For advanced debugging, refer to:
- **`debug/README.md`** — Detailed guide to using each diagnostic script
- **`icm20948.py`** — Driver source code with register definitions
- **ICM-20948 Datasheet** — Register reference and timing requirements
