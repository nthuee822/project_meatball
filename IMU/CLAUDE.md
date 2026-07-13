# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Python driver for the ICM-20948 9-axis IMU (accelerometer, gyroscope, magnetometer) on Raspberry Pi 5, using Linux I2C (`/dev/i2c-1`). The sensor communicates at address `0x68` or `0x69` depending on the `ADO/SDO` pin wiring.

## Setup Requirements

```bash
sudo apt install -y python3-smbus i2c-tools
# or: pip install smbus2
```

Enable I2C via `sudo raspi-config` → Interface Options → I2C.

Verify sensor detected: `sudo i2cdetect -y 1` (look for `68` or `69`).

Scripts may need `sudo` for I2C access: `sudo python3 icm20948_test.py`.

## Running Scripts

```bash
# Stream all sensor data with gyro calibration (main entry point)
python3 icm20948_test.py

# Advanced calibration flags
python3 icm20948_test.py --warmup-seconds 45 --calibration-samples 2500 --calibration-interval 0.01 --calibration-outlier-threshold 0.8

# Use the IMU wrapper class directly
python3 imu_test_run.py

# Synthetic demo mode (no hardware needed)
python3 imu_test_run.py --demo

# Interactive 6-position accelerometer calibration
python3 accel_calib.py

# Debug scripts (run in order to diagnose I2C problems)
cd debug/
python3 debug_addresses.py   # Step 1: can we reach the chip?
python3 debug_pwr.py         # Step 2: can we wake from SLEEP?
python3 debug_minimal.py     # Step 3: is data updating?
python3 debug_banks.py       # Step 4: does bank switching work?
python3 debug_user_ctrl.py   # Step 5: is USER_CTRL blocking?
python3 debug_write_methods.py # Step 6: which SMBus write method works?
python3 debug_registers.py   # Step 7: do driver writes take?
python3 sensor_test.py       # Final: end-to-end sensor read
```

## Architecture

**Two-layer driver design:**

- [icm20948.py](icm20948.py) — Low-level driver. Owns the SMBus handle, implements bank-switched register access (`_bank_sel`, `_write`, `_read`), magnetometer AUX I2C passthrough (`_mag_write`, `_mag_read`), and raw/scaled sensor reads. The `ICM20948` class is the minimal hardware interface.

- [imu.py](imu.py) — High-level `IMU` wrapper. Adds warmup, gyro bias calibration (with outlier rejection), and bias-corrected `read_gyro()`. Use this when building applications on top of the driver.

- [icm20948_test.py](icm20948_test.py) — CLI streaming script using `ICM20948` directly. Includes its own gyro calibration logic, rolling noise sigma display, and moving-average gyro output.

- [imu_test_run.py](imu_test_run.py) — CLI streaming script using the `IMU` wrapper. Has a `--demo` mode (`DemoIMU`) that generates synthetic sinusoidal data for testing without hardware.

- [accel_calib.py](accel_calib.py) — Interactive 6-position calibration. Prompts user to hold each axis against gravity and computes per-axis offset and scale factor.

**ICM-20948 register banking:** The chip has 4 register banks (0–3). `_bank_sel()` caches the current bank and only writes `REG_BANK_SEL` (0x7F) when switching, with a 5 ms delay after each switch.

**Magnetometer (AK09916):** Accessed indirectly through the ICM's internal I2C master (`USER_CTRL` bit 6). All mag reads/writes go through `I2C_SLV0_*` registers in bank 3, then data is retrieved from `EXT_SLV_SENS_DATA_00` in bank 0.

**SMBus backend:** `open_i2c_bus()` tries `smbus2` first, then falls back to `smbus`. Both work transparently.

## Known Hardware Issue: I2C Write Lockup

After a failed or interrupted initialization, the chip can enter a state where I2C writes silently fail (`OSError 121`). **Soft reset does not recover from this — only a hard power cycle (unplug USB-C, wait 5 s, reconnect) restores the chip.** This is a recurring issue in development; see `debug/README.md` for the full diagnostic sequence.
