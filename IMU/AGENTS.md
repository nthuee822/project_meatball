# Repository Guidelines

## Project Structure & Module Organization

This repository contains Raspberry Pi 5 Python code for the ICM-20948 IMU over Linux I2C. Core modules live at the repository root:

- `icm20948.py`: low-level register, bank switching, SMBus, and magnetometer access.
- `imu.py`: higher-level wrapper with warmup and gyro calibration.
- `icm20948_test.py`, `imu_test_run.py`, `accel_calib.py`: runnable scripts for streaming and calibration.
- `debug/`: step-by-step diagnostic scripts and a local debug copy of the driver.
- `Arduino_ICM20948_DMP_Full-Function/`: upstream/reference Arduino and legacy Raspberry Pi material.
- `datasheet.pdf` and `application_note.pdf`: linked hardware references.

## Build, Test, and Development Commands

There is no build step. Install runtime support on a Raspberry Pi:

```bash
sudo apt install -y python3-smbus i2c-tools
python3 -m pip install smbus2  # optional alternative backend
```

Common commands:

```bash
sudo i2cdetect -y 1
python3 icm20948_test.py
python3 imu_test_run.py --demo
python3 accel_calib.py
cd debug && python3 debug_addresses.py
```

Use `i2cdetect` to confirm address `0x68` or `0x69`. Run hardware scripts with `sudo` if normal user access to `/dev/i2c-1` fails.

## Coding Style & Naming Conventions

Use Python 3 with 4-space indentation, type hints where they clarify public interfaces, and module docstrings for runnable scripts. Follow existing naming: `snake_case` for functions and variables, `UPPER_CASE` for register constants, and private helpers prefixed with `_`. Keep hardware constants near the top of driver modules. Prefer explicit delays and clear exceptions around I2C operations.

## Testing Guidelines

Tests are mostly hardware-facing scripts rather than automated unit tests. Use `python3 imu_test_run.py --demo` for a no-hardware smoke test. For real hardware, run `python3 icm20948_test.py` and verify accelerometer, gyro, magnetometer, and temperature output. When debugging I2C failures, follow `debug/README.md` order rather than jumping between scripts.

## Commit & Pull Request Guidelines

Git history uses short, imperative or descriptive summaries, for example `added debug related script & instructions` and `re-wrote README.md; other files are to be checked`. Keep commits focused on one behavior or documentation change. Pull requests should describe hardware used, commands run, observed sensor address, and any calibration or power-cycle requirements. Include screenshots or terminal excerpts for changed streaming/debug output.

## Security & Configuration Tips

Do not commit local machine paths, private network addresses, or generated `__pycache__/` files. Use 3.3V wiring only, keep `NCS/CS` tied high for I2C mode, and document any required hard power cycle when diagnosing write lockups.
