# Repository Guidelines

## Project Structure & Module Organization

This repository contains a Python controller for a three-wheel omnidirectional robot using a Logitech F710 gamepad. Core modules live at the root:

- `omni_robot.py`: display/debug controller with no SPI hardware requirement.
- `controller2spi.py`: Raspberry Pi SPI motor-output controller.
- `ctrl_with_imu_2spi.py`: SPI controller with ICM-20948 IMU support.
- `f710controller.py`: pygame-based gamepad input.
- `inverse_kinematics.py`: hardware-independent kinematics math.
- `IMU_DMP/`: ICM-20948 driver, calibration data, diagnostics, and IMU tools.
- `run_test.py`, `test.py`, `4test.py`: quick validation, gamepad mapping, and controller variants.

## Build, Test, and Development Commands

Create and use a virtual environment before installing dependencies:

```bash
python -m venv .venv
source .venv/bin/activate
pip install pygame numpy
```

For Raspberry Pi SPI mode, also install `spidev` and `RPi.GPIO`. For IMU mode, install `smbus2` or `python3-smbus` and enable SPI/I2C through `raspi-config`.

Common commands:

```bash
python omni_robot.py              # run display/debug mode
python controller2spi.py          # run SPI mode
python controller2spi.py --verbose
python ctrl_with_imu_2spi.py --loop-hz 50
python run_test.py                # validate inverse kinematics expectations
python test.py                    # inspect F710 button/axis mapping
```

## Coding Style & Naming Conventions

Use Python 3 with 4-space indentation. Keep hardware access isolated from pure logic: kinematics belongs in `inverse_kinematics.py`, controller input in `f710controller.py`, and GPIO/SPI code in SPI-specific modules. Use `snake_case` for functions and variables, and keep established constants such as `MOTOR_MAX_RPS` and `TRIGGER_DEADZONE`. Prefer clear names for wheel order and GPIO mappings.

## Testing Guidelines

There is no formal pytest suite yet. Use `python run_test.py` for kinematics regression checks before changing equations, normalization, or speed limits. Use `python omni_robot.py` for hardware-free behavior checks. Run SPI and IMU commands only on a configured Raspberry Pi with the expected motor controller, F710 receiver, SPI, and I2C wiring attached.

## Commit & Pull Request Guidelines

Recent commits use concise Chinese summaries describing changed behavior, for example `新增三輪全向輪機器人主程式...` and `更新 README...`. Follow that style: one focused summary line, in Chinese or English, stating the functional change. Pull requests should include the affected mode (`display`, `SPI`, or `SPI+IMU`), hardware used for validation, commands run, and wiring or calibration assumptions. Include screenshots or terminal excerpts when changing displayed output.

## Security & Configuration Tips

Do not commit local calibration experiments unless they are intended defaults. Treat GPIO pin assignments, SPI chip selects, and IMU calibration files as hardware-specific configuration. Before running motor-output code, verify emergency stop behavior and ensure the robot is physically safe to move.
