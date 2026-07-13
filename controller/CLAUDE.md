# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

A modular three-wheel omnidirectional robot controller for Raspberry Pi, using a Logitech F710 gamepad over USB. The robot uses radial-mount wheels (wheels point toward center) with pure translation via inverse kinematics. Rotation is achieved by adding a uniform velocity offset to all three wheels (exploiting mechanical mounting error).

## Running the Controller

**Display mode (no hardware required, for debugging kinematics):**
```bash
python omni_robot.py
```

**SPI output mode (requires Raspberry Pi with SPI enabled):**
```bash
python controller2spi.py           # concise output
python controller2spi.py --verbose # detailed output
```

**SPI + IMU mode (adds ICM-20948 gyro/accel/mag):**
```bash
python ctrl_with_imu_2spi.py
python ctrl_with_imu_2spi.py --loop-hz 50 --enable-mag --gyro-calib-samples 200
```

**Kinematics unit tests:**
```bash
python run_test.py
```

**Gamepad button/axis mapping test:**
```bash
python test.py
```

## Dependencies

```bash
# Basic (display mode)
pip install pygame numpy

# SPI mode (Raspberry Pi only)
pip install pygame numpy spidev RPi.GPIO

# IMU mode (additional)
sudo apt install python3-smbus i2c-tools
# or: pip install smbus2
```

Enable SPI and I2C on the Pi via `sudo raspi-config` → Interface Options.

## Architecture

### Class Hierarchy

```
OmniRobotController          (omni_robot.py)
├── OmniRobotSPIController   (controller2spi.py)  — adds SPI motor output
├── OmniRobotSPIController   (4test.py)            — adds fixed yaw compensation
└── OmniRobotIMUSPIController (ctrl_with_imu_2spi.py) — adds ICM-20948 IMU
```

`OmniRobotController` owns:
- `F710Controller` (f710controller.py) — pygame joystick reader, headless (`SDL_VIDEODRIVER=dummy`)
- `OmniKinematics` (inverse_kinematics.py) — pure math, no hardware dependency
- Slew-rate limiter for smooth acceleration/deceleration

### Data Flow

```
Gamepad axes → deadzone filter → slew-rate limiter
    → OmniKinematics.calculate(vx, vy, wz) → normalize()
    → rps_to_motor_params() → create_spi_data() → spi_transfer(CS0/1/2)
                                                  → angle encoding → spi_transfer(CS3)
```

### SPI Protocol (16-bit per motor)

```
Bit 15:    Enable (1=on, 0=brake/stop)
Bit 14:    Direction (1=forward, 0=reverse)
Bits 13-12: Reserved (0)
Bits 11-0:  Period parameter p (12-bit, 0=stop, 1–4095: RPS = 10000/64/p)
```

GPIO chip-selects (BCM): CS0=GPIO17 (wheel 1 front), CS1=GPIO27 (wheel 2 rear-left), CS2=GPIO22 (wheel 3 rear-right), CS3=GPIO5 (travel angle channel, 0–360° encoded as uint16).

### Inverse Kinematics (radial mount)

```python
v1 =  vy + wz                          # wheel 1 (front)
v2 = -0.5*vy - 0.866*vx + wz          # wheel 2 (rear-left)
v3 = -0.5*vy + 0.866*vx + wz          # wheel 3 (rear-right)
```

Input vx/vy/wz are −1.0…+1.0 percentages. `normalize()` scales all three wheels proportionally if any exceeds 1.0, preserving direction.

### Key Parameters (omni_robot.py)

| Constant | Default | Meaning |
|---|---|---|
| `MOTOR_MAX_RPS` | 1.5 | Safety speed ceiling |
| `MOTOR_K` | 10000/64 | Converts RPS→p: `p = K/RPS` |
| `ACCEL_RATE_UP` | 0.5 | Max accel per second (50%/s) |
| `ACCEL_RATE_DOWN` | 5.0 | Max decel per second (500%/s) |
| `UPDATE_RATE` | 0.05 s | Loop period (20 Hz) |

### IMU Module (IMU_DMP/)

`IMU_DMP/icm20948.py` is a standalone Linux I2C driver for the ICM-20948 (accel + gyro + AK09916 magnetometer). It auto-selects `smbus2` or `smbus` as backend. `ctrl_with_imu_2spi.py` imports it as `from IMU.icm20948 import ...` — ensure the directory is accessible as `IMU/` (symlink or rename may be needed if directory is `IMU_DMP/`).

Gyro bias calibration runs at startup when `--gyro-calib-samples > 0`; uses median filtering to reject outliers before averaging.

### 4test.py (Anti-Interference Build)

Variant of the SPI controller that maps triggers to X-axis translation only (no left-stick), and adds a `FIXED_YAW_COMP = -0.1` RPS offset applied with slew-rate limiting to counteract yaw drift from unequal wheel mounting.
