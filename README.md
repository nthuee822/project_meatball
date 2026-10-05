# project_meatball

A custom three-wheel omnidirectional robot: radial-mount wheels driven by
FOC-controlled motors with magnetic angle encoders, steered by a Logitech F710
gamepad through a Raspberry Pi, with IMU-based orientation sensing and
custom PCBs and 3D-printed mechanical parts.

## Repository Layout

| Directory | Contents |
|---|---|
| [`controller/`](controller/) | Raspberry Pi Python controller: reads the gamepad, computes inverse kinematics for the three radial wheels, and streams motor commands over SPI (optionally fused with IMU data). |
| [`IMU/`](IMU/) | Standalone Python driver for the ICM-20948 9-axis IMU (accel/gyro/mag) over I2C on the Raspberry Pi, plus calibration and debug scripts. |
| [`stm32/`](stm32/) | STM32CubeIDE firmware projects for the per-wheel motor drivers: field-oriented control (`foc_v0`, `foc_v1`), AS5600 magnetic encoder angle measurement (`angle_measurement_test`, `angle_measurement_kalman`), ADC alignment testing, and a stepper/SPI test build. `common_lib/` holds the shared AS5600 driver. |
| [`hardware/`](hardware/) | KiCad PCB projects: current-sense simulation, the `ht7k1401` breakout board, and a shared component library. |
| [`3d_model/`](3d_model/) | FreeCAD/3MF models for printed parts — motor holders, hall-sensor holders/washers, magnet mold, and corner brackets. |
| [`architecture/`](architecture/) | System architecture diagrams (drawio source + exported PDF). |
| [`images/`](images/) | Reference photos and renders. |

## System Overview

```
F710 gamepad --USB--> Raspberry Pi (controller/) --SPI--> STM32 motor drivers (stm32/foc_*)
                              ^                                    |
                              |                                    v
                        IMU/ (ICM-20948)                 AS5600 encoder + motor
```

Each wheel's STM32 board runs closed-loop FOC using position feedback from an
AS5600 magnetic encoder (angle read directly, or Kalman-filtered — see
`stm32/angle_measurement_kalman`). The Raspberry Pi computes per-wheel speed/
direction from gamepad input via radial-mount inverse kinematics and sends
them as fixed-format SPI frames; see [`controller/README.md`](controller/README.md)
for the protocol and kinematics details.

## Getting Started

### Python Environment (uv)

All Python code (`controller/`, `IMU/`, `stm32/*/` scripts) shares one
environment managed by [uv](https://docs.astral.sh/uv/), defined in
`pyproject.toml` and pinned by `uv.lock`. It lives in `.venv/` at the repo
root and does not touch your system Python.

```bash
# 1. Install uv (once per machine)
curl -LsSf https://astral.sh/uv/install.sh | sh          # Linux / macOS
# powershell -c "irm https://astral.sh/uv/install.ps1 | iex"   # Windows

# 2. Create .venv and install dependencies (re-run after pulling changes)
uv sync

# 3. Run scripts through uv (works from any subdirectory)
cd controller && uv run python run_test.py
```

Hardware libraries (`smbus2`, `spidev`, `rpi-lgpio` which provides
`RPi.GPIO`) are only installed on Raspberry Pi (Linux ARM64). On a laptop,
`uv sync` skips them, so you can still run the hardware-independent parts
(kinematics, gamepad/keyboard input, plotting).

On the Raspberry Pi, `rpi-lgpio` is compiled from source, so install the build
dependencies once before the first `uv sync`:

```bash
sudo apt install -y swig liblgpio-dev
```

To add a dependency: `uv add <package>` (commit both `pyproject.toml` and `uv.lock`).

### Subsystems

Each subsystem has its own setup instructions:

- Robot controller (gamepad + kinematics + SPI): [`controller/README.md`](controller/README.md)
- IMU driver and wiring/calibration: [`IMU/README.md`](IMU/README.md)
- STM32 firmware: open the relevant `.ioc` project in STM32CubeIDE (each project directory is self-contained with CMake presets for building outside the IDE).
- PCB designs: open the `.kicad_pro` files in KiCad.
- 3D models: open the `.FCStd` files in FreeCAD; `.3mf` files are ready to slice/print.
