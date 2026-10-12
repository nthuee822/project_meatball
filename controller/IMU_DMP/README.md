# IMU_DMP — ICM-20948 DMP Driver

ICM-20948 9-axis IMU with Digital Motion Processor (DMP) integration for the
omnidirectional robot controller.  The DMP runs onboard sensor fusion at 112 Hz,
outputting 6-axis quaternions and dynamically-calibrated gyro data.

---

## Hardware Wiring

| Signal | Pin  | Note |
|--------|------|------|
| VCC    | 3.3 V | Do **not** use 5 V |
| GND    | GND  | |
| SDA    | GPIO 2 | I²C bus 1 |
| SCL    | GPIO 3 | I²C bus 1 |
| NCS    | 3.3 V (tied high) | Forces I²C mode |
| ADO    | GND  | I²C address = 0x68 |
| INT    | (not connected) | FIFO-poll mode used |

Verify address: `i2cdetect -y 1` → expect `0x68`.

---

## Dependencies

```bash
sudo apt install python3-smbus i2c-tools   # smbus backend
# or:
uv sync                                    # recommended: installs smbus2 from repo-root pyproject.toml
```

Enable I²C: `sudo raspi-config` → Interface Options → I2C.

---

## Files

| File | Purpose |
|------|---------|
| `icm20948.py` | Low-level I²C register driver (accel / gyro / mag) |
| `dmp_imu.py` | **DMP driver** — firmware load, FIFO reader, `DMPFrame` output |
| `_dmp_firmware.py` | DMP 3.1a firmware bytes (~14 KB, SparkFun/MIT) |
| `imu_benchmark.py` | 60-second verification and statistics tool |
| `accel_calib.py` | Interactive 6-position accelerometer calibration |
| `imu_calibration.json` | Saved accel/gyro calibration values |
| `application_note-*.pdf` | InvenSense DMP programming reference (ICM-20648) |

---

## DMP Initialization Flow

Following InvenSense application note AN-MAPPS (ICM-20648 DMP, same architecture):

1. Wake chip, CLKSEL=1 (PLL auto)
2. Enable accel + gyro: ±2000 dps / ±4g, DLPF on, 225 Hz sample rate
3. Configure FIFO (DMP manages FIFO; hw sensor FIFO disabled)
4. Reset FIFO
5. Set DMP program start address (0x1000)
6. Load 14 301-byte DMP firmware via I²C burst writes to DMP SRAM
7. Configure accel scale to DMP internal format (1g = 2²⁵)
8. Configure accel calibration gains for 112 Hz
9. Compute and write GYRO_SF from TIMEBASE_CORRECTION_PLL
10. Seed gyro bias registers from `imu_calibration.json`
11. Set `DATA_OUT_CTL1 = 0xCC40`: output Accel + Gyro + Quat6 + CalibratedGyro
12. Set ODR = 112 Hz for all channels (`(225 / 112) − 1 = 1`)
13. Enable dynamic accel + gyro calibration via `MOTION_EVENT_CTL`
14. Start DMP + enable FIFO (`USER_CTRL = 0xCE`)
15. Enable DMP interrupt flag

---

## FIFO Packet Format

`DATA_OUT_CTL1 = 0xCC40` → **38 bytes** per packet:

| Field | Bytes | Type | Scale | Unit |
|-------|-------|------|-------|------|
| Header | 2 | uint16 BE | — | bitmask |
| Accel X/Y/Z | 6 | int16 BE × 3 | ÷ 8192 | g |
| Gyro raw X/Y/Z | 6 | int16 BE × 3 | ÷ 16.384 × π/180 | rad/s |
| Quaternion Q1/Q2/Q3 | 12 | int32 BE × 3 | ÷ 2³⁰ | — |
| Calibrated gyro X/Y/Z | 12 | int32 BE × 3 | ÷ 2¹⁵ ÷ 16.384 × π/180 | rad/s |

Q0 is derived: `Q0 = sqrt(max(0, 1 − Q1² − Q2² − Q3²))`.

---

## DMPFrame Fields

```python
@dataclass
class DMPFrame:
    timestamp: float      # time.monotonic(), seconds
    dt: float             # seconds since previous packet
    qw: float             # quaternion w (derived)
    qx: float             # quaternion x (Q1)
    qy: float             # quaternion y (Q2)
    qz: float             # quaternion z (Q3)
    roll: float           # radians
    pitch: float          # radians
    yaw: float            # radians
    gyro_raw: tuple       # (gx, gy, gz) rad/s — raw, from FIFO
    gyro_cal: tuple       # (gx, gy, gz) rad/s — DMP-calibrated
    accel_raw: tuple      # (ax, ay, az) g
    fifo_count: int       # FIFO bytes at read time
    fifo_overflow: bool   # True if FIFO overflow detected
    packet_interval_ms: float
```

---

## Usage

```python
import sys
sys.path.insert(0, "/home/meatball/controller")   # or run from controller/
from IMU_DMP.dmp_imu import DMPIMU
import math, time

imu = DMPIMU()          # i2c_bus=1, i2c_addr=0x68 (defaults)
imu.start()             # loads firmware, ~1 s

# Non-blocking — returns latest frame or None
frame = imu.get_latest()
if frame:
    print(f"yaw = {math.degrees(frame.yaw):.1f} deg")

# Blocking iterator — yields every packet at 112 Hz
for frame in imu.iter_frames():
    vx, vy, wz = get_joystick()
    # use frame.yaw for heading correction
    # use frame.gyro_cal for angular velocity
    # use frame.dt for integrator timestep
    ...

imu.stop()
```

---

## Running the Benchmark

```bash
cd /home/meatball/controller
python IMU_DMP/imu_benchmark.py             # 60-second run (default)
python IMU_DMP/imu_benchmark.py --duration 10   # quick 10-second test
```

Expected output (robot stationary):

```
=== DMP Benchmark Results (60.0 s) ===
  Packets collected      : 6720
  Actual DMP rate        : 112.0 Hz
  Packet interval        : mean=8.93ms  std=0.40ms  min=8.1ms  max=11.5ms
  Jitter (within 20pct   : 99.7%
  FIFO overflow count    : 0
  Packet drops (>2.5xdt) : 0
  Static noise (std)     : roll=0.012 deg  pitch=0.008 deg  yaw=0.031 deg
  Gyro-Z bias residual   : +0.00012 rad/s
  Assessment             : PASS
```

Results are saved to `IMU_DMP/benchmark_results.json`.

### Pass criteria

| Metric | Pass | Marginal | Fail |
|--------|------|----------|------|
| Rate | >= 100 Hz | >= 80 Hz | < 80 Hz |
| Interval std | < 2 ms | — | — |
| FIFO overflows | 0 | <= 2 | > 2 |
| Yaw noise std | < 0.5 deg | — | — |
| Packet drops | 0 | — | > 0 |

---

## Control Recommendations

| Parameter | Recommendation |
|-----------|---------------|
| Orientation | Use `qw/qx/qy/qz` -> `roll/pitch/yaw` from `DMPFrame` |
| Angular velocity | Use `gyro_cal` (DMP dynamic calibration, 32-bit) |
| Integrator timestep | Use `frame.dt` (variable, real measured interval) |
| Fallback | If FIFO overflow > 0, switch to Madgwick fusion in `icm20948.py` |

The DMP automatically updates gyro/accel bias via `MOTION_EVENT_CTL = 0x0300`.
No separate startup calibration loop is needed for heading drift compensation.

### ODR formula

```
ODR register value = (225 / target_Hz) - 1
112 Hz  ->  1      (default, recommended for closed-loop control)
56 Hz   ->  3      (lower CPU load)
225 Hz  ->  0      (max rate, higher FIFO fill risk)
```
