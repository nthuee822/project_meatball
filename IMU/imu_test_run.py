"""Standalone runner for the reusable IMU wrapper."""

from __future__ import annotations

import argparse
import math
import time

from imu import IMU


class DemoIMU:
    def __init__(self) -> None:
        self._start = time.monotonic()

    def close(self) -> None:
        return None

    def read_who_am_i(self) -> int:
        return 0xEA

    def read_accel(self) -> tuple[float, float, float]:
        t = time.monotonic() - self._start
        return (
            0.01 * math.sin(t * 0.7),
            0.01 * math.cos(t * 0.5),
            1.0 + 0.01 * math.sin(t * 0.3),
        )

    def read_gyro(self) -> tuple[float, float, float]:
        t = time.monotonic() - self._start
        return (
            0.2 * math.sin(t * 0.9),
            0.2 * math.cos(t * 0.8),
            0.1 * math.sin(t * 0.6),
        )

    def read_mag(self) -> tuple[float, float, float] | None:
        t = time.monotonic() - self._start
        return (
            25.0 + 2.0 * math.sin(t * 0.4),
            -4.0 + 1.5 * math.cos(t * 0.3),
            41.0 + 1.0 * math.sin(t * 0.2),
        )

    def read_temp(self) -> float:
        t = time.monotonic() - self._start
        return 28.0 + 0.5 * math.sin(t * 0.1)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Run against a local synthetic IMU instead of hardware.",
    )
    parser.add_argument("--bus", type=int, default=1, help="Linux I2C bus number.")
    parser.add_argument(
        "--address",
        type=lambda value: int(value, 0),
        default=None,
        help="ICM-20948 I2C address (default: auto-detect 0x68/0x69).",
    )
    parser.add_argument(
        "--warmup-seconds",
        type=float,
        default=30.0,
        help="Seconds to let the IMU settle before gyro calibration.",
    )
    parser.add_argument(
        "--calibration-samples",
        type=int,
        default=1500,
        help="Samples to average for gyro bias calibration.",
    )
    parser.add_argument(
        "--calibration-interval",
        type=float,
        default=0.01,
        help="Seconds between gyro calibration samples.",
    )
    parser.add_argument(
        "--calibration-outlier-threshold",
        type=float,
        default=1.0,
        help="Reject calibration samples farther than this deg/s from median vector.",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.2,
        help="Seconds between streamed samples.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.demo:
        imu = DemoIMU()
    else:
        imu = IMU(
            bus_id=args.bus,
            address=args.address,
            warmup_seconds=args.warmup_seconds,
            calibration_samples=args.calibration_samples,
            calibration_interval=args.calibration_interval,
            calibration_outlier_threshold=args.calibration_outlier_threshold,
        )

    try:
        print(f"IMU ready (WHO_AM_I = 0x{imu.read_who_am_i():02X})")
        print("Streaming data; press Ctrl-C to stop.\n")

        while True:
            ax, ay, az = imu.read_accel()
            gx, gy, gz = imu.read_gyro()
            mag = imu.read_mag()
            temp = imu.read_temp()

            mag_str = (
                f"X={mag[0]:7.2f} Y={mag[1]:7.2f} Z={mag[2]:7.2f}"
                if mag
                else "       (no data yet)       "
            )

            print(
                f"A: X={ax:6.3f} Y={ay:6.3f} Z={az:6.3f} g  |  "
                f"G: X={gx:7.3f} Y={gy:7.3f} Z={gz:7.3f} deg/s  |  "
                f"M: {mag_str} uT  |  "
                f"{temp:.1f} C"
            )
            time.sleep(args.delay)
    except RuntimeError as error:
        print(error)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        imu.close()


if __name__ == "__main__":
    main()