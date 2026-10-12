#!/usr/bin/env python3
"""Live DMP output display — quaternion, Euler angles, accel, calibrated gyro.

Run from controller/ root:
    python IMU_DMP/dmp_live.py
    python IMU_DMP/dmp_live.py --rate 20    # limit print rate (default 10 Hz)
    python IMU_DMP/dmp_live.py --raw        # also show raw gyro
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from IMU_DMP.dmp_imu import DMPIMU

R2D = 180.0 / math.pi


def main() -> None:
    parser = argparse.ArgumentParser(description="ICM-20948 DMP live display")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="Print rate in Hz (default 10)")
    parser.add_argument("--raw", action="store_true",
                        help="Also print raw (uncalibrated) gyro")
    args = parser.parse_args()

    print("[dmp_live] Initializing ICM-20948 DMP (firmware load ~1 s) …")
    imu = DMPIMU()
    imu.start()

    print("[dmp_live] Running — Ctrl+C to quit\n")
    print(f"{'Roll':>8} {'Pitch':>8} {'Yaw':>8}  "
          f"{'Ax(g)':>7} {'Ay(g)':>7} {'Az(g)':>7}  "
          f"{'gCal_x':>9} {'gCal_y':>9} {'gCal_z':>9}  "
          f"{'dt(ms)':>7}  {'FIFO':>5}")
    print("-" * 110)

    min_dt = 1.0 / args.rate
    last_print = 0.0
    packet_count = 0
    start = time.monotonic()

    try:
        for frame in imu.iter_frames():
            packet_count += 1
            now = time.monotonic()
            if now - last_print < min_dt:
                continue
            last_print = now

            r = frame.roll  * R2D
            p = frame.pitch * R2D
            y = frame.yaw   * R2D

            ax, ay, az = frame.accel_raw
            gcx, gcy, gcz = frame.gyro_cal

            ovf = " OVF!" if frame.fifo_overflow else ""
            print(
                f"{r:8.2f} {p:8.2f} {y:8.2f}  "
                f"{ax:7.3f} {ay:7.3f} {az:7.3f}  "
                f"{gcx:+9.4f} {gcy:+9.4f} {gcz:+9.4f}  "
                f"{frame.packet_interval_ms:7.2f}  "
                f"{frame.fifo_count:5d}"
                f"{ovf}"
            )

            if args.raw:
                grx, gry, grz = frame.gyro_raw
                print(f"  raw gyro: {grx:+9.4f} {gry:+9.4f} {grz:+9.4f} rad/s  "
                      f"quat: w={frame.qw:.4f} x={frame.qx:.4f} y={frame.qy:.4f} z={frame.qz:.4f}")

    except KeyboardInterrupt:
        pass
    finally:
        imu.stop()
        elapsed = time.monotonic() - start
        print(f"\n[dmp_live] {packet_count} DMP packets in {elapsed:.1f} s "
              f"({packet_count/elapsed:.1f} Hz actual)")


if __name__ == "__main__":
    main()
