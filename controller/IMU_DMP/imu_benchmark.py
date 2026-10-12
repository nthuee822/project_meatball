#!/usr/bin/env python3
"""ICM-20948 DMP 60-second verification benchmark.

Run from the controller/ root:
    python IMU_DMP/imu_benchmark.py [--duration 60]

Output: live 1-second summary + final statistics table.
Results saved to IMU_DMP/benchmark_results.json.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from IMU_DMP.dmp_imu import DMPIMU, DMPFrame

_RAD2DEG = 180.0 / math.pi


def _run(duration: int) -> None:
    print(f"[benchmark] Starting DMPIMU (this takes ~1 s for firmware load) …")
    imu = DMPIMU()
    imu.start()

    # Discard first second of packets while DMP settles
    settle = time.monotonic()
    while time.monotonic() - settle < 1.0:
        _ = imu.get_latest()
        time.sleep(0.01)

    print(f"[benchmark] Collecting for {duration} s …")
    print(f"{'Time':>6}  {'Rate':>7}  {'Roll':>7}  {'Pitch':>7}  {'Yaw':>7}  "
          f"{'gCal_z':>9}  {'FIFO':>5}  {'OvF':>3}")

    intervals_ms:  list[float] = []
    rolls:         list[float] = []
    pitches:       list[float] = []
    yaws:          list[float] = []
    gyro_z_cal:   list[float] = []
    overflow_count = 0
    drop_count     = 0
    packet_count   = 0

    t_start     = time.monotonic()
    t_last_tick = t_start
    count_this_sec = 0
    last_frame: DMPFrame | None = None

    for frame in imu.iter_frames():
        now = time.monotonic()
        elapsed = now - t_start
        if elapsed >= duration:
            break

        packet_count    += 1
        count_this_sec  += 1
        if frame.fifo_overflow:
            overflow_count += 1

        if last_frame is not None:
            intervals_ms.append(frame.packet_interval_ms)
            expected_ms = intervals_ms[0] if len(intervals_ms) == 1 else (
                sum(intervals_ms) / len(intervals_ms))
            if frame.packet_interval_ms > 2.5 * expected_ms:
                drop_count += 1

        rolls.append(math.degrees(frame.roll))
        pitches.append(math.degrees(frame.pitch))
        yaws.append(math.degrees(frame.yaw))
        gyro_z_cal.append(frame.gyro_cal[2])
        last_frame = frame

        # Print one line per second
        if now - t_last_tick >= 1.0:
            rate = count_this_sec / (now - t_last_tick)
            print(f"{elapsed:6.1f}s  {rate:6.1f}Hz  "
                  f"{math.degrees(frame.roll):7.2f}°  "
                  f"{math.degrees(frame.pitch):7.2f}°  "
                  f"{math.degrees(frame.yaw):7.2f}°  "
                  f"{frame.gyro_cal[2]:+9.4f}  "
                  f"{frame.fifo_count:5d}  "
                  f"{'Y' if frame.fifo_overflow else 'N':>3}")
            t_last_tick    = now
            count_this_sec = 0

    imu.stop()

    if not intervals_ms:
        print("[benchmark] No packets collected — check wiring / DMP init.")
        return

    total_time   = time.monotonic() - t_start
    actual_rate  = packet_count / total_time
    mean_int     = statistics.mean(intervals_ms)
    std_int      = statistics.stdev(intervals_ms) if len(intervals_ms) > 1 else 0.0
    min_int      = min(intervals_ms)
    max_int      = max(intervals_ms)
    within_20pct = sum(1 for x in intervals_ms if abs(x - mean_int) <= 0.2 * mean_int)
    jitter_pct   = 100.0 * within_20pct / len(intervals_ms)
    roll_noise   = statistics.stdev(rolls)
    pitch_noise  = statistics.stdev(pitches)
    yaw_noise    = statistics.stdev(yaws)
    gyro_z_mean  = statistics.mean(gyro_z_cal)

    # Assessment
    if (actual_rate >= 100 and std_int < 2.0 and overflow_count == 0
            and yaw_noise < 0.5 and drop_count == 0):
        assessment = "PASS"
    elif actual_rate >= 80 and overflow_count <= 2:
        assessment = "MARGINAL"
    else:
        assessment = "FAIL"

    results = {
        "duration_s":        total_time,
        "packets":           packet_count,
        "actual_rate_hz":    actual_rate,
        "interval_mean_ms":  mean_int,
        "interval_std_ms":   std_int,
        "interval_min_ms":   min_int,
        "interval_max_ms":   max_int,
        "jitter_within_20pct": jitter_pct,
        "fifo_overflow_count": overflow_count,
        "packet_drops":        drop_count,
        "static_noise_roll_deg":  roll_noise,
        "static_noise_pitch_deg": pitch_noise,
        "static_noise_yaw_deg":   yaw_noise,
        "gyro_z_cal_mean_rads":   gyro_z_mean,
        "assessment":         assessment,
    }

    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "benchmark_results.json")
    with open(out_path, "w") as fp:
        json.dump(results, fp, indent=2)

    print()
    print("=" * 60)
    print(f"  DMP Benchmark Results ({total_time:.1f} s)")
    print("=" * 60)
    print(f"  Packets collected      : {packet_count}")
    print(f"  Actual DMP rate        : {actual_rate:.1f} Hz")
    print(f"  Packet interval        : mean={mean_int:.2f}ms  std={std_int:.2f}ms  "
          f"min={min_int:.1f}ms  max={max_int:.1f}ms")
    print(f"  Jitter (within ±20%)   : {jitter_pct:.1f}%")
    print(f"  FIFO overflow count    : {overflow_count}")
    print(f"  Packet drops (>2.5×dt) : {drop_count}")
    print(f"  Static noise (std)     : roll={roll_noise:.3f}°  pitch={pitch_noise:.3f}°  "
          f"yaw={yaw_noise:.3f}°")
    print(f"  Gyro-Z bias residual   : {gyro_z_mean:+.5f} rad/s")
    print(f"  Assessment             : {assessment}")
    print(f"  Results saved to       : {out_path}")
    print("=" * 60)


def main() -> None:
    parser = argparse.ArgumentParser(description="ICM-20948 DMP benchmark")
    parser.add_argument("--duration", type=int, default=60,
                        help="Collection duration in seconds (default: 60)")
    args = parser.parse_args()
    _run(args.duration)


if __name__ == "__main__":
    main()
