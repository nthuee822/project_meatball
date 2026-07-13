#!/usr/bin/env python3
"""Read ICM-20948 raw six-axis data and tilt angles on Raspberry Pi."""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time

from icm20948 import ICM20948, ImuSample


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read raw accel/gyro and roll/pitch tilt from ICM-20948 over I2C."
    )
    parser.add_argument("--bus", type=int, default=1, help="Linux I2C bus number")
    parser.add_argument(
        "--address",
        type=lambda value: int(value, 0),
        default=None,
        help="I2C address, usually 0x68 or 0x69. Auto-detect by default.",
    )
    parser.add_argument("--rate", type=float, default=100.0, help="IMU read rate in Hz")
    parser.add_argument("--samples", type=int, default=0, help="Read sample count; 0 means run forever")
    parser.add_argument(
        "--output-rate",
        type=float,
        default=0.0,
        help="Maximum output rate in Hz; 0 means use --output-every",
    )
    parser.add_argument(
        "--output-every",
        type=int,
        default=1,
        help="Print one row every N read samples when --output-rate is 0",
    )
    parser.add_argument(
        "--flush-every",
        type=int,
        default=20,
        help="Flush stdout every N printed rows; 0 flushes only at exit",
    )
    parser.add_argument(
        "--combined-read",
        action="store_true",
        help="Shortcut for --read-mode combined.",
    )
    parser.add_argument(
        "--read-mode",
        choices=("byte", "block", "combined"),
        default="byte",
        help="I2C read method. byte is slowest but most compatible.",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=3,
        help="Retries for each IMU sample after an I2C read error.",
    )
    parser.add_argument(
        "--max-consecutive-errors",
        type=int,
        default=25,
        help="Stop only after this many consecutive read failures; 0 means never stop for read failures.",
    )
    parser.add_argument(
        "--error-backoff",
        type=float,
        default=0.02,
        help="Seconds to sleep after a skipped read error.",
    )
    parser.add_argument(
        "--accel-range",
        type=int,
        default=4,
        choices=(2, 4, 8, 16),
        help="Accelerometer full-scale range in g",
    )
    parser.add_argument(
        "--gyro-range",
        type=int,
        default=2000,
        choices=(250, 500, 1000, 2000),
        help="Gyroscope full-scale range in dps",
    )
    parser.add_argument(
        "--calibrate-samples",
        type=int,
        default=100,
        help="Stationary samples used to estimate gyro bias; set 0 to disable",
    )
    parser.add_argument(
        "--alpha",
        type=float,
        default=0.98,
        help="Complementary filter gyro weight, between 0 and 1",
    )
    parser.add_argument(
        "--format",
        choices=("text", "csv", "json", "none"),
        default="text",
        help="Output format",
    )
    return parser.parse_args()


def estimate_gyro_bias(
    sensor: ICM20948,
    count: int,
    delay_s: float,
    max_consecutive_errors: int,
    error_backoff_s: float,
) -> tuple[float, float, float]:
    if count <= 0:
        return (0.0, 0.0, 0.0)

    sums = [0.0, 0.0, 0.0]
    accepted = 0
    consecutive_errors = 0
    while accepted < count:
        try:
            sample = sensor.read_sample()
        except RuntimeError as exc:
            consecutive_errors += 1
            print(f"warning: skipped calibration sample: {exc}", file=sys.stderr)
            if max_consecutive_errors and consecutive_errors >= max_consecutive_errors:
                raise RuntimeError(
                    f"Too many consecutive I2C errors during calibration ({consecutive_errors})"
                ) from exc
            time.sleep(error_backoff_s)
            continue

        consecutive_errors = 0
        for axis in range(3):
            sums[axis] += sample.gyro_dps[axis]
        accepted += 1
        time.sleep(delay_s)
    return tuple(value / count for value in sums)


def read_sample_or_skip(
    sensor: ICM20948,
    label: str,
    max_consecutive_errors: int,
    error_backoff_s: float,
) -> ImuSample:
    consecutive_errors = 0
    while True:
        try:
            return sensor.read_sample()
        except RuntimeError as exc:
            consecutive_errors += 1
            print(f"warning: skipped {label}: {exc}", file=sys.stderr)
            if max_consecutive_errors and consecutive_errors >= max_consecutive_errors:
                raise RuntimeError(
                    f"Too many consecutive I2C errors while reading {label} ({consecutive_errors})"
                ) from exc
            time.sleep(error_backoff_s)


def emit_header(args: argparse.Namespace) -> csv.writer | None:
    if args.format == "csv":
        writer = csv.writer(sys.stdout)
        writer.writerow(
            [
                "timestamp_s",
                "ax_raw",
                "ay_raw",
                "az_raw",
                "gx_raw",
                "gy_raw",
                "gz_raw",
                "ax_g",
                "ay_g",
                "az_g",
                "gx_dps",
                "gy_dps",
                "gz_dps",
                "roll_accel_deg",
                "pitch_accel_deg",
                "roll_filter_deg",
                "pitch_filter_deg",
            ]
        )
        return writer
    return None


def main() -> int:
    args = parse_args()
    if args.rate <= 0:
        raise SystemExit("--rate must be positive")
    if args.output_rate < 0:
        raise SystemExit("--output-rate cannot be negative")
    if args.output_every <= 0:
        raise SystemExit("--output-every must be positive")
    if args.flush_every < 0:
        raise SystemExit("--flush-every cannot be negative")
    if args.retries < 0:
        raise SystemExit("--retries cannot be negative")
    if args.max_consecutive_errors < 0:
        raise SystemExit("--max-consecutive-errors cannot be negative")
    if args.error_backoff < 0:
        raise SystemExit("--error-backoff cannot be negative")
    if not 0.0 <= args.alpha <= 1.0:
        raise SystemExit("--alpha must be between 0 and 1")

    period_s = 1.0 / args.rate
    output_period_s = 1.0 / args.output_rate if args.output_rate > 0 else 0.0
    read_mode = "combined" if args.combined_read else args.read_mode
    sensor: ICM20948 | None = None

    try:
        sensor = ICM20948(
            bus=args.bus,
            address=args.address,
            accel_range_g=args.accel_range,
            gyro_range_dps=args.gyro_range,
            sample_rate_hz=max(1, int(args.rate)),
            read_mode=read_mode,
            retries=args.retries,
        )
        sensor.initialize()
        bias = estimate_gyro_bias(
            sensor,
            args.calibrate_samples,
            period_s,
            args.max_consecutive_errors,
            args.error_backoff,
        )
        writer = emit_header(args)

        initial = read_sample_or_skip(
            sensor,
            "initial sample",
            args.max_consecutive_errors,
            args.error_backoff,
        )
        roll_filter = initial.roll_deg
        pitch_filter = initial.pitch_deg
        last_time = time.monotonic()
        index = 0
        printed = 0
        next_output_time = last_time
        consecutive_errors = 0

        while args.samples == 0 or index < args.samples:
            loop_start = time.monotonic()
            try:
                sample = sensor.read_sample()
            except RuntimeError as exc:
                consecutive_errors += 1
                print(f"warning: skipped sample {index}: {exc}", file=sys.stderr)
                if args.max_consecutive_errors and consecutive_errors >= args.max_consecutive_errors:
                    raise RuntimeError(
                        f"Too many consecutive I2C read errors ({consecutive_errors})"
                    ) from exc
                time.sleep(args.error_backoff)
                index += 1
                continue

            consecutive_errors = 0
            now = time.monotonic()
            dt = now - last_time
            last_time = now

            gx = sample.gyro_dps[0] - bias[0]
            gy = sample.gyro_dps[1] - bias[1]

            roll_filter = args.alpha * (roll_filter + gx * dt) + (1.0 - args.alpha) * sample.roll_deg
            pitch_filter = args.alpha * (pitch_filter + gy * dt) + (1.0 - args.alpha) * sample.pitch_deg

            should_output = args.format != "none"
            if should_output and output_period_s > 0:
                should_output = now >= next_output_time
                if should_output:
                    while next_output_time <= now:
                        next_output_time += output_period_s
            elif should_output:
                should_output = index % args.output_every == 0

            if should_output:
                timestamp_s = time.time()
                if args.format == "json":
                    row = {
                        "timestamp_s": timestamp_s,
                        "accel_raw": sample.accel_raw,
                        "gyro_raw": sample.gyro_raw,
                        "accel_g": sample.accel_g,
                        "gyro_dps": sample.gyro_dps,
                        "roll_accel_deg": sample.roll_deg,
                        "pitch_accel_deg": sample.pitch_deg,
                        "roll_filter_deg": roll_filter,
                        "pitch_filter_deg": pitch_filter,
                    }
                    print(json.dumps(row, separators=(",", ":")))
                elif args.format == "csv":
                    assert writer is not None
                    writer.writerow(
                        [
                            f"{timestamp_s:.6f}",
                            *sample.accel_raw,
                            *sample.gyro_raw,
                            *(f"{value:.6f}" for value in sample.accel_g),
                            *(f"{value:.6f}" for value in sample.gyro_dps),
                            f"{sample.roll_deg:.3f}",
                            f"{sample.pitch_deg:.3f}",
                            f"{roll_filter:.3f}",
                            f"{pitch_filter:.3f}",
                        ]
                    )
                else:
                    print(
                        "raw_accel=({:6d},{:6d},{:6d}) raw_gyro=({:6d},{:6d},{:6d}) "
                        "accel_g=({:+.3f},{:+.3f},{:+.3f}) gyro_dps=({:+.2f},{:+.2f},{:+.2f}) "
                        "tilt_accel roll={:+7.2f} pitch={:+7.2f} "
                        "tilt_filter roll={:+7.2f} pitch={:+7.2f}".format(
                            *sample.accel_raw,
                            *sample.gyro_raw,
                            *sample.accel_g,
                            *sample.gyro_dps,
                            sample.roll_deg,
                            sample.pitch_deg,
                            roll_filter,
                            pitch_filter,
                        )
                    )
                printed += 1
                if args.flush_every > 0 and printed % args.flush_every == 0:
                    sys.stdout.flush()

            index += 1
            elapsed = time.monotonic() - loop_start
            if elapsed < period_s:
                time.sleep(period_s - elapsed)

        sys.stdout.flush()

    except KeyboardInterrupt:
        return 130
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        if sensor is not None:
            sensor.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
