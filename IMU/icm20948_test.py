"""Stream ICM-20948 sensor data on Raspberry Pi 5."""

from __future__ import annotations

import argparse
import collections
import math
import statistics
import time

from icm20948 import ICM20948, detect_icm20948_addresses, open_i2c_bus

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bus", type=int, default=1, help="Linux I2C bus number.")
    parser.add_argument(
        "--address",
        type=lambda value: int(value, 0),
        default=None,
        help="ICM-20948 I2C address (default: auto-detect 0x68/0x69).",
    )
    parser.add_argument(
        "--calibration-samples",
        type=int,
        default=1500,
        help="Samples to average for gyro bias calibration.",
    )
    parser.add_argument(
        "--warmup-seconds",
        type=float,
        default=30.0,
        help="Seconds to let IMU thermally settle before calibration.",
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
        "--noise-window",
        type=int,
        default=50,
        help="Rolling sample window for reported gyro noise standard deviation.",
    )
    parser.add_argument(
        "--gyro-average-window",
        type=int,
        default=15,
        help="Rolling sample window for displayed gyro moving average.",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.2,
        help="Seconds between streamed samples.",
    )
    return parser.parse_args()


def resolve_address(bus, requested_address: int | None) -> int:
    if requested_address is not None:
        return requested_address

    found = detect_icm20948_addresses(bus)
    if not found:
        raise RuntimeError(
            "ICM-20948 not found at 0x68 or 0x69. Check wiring, power, and CS/AD0 pins."
        )

    print("Detected ICM-20948 address(es):", ", ".join(hex(addr) for addr in found))
    return found[0]


def _mean_triplet(samples: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    count = len(samples)
    sx = sy = sz = 0.0
    for gx, gy, gz in samples:
        sx += gx
        sy += gy
        sz += gz
    return sx / count, sy / count, sz / count


def _std_triplet(samples: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    xs = [sample[0] for sample in samples]
    ys = [sample[1] for sample in samples]
    zs = [sample[2] for sample in samples]
    return statistics.pstdev(xs), statistics.pstdev(ys), statistics.pstdev(zs)


def calibrate_gyro(
    imu: ICM20948,
    samples: int,
    interval: float,
    outlier_threshold: float,
) -> tuple[float, float, float]:
    if samples < 10:
        raise RuntimeError("--calibration-samples must be at least 10")

    print("Gyro calibration: keep the board completely still...")
    raw_samples: list[tuple[float, float, float]] = []
    for _ in range(samples):
        raw_samples.append(imu.read_gyro())
        time.sleep(interval)

    median_x = statistics.median(sample[0] for sample in raw_samples)
    median_y = statistics.median(sample[1] for sample in raw_samples)
    median_z = statistics.median(sample[2] for sample in raw_samples)

    filtered = [
        sample
        for sample in raw_samples
        if math.sqrt(
            (sample[0] - median_x) ** 2
            + (sample[1] - median_y) ** 2
            + (sample[2] - median_z) ** 2
        )
        <= outlier_threshold
    ]

    min_required = max(50, samples // 3)
    if len(filtered) < min_required:
        print(
            "Warning: too many calibration outliers were rejected; "
            "using all samples instead."
        )
        filtered = raw_samples

    bias = _mean_triplet(filtered)
    std = _std_triplet(filtered)
    print(
        f"Bias: X={bias[0]:.4f}  Y={bias[1]:.4f}  Z={bias[2]:.4f} deg/s "
        f"(accepted {len(filtered)}/{samples})"
    )
    print(
        f"Gyro noise std during calibration: "
        f"X={std[0]:.4f}  Y={std[1]:.4f}  Z={std[2]:.4f} deg/s\n"
    )
    return bias


def main() -> None:
    args = parse_args()
    bus = None
    try:
        bus = open_i2c_bus(args.bus)
        address = resolve_address(bus, args.address)
        print(f"Using /dev/i2c-{args.bus}, address 0x{address:02X}")
        imu = ICM20948(bus, address=address)

        print(f"ICM-20948 detected (WHO_AM_I = 0x{imu.read_who_am_i():02X})")

        if args.warmup_seconds > 0.0:
            print(
                f"Warm-up: collecting/discarding data for "
                f"{args.warmup_seconds:.1f}s before calibration..."
            )
            warmup_deadline = time.monotonic() + args.warmup_seconds
            while time.monotonic() < warmup_deadline:
                imu.read_gyro()
                time.sleep(0.01)

        gyro_bias = calibrate_gyro(
            imu,
            args.calibration_samples,
            args.calibration_interval,
            args.calibration_outlier_threshold,
        )

        noise_window = max(2, args.noise_window)
        avg_window = max(1, args.gyro_average_window)
        gx_noise_hist: collections.deque[float] = collections.deque(maxlen=noise_window)
        gy_noise_hist: collections.deque[float] = collections.deque(maxlen=noise_window)
        gz_noise_hist: collections.deque[float] = collections.deque(maxlen=noise_window)
        gx_avg_hist: collections.deque[float] = collections.deque(maxlen=avg_window)
        gy_avg_hist: collections.deque[float] = collections.deque(maxlen=avg_window)
        gz_avg_hist: collections.deque[float] = collections.deque(maxlen=avg_window)

        print("Streaming data; press Ctrl-C to stop.\n")
        print(
            f"{'Accel (g)':>32}   {'Gyro avg (deg/s)':>32}   "
            f"{'Gyro sigma (deg/s)':>32}   {'Mag (uT)':>32}   Temp"
        )
        print("-" * 154)

        while True:
            ax, ay, az = imu.read_accel()
            gx, gy, gz = imu.read_gyro()
            mag = imu.read_mag()
            temp = imu.read_temp()

            gx -= gyro_bias[0]
            gy -= gyro_bias[1]
            gz -= gyro_bias[2]

            gx_noise_hist.append(gx)
            gy_noise_hist.append(gy)
            gz_noise_hist.append(gz)
            gx_avg_hist.append(gx)
            gy_avg_hist.append(gy)
            gz_avg_hist.append(gz)

            gx_avg = sum(gx_avg_hist) / len(gx_avg_hist)
            gy_avg = sum(gy_avg_hist) / len(gy_avg_hist)
            gz_avg = sum(gz_avg_hist) / len(gz_avg_hist)

            if len(gx_noise_hist) > 1:
                gx_sigma = statistics.pstdev(gx_noise_hist)
                gy_sigma = statistics.pstdev(gy_noise_hist)
                gz_sigma = statistics.pstdev(gz_noise_hist)
            else:
                gx_sigma = gy_sigma = gz_sigma = 0.0

            mag_str = (
                f"X={mag[0]:7.2f} Y={mag[1]:7.2f} Z={mag[2]:7.2f}"
                if mag
                else "       (no data yet)       "
            )

            print(
                f"A: X={ax:6.3f} Y={ay:6.3f} Z={az:6.3f} g  |  "
                f"Gavg: X={gx_avg:7.3f} Y={gy_avg:7.3f} Z={gz_avg:7.3f} deg/s  |  "
                f"Gsigma: X={gx_sigma:6.3f} Y={gy_sigma:6.3f} Z={gz_sigma:6.3f}  |  "
                f"M: {mag_str} uT  |  "
                f"{temp:.1f} C"
            )
            time.sleep(args.delay)
    except RuntimeError as error:
        print(error)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if bus is not None:
            bus.close()


if __name__ == "__main__":
    main()
