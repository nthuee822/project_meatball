"""Interactive 6-position accelerometer calibration for Raspberry Pi 5."""

from __future__ import annotations

import argparse
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
        "--samples",
        type=int,
        default=100,
        help="Samples to average per position.",
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
    return found[0]


def collect(imu: ICM20948, label: str, samples: int) -> tuple[float, float, float]:
    input(f"\n-> {label}\n   Press Enter when ready...")
    print(f"   Collecting {samples} samples...", end="", flush=True)

    sx = sy = sz = 0.0
    for _ in range(samples):
        ax, ay, az = imu.read_accel()
        sx += ax
        sy += ay
        sz += az
        time.sleep(0.01)

    avg = (sx / samples, sy / samples, sz / samples)
    print(f" avg = X:{avg[0]:+.4f}  Y:{avg[1]:+.4f}  Z:{avg[2]:+.4f} g")
    return avg


def main() -> None:
    args = parse_args()
    bus = open_i2c_bus(args.bus)
    try:
        address = resolve_address(bus, args.address)
        imu = ICM20948(bus, address=address, enable_magnetometer=False)

        print("=" * 55)
        print("  Accelerometer 6-position calibration")
        print("=" * 55)
        print("Hold each position still and flat with the Pi powered on.")

        zp = collect(imu, "Z+ UP   (board flat, component side up)", args.samples)
        zn = collect(imu, "Z- UP   (board flat, component side down)", args.samples)
        xp = collect(imu, "X+ UP   (board standing on its LEFT edge)", args.samples)
        xn = collect(imu, "X- UP   (board standing on its RIGHT edge)", args.samples)
        yp = collect(imu, "Y+ UP   (board standing on its BOTTOM edge)", args.samples)
        yn = collect(imu, "Y- UP   (board standing on its TOP edge)", args.samples)

        results = {
            "X": (xp[0], xn[0]),
            "Y": (yp[1], yn[1]),
            "Z": (zp[2], zn[2]),
        }

        print("\n" + "=" * 55)
        print("  Results")
        print("=" * 55)
        print(
            f"  {'Axis':<6} {'+face':>8} {'-face':>8} {'offset':>8} {'scale':>8}  {'scale error':>11}"
        )
        print("  " + "-" * 53)

        offsets = {}
        scales = {}
        for axis, (pos, neg) in results.items():
            offset = (pos + neg) / 2
            scale = (pos - neg) / 2
            err_pct = (scale - 1.0) * 100
            offsets[axis] = offset
            scales[axis] = scale
            flag = "  <- check setup" if abs(scale) < 0.8 else ""
            print(
                f"  {axis:<6} {pos:>+8.4f} {neg:>+8.4f} {offset:>+8.4f} "
                f"{scale:>8.4f}  {err_pct:>+9.2f}%{flag}"
            )

        print("\n  Calibration values to apply:")
        print(
            f"    ACCEL_OFFSET = ({offsets['X']:.5f}, {offsets['Y']:.5f}, {offsets['Z']:.5f})"
        )
        print(
            f"    ACCEL_SCALE  = ({scales['X']:.5f},  {scales['Y']:.5f},  {scales['Z']:.5f})"
        )
        print("\n  Corrected reading formula:")
        print("    ax = (raw_ax - offset_x) / scale_x")
    except RuntimeError as error:
        print(error)
    finally:
        bus.close()


if __name__ == "__main__":
    main()
