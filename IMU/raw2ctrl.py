"""Read ICM-20948 raw accel/gyro data and estimate roll, pitch, yaw."""

from __future__ import annotations

import argparse
import math
import time

from icm20948 import (
    ACCEL_SCALE,
    ACCEL_CONFIG,
    GYRO_SCALE,
    GYRO_CONFIG_1,
    ICM20948,
    PWR_MGMT_1,
    PWR_MGMT_2,
    USER_CTRL,
    detect_icm20948_addresses,
    open_i2c_bus,
    sleep_ms,
)


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
        "--delay",
        type=float,
        default=0.05,
        help="Seconds between terminal updates.",
    )
    parser.add_argument(
        "--yaw-zero",
        type=float,
        default=0.0,
        help="Initial yaw angle in degrees for gyro integration.",
    )
    parser.add_argument(
        "--alpha",
        type=float,
        default=0.98,
        help="Complementary filter gyro weight for roll/pitch.",
    )
    parser.add_argument(
        "--calibration-samples",
        type=int,
        default=500,
        help="Gyro samples used for stationary bias calibration.",
    )
    parser.add_argument(
        "--calibration-interval",
        type=float,
        default=0.005,
        help="Seconds between gyro bias calibration samples.",
    )
    parser.add_argument(
        "--show-dt",
        action="store_true",
        help="Print loop dt and frequency for timing diagnostics.",
    )
    parser.add_argument(
        "--zero-limit",
        type=int,
        default=3,
        help="Consecutive all-zero raw samples before recovery diagnostics.",
    )
    parser.add_argument(
        "--no-auto-recover",
        action="store_true",
        help="Stop instead of trying to re-enable sensors after all-zero raw data.",
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


def wrap_degrees(angle: float) -> float:
    return (angle + 360.0) % 360.0


def accel_to_roll_pitch(
    ax_g: float,
    ay_g: float,
    az_g: float,
) -> tuple[float, float]:
    roll = math.degrees(math.atan2(ay_g, az_g))
    pitch = math.degrees(math.atan2(-ax_g, math.sqrt(ay_g * ay_g + az_g * az_g)))
    return roll, pitch


def calibrate_gyro_bias(
    imu: ICM20948,
    samples: int,
    interval: float,
) -> tuple[float, float, float]:
    if samples < 10:
        raise RuntimeError("--calibration-samples must be at least 10")

    print(f"Gyro bias calibration: keep IMU still for {samples} samples...")
    sum_x = sum_y = sum_z = 0.0
    zero_samples = 0
    for _ in range(samples):
        gx_raw, gy_raw, gz_raw = imu.read_gyro_raw()
        if gx_raw == 0 and gy_raw == 0 and gz_raw == 0:
            zero_samples += 1
        sum_x += gx_raw * GYRO_SCALE
        sum_y += gy_raw * GYRO_SCALE
        sum_z += gz_raw * GYRO_SCALE
        time.sleep(interval)

    if zero_samples == samples:
        raise RuntimeError(
            "Gyro calibration read all-zero data. IMU is not producing valid gyro output."
        )

    bias = sum_x / samples, sum_y / samples, sum_z / samples
    print(
        f"Gyro bias: X={bias[0]:.4f} Y={bias[1]:.4f} Z={bias[2]:.4f} deg/s\n"
    )
    return bias


def read_status_registers(imu: ICM20948) -> dict[str, int]:
    return {
        "PWR_MGMT_1": imu._read(0, PWR_MGMT_1, 1)[0],
        "PWR_MGMT_2": imu._read(0, PWR_MGMT_2, 1)[0],
        "USER_CTRL": imu._read(0, USER_CTRL, 1)[0],
        "ACCEL_CONFIG": imu._read(2, ACCEL_CONFIG, 1)[0],
        "GYRO_CONFIG_1": imu._read(2, GYRO_CONFIG_1, 1)[0],
    }


def print_status_registers(imu: ICM20948) -> None:
    try:
        registers = read_status_registers(imu)
    except OSError as error:
        print(f"Could not read diagnostic registers: {error}")
        return

    print("Diagnostic registers:")
    for name, value in registers.items():
        print(f"  {name}: 0x{value:02X}")


def reenable_accel_gyro(imu: ICM20948) -> None:
    print("Trying to re-enable accel/gyro without magnetometer...")
    imu._write(0, USER_CTRL, 0x00)
    sleep_ms(10)
    imu._write(0, PWR_MGMT_1, 0x01)
    sleep_ms(100)
    imu._write(0, PWR_MGMT_2, 0x00)
    sleep_ms(50)
    imu._write(2, ACCEL_CONFIG, 0x00)
    sleep_ms(10)
    imu._write(2, GYRO_CONFIG_1, 0x11)
    sleep_ms(10)


def accel_gyro_are_enabled(imu: ICM20948) -> bool:
    registers = read_status_registers(imu)
    sleep_bit_clear = (registers["PWR_MGMT_1"] & 0x40) == 0
    sensors_enabled = registers["PWR_MGMT_2"] == 0x00
    accel_config_ok = registers["ACCEL_CONFIG"] == 0x00
    gyro_config_ok = registers["GYRO_CONFIG_1"] == 0x11
    return sleep_bit_clear and sensors_enabled and accel_config_ok and gyro_config_ok


def raw_accel_gyro_are_valid(imu: ICM20948, samples: int = 5) -> bool:
    for _ in range(samples):
        ax, ay, az = imu.read_accel_raw()
        gx, gy, gz = imu.read_gyro_raw()
        if (ax, ay, az, gx, gy, gz) != (0, 0, 0, 0, 0, 0):
            return True
        sleep_ms(20)
    return False


def main() -> None:
    args = parse_args()
    bus = None
    try:
        bus = open_i2c_bus(args.bus)
        address = resolve_address(bus, args.address)
        imu = ICM20948(bus, address=address, enable_magnetometer=False)

        print(f"Using /dev/i2c-{args.bus}, address 0x{address:02X}")
        who_am_i = imu.read_who_am_i()
        print(f"WHO_AM_I = 0x{who_am_i:02X}")
        if who_am_i != 0xEA:
            raise RuntimeError("Unexpected WHO_AM_I; expected 0xEA.")

        print_status_registers(imu)
        if not accel_gyro_are_enabled(imu):
            raise RuntimeError(
                "IMU init did not enable accel/gyro. "
                "PWR_MGMT_1 sleep bit or config registers are wrong."
            )
        if not raw_accel_gyro_are_valid(imu):
            raise RuntimeError(
                "Raw accel/gyro stayed all zero after init. "
                "Check power, wiring, and whether another process/hardware is disturbing the IMU."
            )

        gx_bias, gy_bias, gz_bias = calibrate_gyro_bias(
            imu,
            args.calibration_samples,
            args.calibration_interval,
        )

        ax_raw, ay_raw, az_raw = imu.read_accel_raw()
        roll, pitch = accel_to_roll_pitch(
            ax_raw * ACCEL_SCALE,
            ay_raw * ACCEL_SCALE,
            az_raw * ACCEL_SCALE,
        )
        yaw = wrap_degrees(args.yaw_zero)

        print("Streaming raw -> unit conversion -> complementary filter.")
        print("Yaw is integrated from gyro Z only; it is relative and will drift.")
        print("Check axis direction/sign before connecting control output.\n")

        dt_header = "       dt      Hz" if args.show_dt else ""
        print(
            f"{'Accel angle (deg)':>24}   "
            f"{'Filtered attitude (deg)':>34}{dt_header}"
        )
        print("-" * (81 if args.show_dt else 64))

        last_time = time.monotonic()
        zero_count = 0
        recovered_once = False

        while True:
            try:
                ax_raw, ay_raw, az_raw = imu.read_accel_raw()
                gx_raw, gy_raw, gz_raw = imu.read_gyro_raw()
            except OSError as error:
                print(f"\nI2C read failed: {error}")
                print(
                    "Stop the script, check wiring/power, then hard power-cycle the IMU/Pi "
                    "if reads keep timing out."
                )
                break

            all_zero = (
                ax_raw == 0
                and ay_raw == 0
                and az_raw == 0
                and gx_raw == 0
                and gy_raw == 0
                and gz_raw == 0
            )
            zero_count = zero_count + 1 if all_zero else 0
            if zero_count >= args.zero_limit:
                print(f"\nRaw accel/gyro are all zero for {zero_count} samples.")
                print_status_registers(imu)

                if args.no_auto_recover or recovered_once:
                    print(
                        "Stopping because all-zero data is not valid for attitude/control. "
                        "Hard power-cycle the IMU/Pi if this repeats."
                    )
                    break

                try:
                    reenable_accel_gyro(imu)
                    print_status_registers(imu)
                    if not accel_gyro_are_enabled(imu):
                        print(
                            "Recovery failed: config writes did not stick. "
                            "Hard power-cycle the IMU/Pi, then check wiring."
                        )
                        break
                except OSError as error:
                    print(f"Recovery write failed: {error}")
                    print("Hard power-cycle the IMU/Pi, then rerun the debug sequence.")
                    break

                recovered_once = True
                zero_count = 0
                last_time = time.monotonic()
                roll = pitch = 0.0
                yaw = wrap_degrees(args.yaw_zero)
                continue

            ax_g = ax_raw * ACCEL_SCALE
            ay_g = ay_raw * ACCEL_SCALE
            az_g = az_raw * ACCEL_SCALE
            gx_dps = gx_raw * GYRO_SCALE - gx_bias
            gy_dps = gy_raw * GYRO_SCALE - gy_bias
            gz_dps = gz_raw * GYRO_SCALE - gz_bias

            now = time.monotonic()
            dt = now - last_time
            last_time = now

            roll_acc, pitch_acc = accel_to_roll_pitch(ax_g, ay_g, az_g)

            roll_gyro = roll + gx_dps * dt
            pitch_gyro = pitch + gy_dps * dt
            roll = args.alpha * roll_gyro + (1.0 - args.alpha) * roll_acc
            pitch = args.alpha * pitch_gyro + (1.0 - args.alpha) * pitch_acc
            yaw = wrap_degrees(yaw + gz_dps * dt)

            dt_text = ""
            if args.show_dt:
                hz = 1.0 / dt if dt > 0.0 else 0.0
                dt_text = f"  {dt:7.4f} {hz:7.1f}"

            print(
                f"Accel: roll={roll_acc:7.2f} deg pitch={pitch_acc:7.2f} deg   "
                f"Filtered: roll={roll:7.2f} deg pitch={pitch:7.2f} deg "
                f"yaw={yaw:7.2f} deg"
                f"{dt_text}"
            )
            time.sleep(args.delay)
    except (RuntimeError, OSError) as error:
        print(error)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if bus is not None:
            bus.close()


if __name__ == "__main__":
    main()
