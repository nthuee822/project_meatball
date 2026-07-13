"""End-to-end sensor check: initializes through the full driver and reads all sensors."""

from icm20948 import ICM20948, detect_icm20948_addresses, open_i2c_bus

SAMPLES = 5

bus = open_i2c_bus(1)
try:
    found = detect_icm20948_addresses(bus)
    if not found:
        print("FAIL: ICM-20948 not found at 0x68 or 0x69")
        raise SystemExit(1)

    addr = found[0]
    print(f"Found ICM-20948 at 0x{addr:02X}")

    imu = ICM20948(bus, address=addr)
    who = imu.read_who_am_i()
    print(f"WHO_AM_I: 0x{who:02X} {'OK' if who == 0xEA else 'UNEXPECTED'}\n")

    print(f"{'#':<3}  {'Accel (g)':^30}  {'Gyro (deg/s)':^30}  {'Temp (C)':>8}")
    print("-" * 80)
    for i in range(SAMPLES):
        ax, ay, az = imu.read_accel()
        gx, gy, gz = imu.read_gyro()
        temp = imu.read_temp()
        print(
            f"{i+1:<3}  "
            f"X={ax:+.3f} Y={ay:+.3f} Z={az:+.3f}  "
            f"X={gx:+.3f} Y={gy:+.3f} Z={gz:+.3f}  "
            f"{temp:>8.2f}"
        )

    mag = imu.read_mag()
    print()
    if mag:
        print(f"Mag (uT): X={mag[0]:.2f}  Y={mag[1]:.2f}  Z={mag[2]:.2f}")
    else:
        print("Mag: no data (magnetometer may need another read cycle)")

    az_vals = [imu.read_accel()[2] for _ in range(SAMPLES)]
    if all(abs(v) < 0.01 for v in az_vals):
        print("\nWARNING: accel Z is near zero — sensor may not be updating")
    else:
        print("\nAll sensors responding OK")
finally:
    bus.close()
