from icm20948 import ICM20948, open_i2c_bus

bus = open_i2c_bus(1)
imu = ICM20948(bus, address=0x68)

# Read back the config registers we set
accel_cfg = imu._read(2, 0x14, 1)[0]
gyro_cfg = imu._read(2, 0x01, 1)[0]
pwr_mgmt_2 = imu._read(0, 0x07, 1)[0]

print(f"ACCEL_CONFIG (0x14):  0x{accel_cfg:02X} (set to 0x01, read back)")
print(f"GYRO_CONFIG_1 (0x01): 0x{gyro_cfg:02X} (set to 0x19, read back)")
print(f"PWR_MGMT_2 (0x07):    0x{pwr_mgmt_2:02X} (should be 0x00)")

print("\nRaw sensor reads (6 samples):")
for i in range(6):
    ax, ay, az = imu.read_accel_raw()
    gx, gy, gz = imu.read_gyro_raw()
    print(f"  A: {ax:6d} {ay:6d} {az:6d}  |  G: {gx:6d} {gy:6d} {gz:6d}")

bus.close()
