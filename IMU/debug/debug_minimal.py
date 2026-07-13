from icm20948 import open_i2c_bus, sleep_ms

bus = open_i2c_bus(1)
addr = 0x68
REG_BANK_SEL = 0x7F
PWR_MGMT_1 = 0x06
PWR_MGMT_2 = 0x07
ACCEL_XOUT_H = 0x2D

print("Minimal sensor test - raw I2C without driver\n")

# 1. Soft reset
print("1. Soft reset (PWR_MGMT_1 = 0x80)...")
bus.write_byte_data(addr, REG_BANK_SEL, 0x00)
sleep_ms(5)
bus.write_byte_data(addr, PWR_MGMT_1, 0x80)
sleep_ms(200)

# 2. Wake up - set to PLL (0x01)
print("2. Wake up (PWR_MGMT_1 = 0x01)...")
bus.write_byte_data(addr, PWR_MGMT_1, 0x01)
sleep_ms(100)
pwr = bus.read_byte_data(addr, PWR_MGMT_1)
print(f"   PWR_MGMT_1 reads: 0x{pwr:02X}")

# 3. Enable all sensors
print("3. Enable all sensors (PWR_MGMT_2 = 0x00)...")
bus.write_byte_data(addr, PWR_MGMT_2, 0x00)
sleep_ms(50)
pwr2 = bus.read_byte_data(addr, PWR_MGMT_2)
print(f"   PWR_MGMT_2 reads: 0x{pwr2:02X}")

# 4. Read accel data multiple times - should change if sensor is working
print("\n4. Reading ACCEL_XOUT_H (first 2 bytes of accel) 10 times:")
print("   (should be different values if sensor is capturing data)")
for i in range(10):
    data = bus.read_i2c_block_data(addr, ACCEL_XOUT_H, 2)
    value = (data[0] << 8) | data[1]
    print(f"   Read {i+1}: {data[0]:3d}, {data[1]:3d} -> raw value: {value:6d}")
    sleep_ms(50)

print("\n5. Checking if any data is changing...")
values = []
for i in range(5):
    data = bus.read_i2c_block_data(addr, ACCEL_XOUT_H, 2)
    value = (data[0] << 8) | data[1]
    values.append(value)
    sleep_ms(100)

unique_values = set(values)
print(f"   Collected {len(values)} reads, {len(unique_values)} unique values")
print(f"   Values: {values}")
if len(unique_values) == 1:
    print("   WARNING: All values are identical - sensor may not be updating!")

bus.close()
