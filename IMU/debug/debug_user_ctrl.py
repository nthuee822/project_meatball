from icm20948 import open_i2c_bus, sleep_ms

bus = open_i2c_bus(1)
addr = 0x68
REG_BANK_SEL = 0x7F
USER_CTRL = 0x03
PWR_MGMT_1 = 0x06

print("Checking initialization state...\n")

# Check current PWR_MGMT_1
bus.write_byte_data(addr, REG_BANK_SEL, 0x00)
sleep_ms(5)
pwr_mgmt_1 = bus.read_byte_data(addr, PWR_MGMT_1)
print(f"PWR_MGMT_1: 0x{pwr_mgmt_1:02X}")
print(f"  Bit 7 (DEVICE_RESET): {(pwr_mgmt_1 >> 7) & 1}")
print(f"  Bit 6 (SLEEP): {(pwr_mgmt_1 >> 6) & 1}")
print(f"  Bits 2:0 (CLKSEL): {pwr_mgmt_1 & 0x07}")

# Check USER_CTRL
user_ctrl = bus.read_byte_data(addr, USER_CTRL)
print(f"\nUSER_CTRL: 0x{user_ctrl:02X}")
print(f"  Bit 7 (FIFO_EN): {(user_ctrl >> 7) & 1}")
print(f"  Bit 6 (I2C_MST_EN): {(user_ctrl >> 6) & 1}")
print(f"  Bit 5 (I2C_IF_DIS): {(user_ctrl >> 5) & 1}")
print(f"  Bit 4 (DMP_RST): {(user_ctrl >> 4) & 1}")
print(f"  Bit 3 (FIFO_RST): {(user_ctrl >> 3) & 1}")
print(f"  Bit 1 (I2C_MST_RST): {(user_ctrl >> 1) & 1}")
print(f"  Bit 0 (SIG_COND_RST): {(user_ctrl >> 0) & 1}")

# Try writing to USER_CTRL to disable features that might lock registers
print("\nTrying to write 0x00 to USER_CTRL (disable all)...")
bus.write_byte_data(addr, USER_CTRL, 0x00)
sleep_ms(10)
user_ctrl_after = bus.read_byte_data(addr, USER_CTRL)
print(f"USER_CTRL after write: 0x{user_ctrl_after:02X}")

# Now try writing to GYRO_CONFIG_1 again
print("\nTrying to write to GYRO_CONFIG_1 again after USER_CTRL=0x00...")
bus.write_byte_data(addr, REG_BANK_SEL, 0x20)
sleep_ms(5)
bus.write_byte_data(addr, 0x01, 0x11)
sleep_ms(10)
gyro_cfg_1 = bus.read_byte_data(addr, 0x01)
print(f"GYRO_CONFIG_1 after write: 0x{gyro_cfg_1:02X} (should be 0x11)")

bus.close()
