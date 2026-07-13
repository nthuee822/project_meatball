from icm20948 import open_i2c_bus, sleep_ms

bus = open_i2c_bus(1)
addr = 0x68
REG_BANK_SEL = 0x7F

print("Testing bank switching...\n")

# Test 1: Verify WHO_AM_I in bank 0
bus.write_byte_data(addr, REG_BANK_SEL, 0x00)
sleep_ms(5)
who_am_i = bus.read_byte_data(addr, 0x00)
print(f"Bank 0, WHO_AM_I (0x00): 0x{who_am_i:02X} (should be 0xEA)")

# Test 2: Try switching to bank 2 and verifying we're there
print("\nSwitching to bank 2...")
bus.write_byte_data(addr, REG_BANK_SEL, 0x20)  # 2 << 4
sleep_ms(10)
bank_sel_readback = bus.read_byte_data(addr, REG_BANK_SEL)
print(f"REG_BANK_SEL reads back as: 0x{bank_sel_readback:02X} (should be 0x20)")

# Test 3: Try reading GYRO_CONFIG_1 (0x01) from bank 2
gyro_cfg_1 = bus.read_byte_data(addr, 0x01)
print(f"Bank 2, GYRO_CONFIG_1 (0x01): 0x{gyro_cfg_1:02X}")

# Test 4: Try writing to GYRO_CONFIG_1 and reading back
print("\nTrying to write 0x11 to GYRO_CONFIG_1...")
bus.write_byte_data(addr, 0x01, 0x11)
sleep_ms(10)
gyro_cfg_1_after = bus.read_byte_data(addr, 0x01)
print(f"After write, GYRO_CONFIG_1 (0x01): 0x{gyro_cfg_1_after:02X} (should be 0x11)")

# Test 5: Switch back to bank 0 and verify
print("\nSwitching back to bank 0...")
bus.write_byte_data(addr, REG_BANK_SEL, 0x00)
sleep_ms(5)
who_am_i_after = bus.read_byte_data(addr, 0x00)
print(f"Bank 0, WHO_AM_I (0x00): 0x{who_am_i_after:02X} (should still be 0xEA)")

bus.close()
