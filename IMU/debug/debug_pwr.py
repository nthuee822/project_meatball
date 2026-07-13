from icm20948 import open_i2c_bus, sleep_ms

bus = open_i2c_bus(1)
addr = 0x68
PWR_MGMT_1 = 0x06

print("Testing PWR_MGMT_1 writes to clear SLEEP...\n")

# Read initial state
pwr = bus.read_byte_data(addr, PWR_MGMT_1)
print(f"Initial PWR_MGMT_1: 0x{pwr:02X} (SLEEP bit = {(pwr >> 6) & 1})")

# Try writing 0x01 (clear SLEEP, use PLL)
print("\nWriting 0x01 to PWR_MGMT_1...")
bus.write_byte_data(addr, PWR_MGMT_1, 0x01)
sleep_ms(10)
pwr = bus.read_byte_data(addr, PWR_MGMT_1)
print(f"After first write: 0x{pwr:02X} (SLEEP bit = {(pwr >> 6) & 1})")

# Try again with longer wait
sleep_ms(100)
pwr = bus.read_byte_data(addr, PWR_MGMT_1)
print(f"After 100ms wait: 0x{pwr:02X} (SLEEP bit = {(pwr >> 6) & 1})")

# Try multiple writes
print("\nTrying 5 consecutive writes...")
for i in range(5):
    bus.write_byte_data(addr, PWR_MGMT_1, 0x01)
    sleep_ms(20)
    pwr = bus.read_byte_data(addr, PWR_MGMT_1)
    print(f"  Write {i+1}: 0x{pwr:02X} (SLEEP bit = {(pwr >> 6) & 1})")

# Try disabling CLKSEL too (0x00)
print("\nTrying PWR_MGMT_1 = 0x00 (disable PLL)...")
bus.write_byte_data(addr, PWR_MGMT_1, 0x00)
sleep_ms(100)
pwr = bus.read_byte_data(addr, PWR_MGMT_1)
print(f"After write: 0x{pwr:02X} (SLEEP bit = {(pwr >> 6) & 1})")

# Try soft reset (bit 7)
print("\nTrying PWR_MGMT_1 = 0x80 (DEVICE_RESET)...")
bus.write_byte_data(addr, PWR_MGMT_1, 0x80)
sleep_ms(200)
pwr = bus.read_byte_data(addr, PWR_MGMT_1)
print(f"After reset: 0x{pwr:02X} (should auto-clear)")

# Then try waking up
print("\nAfter reset, writing 0x01 to wake up...")
bus.write_byte_data(addr, PWR_MGMT_1, 0x01)
sleep_ms(100)
pwr = bus.read_byte_data(addr, PWR_MGMT_1)
print(f"Final state: 0x{pwr:02X} (SLEEP bit = {(pwr >> 6) & 1})")

bus.close()
