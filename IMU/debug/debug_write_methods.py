from icm20948 import open_i2c_bus, sleep_ms
import smbus2

bus = open_i2c_bus(1)
addr = 0x68

print("Testing different I2C write methods...\n")

# Method 1: smbus2.SMBus.write_byte_data (used in driver)
print("1. Testing write_byte_data(0x06, 0x01)...")
try:
    bus.write_byte_data(addr, 0x06, 0x01)
    sleep_ms(10)
    val = bus.read_byte_data(addr, 0x06)
    print(f"   After: 0x{val:02X}")
except OSError as e:
    print(f"   OSError: {e}")

# Method 2: Try write_i2c_block_data with just 1 byte
print("\n2. Testing write_i2c_block_data(0x06, [0x01])...")
try:
    bus.write_i2c_block_data(addr, 0x06, [0x01])
    sleep_ms(10)
    val = bus.read_byte_data(addr, 0x06)
    print(f"   After: 0x{val:02X}")
except OSError as e:
    print(f"   OSError: {e}")

# Method 3: Try writing 0x00 (neutral)
print("\n3. Testing write_byte_data(0x06, 0x00)...")
try:
    bus.write_byte_data(addr, 0x06, 0x00)
    sleep_ms(10)
    val = bus.read_byte_data(addr, 0x06)
    print(f"   After: 0x{val:02X}")
except OSError as e:
    print(f"   OSError: {e}")

# Method 4: Check if reading multiple times causes errors
print("\n4. Testing repeated reads...")
try:
    for i in range(5):
        val = bus.read_byte_data(addr, 0x06)
        print(f"   Read {i+1}: 0x{val:02X}")
        sleep_ms(10)
except OSError as e:
    print(f"   OSError: {e}")

# Method 5: Try writing via write_word_data
print("\n5. Testing write_word_data(0x06, 0x0001)...")
try:
    bus.write_word_data(addr, 0x06, 0x0001)
    sleep_ms(10)
    val = bus.read_byte_data(addr, 0x06)
    print(f"   After: 0x{val:02X}")
except OSError as e:
    print(f"   OSError: {e}")

bus.close()
