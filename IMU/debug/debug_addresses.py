from icm20948 import open_i2c_bus, sleep_ms

bus = open_i2c_bus(1)
WHO_AM_I = 0x00
PWR_MGMT_1 = 0x06

for addr in [0x68, 0x69]:
    print(f"\n=== Testing address 0x{addr:02X} ===")
    
    # Check WHO_AM_I
    try:
        who = bus.read_byte_data(addr, WHO_AM_I)
        print(f"WHO_AM_I: 0x{who:02X}", end="")
        if who == 0xEA:
            print(" ✓ (ICM-20948 found)")
        else:
            print(f" (not ICM-20948)")
    except Exception as e:
        print(f"WHO_AM_I read failed: {e}")
        continue
    
    # Try a write/read cycle
    print(f"Testing PWR_MGMT_1 write...")
    
    # Read initial
    initial = bus.read_byte_data(addr, PWR_MGMT_1)
    print(f"  Initial value: 0x{initial:02X}")
    
    # Write 0xAA (a distinctive pattern)
    bus.write_byte_data(addr, PWR_MGMT_1, 0xAA)
    sleep_ms(10)
    after_write = bus.read_byte_data(addr, PWR_MGMT_1)
    print(f"  After write 0xAA: 0x{after_write:02X}", end="")
    
    if after_write == 0xAA:
        print(" ✓ (write worked!)")
    else:
        print(f" ✗ (write ignored or masked)")
    
    # Try reset
    print(f"Testing DEVICE_RESET...")
    bus.write_byte_data(addr, PWR_MGMT_1, 0x80)
    sleep_ms(200)
    after_reset = bus.read_byte_data(addr, PWR_MGMT_1)
    print(f"  After reset: 0x{after_reset:02X}")

bus.close()
