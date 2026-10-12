#!/usr/bin/env python3
"""DMP diagnostic: check registers after init, then watch FIFO for a few seconds."""

import sys, os, time, struct
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from IMU_DMP.dmp_imu import DMPIMU, PACKET_SIZE

# Register addresses (Bank 0)
REG_USER_CTRL    = 0x03
REG_PWR_MGMT_1   = 0x06
REG_INT_STATUS   = 0x19
REG_INT_STATUS_1 = 0x1A
REG_INT_STATUS_2 = 0x1B
REG_INT_STATUS_3 = 0x1C
REG_FIFO_COUNT_H = 0x70
REG_FIFO_COUNT_L = 0x71
REG_FIFO_R_W     = 0x72
PRGM_STRT_ADDRH  = 0x50   # Bank 2
PRGM_STRT_ADDRL  = 0x51

d = DMPIMU()

# Run init without starting the reader thread
print("[diag] Running DMP init ...")
if not d.init():
    print("[diag] init() failed — check I2C wiring")
    sys.exit(1)
imu = d._imu

print("[diag] Init complete. Checking registers ...")

with imu._io_lock:
    imu._bank_sel(0)
    pwr   = imu.bus.read_byte_data(imu.addr, REG_PWR_MGMT_1)
    uctrl = imu.bus.read_byte_data(imu.addr, REG_USER_CTRL)
    is0   = imu.bus.read_byte_data(imu.addr, REG_INT_STATUS)
    is1   = imu.bus.read_byte_data(imu.addr, REG_INT_STATUS_1)
    is2   = imu.bus.read_byte_data(imu.addr, REG_INT_STATUS_2)
    is3   = imu.bus.read_byte_data(imu.addr, REG_INT_STATUS_3)
    cnt_h = imu.bus.read_byte_data(imu.addr, REG_FIFO_COUNT_H)
    cnt_l = imu.bus.read_byte_data(imu.addr, REG_FIFO_COUNT_L)
    imu._bank_sel(2)  # PRGM_STRT_ADDR lives in Bank 2
    psh   = imu.bus.read_byte_data(imu.addr, PRGM_STRT_ADDRH)
    psl   = imu.bus.read_byte_data(imu.addr, PRGM_STRT_ADDRL)
    imu._bank_sel(0)

print(f"  PWR_MGMT_1    = 0x{pwr:02X}  (expect 0x01 = wake + PLL)")
print(f"  USER_CTRL     = 0x{uctrl:02X}  (expect 0xC0 after DMP_RST pulse; DMP_EN bit7, FIFO_EN bit6)")
print(f"  PRGM_STRT     = 0x{psh:02X}{psl:02X}  (expect 0x1000)")
print(f"  INT_STATUS    = 0x{is0:02X}")
print(f"  INT_STATUS_1  = 0x{is1:02X}  (DMP interrupt: bit1)")
print(f"  INT_STATUS_2  = 0x{is2:02X}  (FIFO overflow: bit0)")
print(f"  INT_STATUS_3  = 0x{is3:02X}")
print(f"  FIFO_COUNT    = {(cnt_h << 8) | cnt_l} bytes (packet = {PACKET_SIZE} bytes)")

print()
print("[diag] Polling FIFO for 5 seconds ...")
t0 = time.monotonic()
last_print = t0
count_reads = 0

while time.monotonic() - t0 < 5.0:
    with imu._io_lock:
        imu._bank_sel(0)
        cnt_h = imu.bus.read_byte_data(imu.addr, REG_FIFO_COUNT_H)
        cnt_l = imu.bus.read_byte_data(imu.addr, REG_FIFO_COUNT_L)
        is1   = imu.bus.read_byte_data(imu.addr, REG_INT_STATUS_1)
        uctrl = imu.bus.read_byte_data(imu.addr, REG_USER_CTRL)
    count = (cnt_h << 8) | cnt_l
    count_reads += 1

    now = time.monotonic()
    if now - last_print >= 0.5:
        print(f"  t={now-t0:.1f}s  FIFO={count:4d}B  INT_STATUS_1=0x{is1:02X}  USER_CTRL=0x{uctrl:02X}")
        last_print = now

    time.sleep(0.05)

print(f"[diag] Done ({count_reads} polls).")
