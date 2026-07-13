"""ICM-20948 driver for Raspberry Pi / Linux I2C."""

from __future__ import annotations

import importlib
import struct
import time
from typing import Iterable

ICM_ADDR_LOW = 0x68
ICM_ADDR_HIGH = 0x69
DEFAULT_I2C_BUS = 1

# Register map
REG_BANK_SEL = 0x7F

# Bank 0
WHO_AM_I = 0x00
USER_CTRL = 0x03
PWR_MGMT_1 = 0x06
PWR_MGMT_2 = 0x07
ACCEL_XOUT_H = 0x2D
GYRO_XOUT_H = 0x33
TEMP_OUT_H = 0x39
EXT_SLV_SENS_DATA_00 = 0x3B

# Bank 2
GYRO_CONFIG_1 = 0x01
ACCEL_CONFIG = 0x14

# Bank 3
I2C_MST_CTRL = 0x01
I2C_SLV0_ADDR = 0x03
I2C_SLV0_REG = 0x04
I2C_SLV0_CTRL = 0x05
I2C_SLV0_DO = 0x06

# Internal magnetometer (AK09916)
MAG_ADDR = 0x0C
MAG_WIA2 = 0x01
MAG_ST1 = 0x10
MAG_CNTL2 = 0x31
MAG_CNTL3 = 0x32

ACCEL_SCALE = 1.0 / 16384.0
GYRO_SCALE = 1.0 / 131.0
MAG_SCALE = 0.15


def sleep_ms(milliseconds: int) -> None:
    time.sleep(milliseconds / 1000.0)


def open_i2c_bus(bus_id: int = DEFAULT_I2C_BUS):
    for module_name in ("smbus2", "smbus"):
        try:
            module = importlib.import_module(module_name)
            return module.SMBus(bus_id)
        except ImportError:
            continue
    raise RuntimeError(
        "No SMBus backend found. Install `python3-smbus` or `pip install smbus2`."
    )


def detect_icm20948_addresses(
    bus, candidates: Iterable[int] = (ICM_ADDR_LOW, ICM_ADDR_HIGH)
) -> list[int]:
    found = []
    for address in candidates:
        try:
            bus.write_byte_data(address, REG_BANK_SEL, 0x00)
            who_am_i = bus.read_byte_data(address, WHO_AM_I)
        except OSError:
            continue
        if who_am_i == 0xEA:
            found.append(address)
    return found


class ICM20948:
    def __init__(self, bus, address: int = ICM_ADDR_LOW, enable_magnetometer: bool = True):
        self.bus = bus
        self.addr = address
        self._bank = -1
        self._mag_verified = False
        self._mag_ok = False

        self._init_imu()
        if enable_magnetometer:
            try:
                self._init_mag()
                self._mag_ok = True
            except OSError:
                print("Warning: magnetometer not found; accel/gyro still work.")

    def close(self) -> None:
        close = getattr(self.bus, "close", None)
        if callable(close):
            close()

    def _bank_sel(self, bank: int) -> None:
        if self._bank != bank:
            self.bus.write_byte_data(self.addr, REG_BANK_SEL, bank << 4)
            sleep_ms(5)
            self._bank = bank

    def _write(self, bank: int, register: int, value: int) -> None:
        self._bank_sel(bank)
        self.bus.write_byte_data(self.addr, register, value)

    def _read(self, bank: int, register: int, length: int = 1) -> bytes:
        self._bank_sel(bank)
        return bytes(self.bus.read_i2c_block_data(self.addr, register, length))

    def _mag_write(self, register: int, value: int) -> None:
        # Trigger a single AUX write and then disable the slave immediately.
        self._write(3, I2C_SLV0_ADDR, MAG_ADDR)
        self._write(3, I2C_SLV0_REG, register)
        self._write(3, I2C_SLV0_DO, value)
        self._write(3, I2C_SLV0_CTRL, 0x81)
        sleep_ms(10)
        self._write(3, I2C_SLV0_CTRL, 0x00)

    def _mag_read(self, register: int, length: int) -> bytes:
        self._write(3, I2C_SLV0_ADDR, MAG_ADDR | 0x80)
        self._write(3, I2C_SLV0_REG, register)
        self._write(3, I2C_SLV0_CTRL, 0x80 | length)
        sleep_ms(10)
        data = self._read(0, EXT_SLV_SENS_DATA_00, length)
        self._write(3, I2C_SLV0_CTRL, 0x00)
        sleep_ms(20)
        return data

    def _init_imu(self) -> None:
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x80)
        sleep_ms(200)
        self._bank = -1

        who_am_i = self._read(0, WHO_AM_I)[0]
        if who_am_i != 0xEA:
            raise RuntimeError(
                f"ICM-20948 not found at 0x{self.addr:02X}; "
                f"WHO_AM_I=0x{who_am_i:02X} (expected 0xEA)."
            )

        self._write(0, PWR_MGMT_1, 0x01)
        sleep_ms(100)
        self._write(0, PWR_MGMT_2, 0x00)
        sleep_ms(50)
        self._write(2, ACCEL_CONFIG, 0x00)
        sleep_ms(10)
        self._write(2, GYRO_CONFIG_1, 0x11)
        sleep_ms(10)

    def _init_mag(self) -> None:
        self._write(0, USER_CTRL, 0x40)
        self._write(3, I2C_MST_CTRL, 0x07)

        self._mag_write(MAG_CNTL3, 0x01)
        sleep_ms(100)
        self._mag_write(MAG_CNTL2, 0x08)
        sleep_ms(10)

    def read_who_am_i(self) -> int:
        return self._read(0, WHO_AM_I)[0]

    def read_accel_raw(self) -> tuple[int, int, int]:
        return struct.unpack(">hhh", self._read(0, ACCEL_XOUT_H, 6))

    def read_gyro_raw(self) -> tuple[int, int, int]:
        return struct.unpack(">hhh", self._read(0, GYRO_XOUT_H, 6))

    def read_accel(self) -> tuple[float, float, float]:
        ax, ay, az = self.read_accel_raw()
        return ax * ACCEL_SCALE, ay * ACCEL_SCALE, az * ACCEL_SCALE

    def read_gyro(self) -> tuple[float, float, float]:
        gx, gy, gz = self.read_gyro_raw()
        return gx * GYRO_SCALE, gy * GYRO_SCALE, gz * GYRO_SCALE

    def read_temp(self) -> float:
        raw = struct.unpack(">h", self._read(0, TEMP_OUT_H, 2))[0]
        return (raw / 333.87) + 21.0

    def read_mag(self) -> tuple[float, float, float] | None:
        if not self._mag_ok:
            return None

        if not self._mag_verified:
            who_am_i = self._mag_read(MAG_WIA2, 1)[0]
            if who_am_i != 0x09:
                print(
                    f"Warning: AK09916 WHO_AM_I=0x{who_am_i:02X} (expected 0x09)."
                )
            self._mag_verified = True

        data = self._mag_read(MAG_ST1, 9)
        if not (data[0] & 0x01):
            return None

        mx, my, mz = struct.unpack("<hhh", data[1:7])
        if data[8] & 0x08:
            return None

        return mx * MAG_SCALE, my * MAG_SCALE, mz * MAG_SCALE
