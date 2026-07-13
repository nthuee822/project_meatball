#!/usr/bin/env python3
"""Minimal I2C driver for reading ICM-20948 raw accel/gyro on Raspberry Pi."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Iterable

try:
    from smbus2 import SMBus
except ImportError as exc:  # pragma: no cover - depends on target system
    raise SystemExit(
        "Missing dependency: smbus2. Install with: python3 -m pip install smbus2"
    ) from exc


WHO_AM_I_VALUE = 0xEA
DEFAULT_ADDRESSES = (0x68, 0x69)

REG_BANK_SEL = 0x7F

BANK0 = 0
REG_WHO_AM_I = 0x00
REG_USER_CTRL = 0x03
REG_LP_CONFIG = 0x05
REG_PWR_MGMT_1 = 0x06
REG_PWR_MGMT_2 = 0x07
REG_ACCEL_XOUT_H = 0x2D
REG_GYRO_XOUT_H = 0x33

BANK2 = 2
REG_GYRO_SMPLRT_DIV = 0x00
REG_GYRO_CONFIG_1 = 0x01
REG_ACCEL_SMPLRT_DIV_1 = 0x10
REG_ACCEL_SMPLRT_DIV_2 = 0x11
REG_ACCEL_CONFIG = 0x14


ACCEL_SCALE_FACTORS = {
    2: 16384.0,
    4: 8192.0,
    8: 4096.0,
    16: 2048.0,
}

GYRO_SCALE_FACTORS = {
    250: 131.0,
    500: 65.5,
    1000: 32.8,
    2000: 16.4,
}


@dataclass(frozen=True)
class ImuSample:
    accel_raw: tuple[int, int, int]
    gyro_raw: tuple[int, int, int]
    accel_g: tuple[float, float, float]
    gyro_dps: tuple[float, float, float]
    roll_deg: float
    pitch_deg: float


class ICM20948:
    """ICM-20948 raw accel/gyro reader using Linux I2C."""

    def __init__(
        self,
        bus: int = 1,
        address: int | None = None,
        accel_range_g: int = 4,
        gyro_range_dps: int = 2000,
        sample_rate_hz: int = 100,
        dlpf_cfg: int = 3,
        read_mode: str = "byte",
        retries: int = 3,
    ) -> None:
        if accel_range_g not in ACCEL_SCALE_FACTORS:
            raise ValueError(f"Unsupported accel range: +/-{accel_range_g}g")
        if gyro_range_dps not in GYRO_SCALE_FACTORS:
            raise ValueError(f"Unsupported gyro range: +/-{gyro_range_dps}dps")
        if sample_rate_hz <= 0:
            raise ValueError("sample_rate_hz must be positive")
        if read_mode not in ("byte", "block", "combined"):
            raise ValueError("read_mode must be byte, block, or combined")
        if retries < 0:
            raise ValueError("retries cannot be negative")

        self.bus_num = bus
        try:
            self.bus = SMBus(bus)
        except FileNotFoundError as exc:
            raise RuntimeError(
                f"I2C bus /dev/i2c-{bus} was not found. Enable I2C with "
                "raspi-config, then reboot if the device node still does not exist."
            ) from exc
        except PermissionError as exc:
            raise RuntimeError(
                f"Permission denied opening /dev/i2c-{bus}. Run with sudo or add "
                "your user to the i2c group."
            ) from exc
        self.address = address or self._detect_address(DEFAULT_ADDRESSES)
        self.accel_range_g = accel_range_g
        self.gyro_range_dps = gyro_range_dps
        self.accel_lsb_per_g = ACCEL_SCALE_FACTORS[accel_range_g]
        self.gyro_lsb_per_dps = GYRO_SCALE_FACTORS[gyro_range_dps]
        self.sample_rate_hz = sample_rate_hz
        self.dlpf_cfg = dlpf_cfg & 0x07
        self.read_mode = read_mode
        self.retries = retries
        self._bank = None

    def close(self) -> None:
        self.bus.close()

    def initialize(self) -> None:
        whoami = self.who_am_i()
        if whoami != WHO_AM_I_VALUE:
            raise RuntimeError(
                f"Unexpected WHO_AM_I 0x{whoami:02X} at 0x{self.address:02X}; "
                f"expected 0x{WHO_AM_I_VALUE:02X}"
            )

        self._select_bank(BANK0)
        self._write(REG_PWR_MGMT_1, 0x80)
        time.sleep(0.1)

        self._select_bank(BANK0)
        self._write(REG_PWR_MGMT_1, 0x01)  # Auto-select best available clock.
        time.sleep(0.01)
        self._write(REG_PWR_MGMT_2, 0x00)  # Enable accel and gyro axes.
        self._write(REG_LP_CONFIG, 0x00)
        self._write(REG_USER_CTRL, 0x00)

        self._configure_accel()
        self._configure_gyro()
        self._select_bank(BANK0)

    def who_am_i(self) -> int:
        self._select_bank(BANK0)
        return self._read(REG_WHO_AM_I)

    def read_sample(self) -> ImuSample:
        self._select_bank(BANK0)
        try:
            accel_raw, gyro_raw = self._read_raw_axes_with_retries()
        except OSError as exc:
            raise RuntimeError(
                f"I2C read failed at address 0x{self.address:02X}: {exc}. "
                "Try --read-mode byte, lower --rate, or check wiring, pull-ups, "
                "I2C speed, and sensor power."
            ) from exc

        accel_g = tuple(v / self.accel_lsb_per_g for v in accel_raw)
        gyro_dps = tuple(v / self.gyro_lsb_per_dps for v in gyro_raw)
        roll, pitch = self.accel_roll_pitch(accel_g)

        return ImuSample(
            accel_raw=accel_raw,
            gyro_raw=gyro_raw,
            accel_g=accel_g,
            gyro_dps=gyro_dps,
            roll_deg=roll,
            pitch_deg=pitch,
        )

    @staticmethod
    def accel_roll_pitch(accel_g: Iterable[float]) -> tuple[float, float]:
        ax, ay, az = accel_g
        roll = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        return roll, pitch

    def _detect_address(self, addresses: Iterable[int]) -> int:
        last_error: OSError | None = None
        for addr in addresses:
            try:
                self.address = addr
                self._bank = None
                self._select_bank(BANK0)
                if self._read(REG_WHO_AM_I) == WHO_AM_I_VALUE:
                    return addr
            except OSError as exc:
                last_error = exc
        if last_error:
            raise RuntimeError(
                "Could not find ICM-20948 on I2C addresses 0x68 or 0x69"
            ) from last_error
        raise RuntimeError("Could not find ICM-20948 on I2C addresses 0x68 or 0x69")

    def _configure_accel(self) -> None:
        fs_sel = {2: 0, 4: 1, 8: 2, 16: 3}[self.accel_range_g]
        divider = max(0, int(1125 / self.sample_rate_hz) - 1)
        divider = min(divider, 0x0FFF)

        self._select_bank(BANK2)
        self._write(REG_ACCEL_SMPLRT_DIV_1, (divider >> 8) & 0x0F)
        self._write(REG_ACCEL_SMPLRT_DIV_2, divider & 0xFF)
        self._write(REG_ACCEL_CONFIG, (self.dlpf_cfg << 3) | (fs_sel << 1) | 0x01)

    def _configure_gyro(self) -> None:
        fs_sel = {250: 0, 500: 1, 1000: 2, 2000: 3}[self.gyro_range_dps]
        divider = max(0, int(1100 / self.sample_rate_hz) - 1)
        divider = min(divider, 0xFF)

        self._select_bank(BANK2)
        self._write(REG_GYRO_SMPLRT_DIV, divider)
        self._write(REG_GYRO_CONFIG_1, (self.dlpf_cfg << 3) | (fs_sel << 1) | 0x01)

    def _select_bank(self, bank: int) -> None:
        if self._bank == bank:
            return
        self.bus.write_byte_data(self.address, REG_BANK_SEL, (bank & 0x03) << 4)
        self._bank = bank

    def _read(self, register: int) -> int:
        return self.bus.read_byte_data(self.address, register)

    def _write(self, register: int, value: int) -> None:
        self.bus.write_byte_data(self.address, register, value & 0xFF)

    def _read_i16_xyz(self, start_register: int) -> tuple[int, int, int]:
        data = self.bus.read_i2c_block_data(self.address, start_register, 6)
        return self._i16_xyz_from_block(data, 0)

    def _read_raw_axes_with_retries(self) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
        attempts = self.retries + 1
        last_error: OSError | None = None
        for attempt in range(attempts):
            try:
                return self._read_raw_axes_once()
            except OSError as exc:
                last_error = exc
                self._bank = None
                if attempt + 1 < attempts:
                    time.sleep(0.001)
                    self._select_bank(BANK0)
        assert last_error is not None
        raise last_error

    def _read_raw_axes_once(self) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
        if self.read_mode == "combined":
            data = self.bus.read_i2c_block_data(self.address, REG_ACCEL_XOUT_H, 12)
            return self._i16_xyz_from_block(data, 0), self._i16_xyz_from_block(data, 6)
        if self.read_mode == "block":
            return self._read_i16_xyz(REG_ACCEL_XOUT_H), self._read_i16_xyz(REG_GYRO_XOUT_H)
        return self._read_i16_xyz_bytes(REG_ACCEL_XOUT_H), self._read_i16_xyz_bytes(REG_GYRO_XOUT_H)

    def _read_i16_xyz_bytes(self, start_register: int) -> tuple[int, int, int]:
        data = [self._read(start_register + offset) for offset in range(6)]
        return self._i16_xyz_from_block(data, 0)

    def _i16_xyz_from_block(self, data: list[int], offset: int) -> tuple[int, int, int]:
        return (
            self._to_i16(data[offset], data[offset + 1]),
            self._to_i16(data[offset + 2], data[offset + 3]),
            self._to_i16(data[offset + 4], data[offset + 5]),
        )

    @staticmethod
    def _to_i16(msb: int, lsb: int) -> int:
        value = (msb << 8) | lsb
        return value - 0x10000 if value & 0x8000 else value
