"""Reusable IMU wrapper for ICM-20948 hardware."""

from __future__ import annotations

import math
import statistics
import time

from icm20948 import ICM20948, detect_icm20948_addresses, open_i2c_bus


class IMU:
    def __init__(
        self,
        bus_id: int = 1,
        address: int | None = None,
        warmup_seconds: float = 30.0,
        calibration_samples: int = 1500,
        calibration_interval: float = 0.01,
        calibration_outlier_threshold: float = 1.0,
        enable_magnetometer: bool = True,
    ) -> None:
        self.bus = None
        self.imu: ICM20948 | None = None
        self.gyro_bias = (0.0, 0.0, 0.0)

        try:
            self.bus = open_i2c_bus(bus_id)
            resolved_address = self._resolve_address(address)
            self.imu = ICM20948(
                self.bus,
                address=resolved_address,
                enable_magnetometer=enable_magnetometer,
            )

            if warmup_seconds > 0.0:
                self._warm_up(warmup_seconds)

            self.gyro_bias = self._calibrate_gyro(
                calibration_samples,
                calibration_interval,
                calibration_outlier_threshold,
            )
        except Exception:
            self.close()
            raise

    def close(self) -> None:
        if self.imu is not None:
            self.imu.close()
            self.imu = None
            self.bus = None
            return

        if self.bus is not None:
            close = getattr(self.bus, "close", None)
            if callable(close):
                close()
            self.bus = None

    def read_who_am_i(self) -> int:
        return self._require_imu().read_who_am_i()

    def read_accel(self) -> tuple[float, float, float]:
        return self._require_imu().read_accel()

    def read_gyro_raw(self) -> tuple[float, float, float]:
        return self._require_imu().read_gyro()

    def read_gyro(self) -> tuple[float, float, float]:
        gx, gy, gz = self._require_imu().read_gyro()
        bx, by, bz = self.gyro_bias
        return gx - bx, gy - by, gz - bz

    def read_mag(self) -> tuple[float, float, float] | None:
        return self._require_imu().read_mag()

    def read_temp(self) -> float:
        return self._require_imu().read_temp()

    def _require_imu(self) -> ICM20948:
        if self.imu is None:
            raise RuntimeError("IMU is closed or was not initialized correctly.")
        return self.imu

    def _resolve_address(self, requested_address: int | None) -> int:
        if requested_address is not None:
            return requested_address

        assert self.bus is not None
        found = detect_icm20948_addresses(self.bus)
        if not found:
            raise RuntimeError(
                "ICM-20948 not found at 0x68 or 0x69. Check wiring, power, and CS/AD0 pins."
            )

        print("Detected ICM-20948 address(es):", ", ".join(hex(addr) for addr in found))
        return found[0]

    def _warm_up(self, warmup_seconds: float) -> None:
        print(
            f"Warm-up: collecting/discarding data for {warmup_seconds:.1f}s before calibration..."
        )
        deadline = time.monotonic() + warmup_seconds
        imu = self._require_imu()
        while time.monotonic() < deadline:
            imu.read_gyro()
            time.sleep(0.01)

    @staticmethod
    def _mean_triplet(samples: list[tuple[float, float, float]]) -> tuple[float, float, float]:
        count = len(samples)
        sx = sy = sz = 0.0
        for gx, gy, gz in samples:
            sx += gx
            sy += gy
            sz += gz
        return sx / count, sy / count, sz / count

    @staticmethod
    def _std_triplet(samples: list[tuple[float, float, float]]) -> tuple[float, float, float]:
        xs = [sample[0] for sample in samples]
        ys = [sample[1] for sample in samples]
        zs = [sample[2] for sample in samples]
        return statistics.pstdev(xs), statistics.pstdev(ys), statistics.pstdev(zs)

    def _calibrate_gyro(
        self,
        samples: int,
        interval: float,
        outlier_threshold: float,
    ) -> tuple[float, float, float]:
        if samples < 10:
            raise RuntimeError("calibration_samples must be at least 10")

        print("Gyro calibration: keep the board completely still...")
        imu = self._require_imu()
        raw_samples: list[tuple[float, float, float]] = []
        for _ in range(samples):
            raw_samples.append(imu.read_gyro())
            time.sleep(interval)

        median_x = statistics.median(sample[0] for sample in raw_samples)
        median_y = statistics.median(sample[1] for sample in raw_samples)
        median_z = statistics.median(sample[2] for sample in raw_samples)

        filtered = [
            sample
            for sample in raw_samples
            if math.sqrt(
                (sample[0] - median_x) ** 2
                + (sample[1] - median_y) ** 2
                + (sample[2] - median_z) ** 2
            )
            <= outlier_threshold
        ]

        min_required = max(50, samples // 3)
        if len(filtered) < min_required:
            print(
                "Warning: too many calibration outliers were rejected; using all samples instead."
            )
            filtered = raw_samples

        bias = self._mean_triplet(filtered)
        std = self._std_triplet(filtered)
        print(
            f"Bias: X={bias[0]:.4f}  Y={bias[1]:.4f}  Z={bias[2]:.4f} deg/s "
            f"(accepted {len(filtered)}/{samples})"
        )
        print(
            f"Gyro noise std during calibration: "
            f"X={std[0]:.4f}  Y={std[1]:.4f}  Z={std[2]:.4f} deg/s\n"
        )
        return bias
