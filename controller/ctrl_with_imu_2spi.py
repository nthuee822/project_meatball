"""三輪全向輪機器人主程式 - SPI + IMU 整合版本。"""

from __future__ import annotations

import argparse
import glob
import math
import os
import statistics
import sys
import time
from dataclasses import dataclass

try:
    import spidev
    import RPi.GPIO as GPIO
except ImportError:
    print("警告: 缺少 spidev 或 RPi.GPIO，請確保在 Raspberry Pi 上執行")
    sys.exit(1)

from omni_robot import OmniRobotController, MOTOR_MAX_RPS
from IMU_DMP.icm20948 import ICM20948, detect_icm20948_addresses, open_i2c_bus

TRIGGER_DEADZONE = 0.1
BUTTON_A = 0
BUTTON_X = 2
LONG_PRESS_DURATION = 1.5

CS_PINS = {
    "CS0": 17,
    "CS1": 27,
    "CS2": 22,
    "CS3": 5,
}

DEFAULT_IMU_CALIB = "IMU_DMP/imu_calibration.json"


@dataclass
class LoopStats:
    hz: float = 0.0
    jitter_ms: float = 0.0
    worst_jitter_ms: float = 0.0
    overrun_count: int = 0


@dataclass
class IMUState:
    accel: tuple[float, float, float] = (0.0, 0.0, 0.0)
    gyro: tuple[float, float, float] = (0.0, 0.0, 0.0)
    mag: tuple[float, float, float] | None = None
    temp_c: float = 0.0


def apply_trigger_deadzone(trigger_value: float, deadzone: float = TRIGGER_DEADZONE) -> float:
    normalized = (trigger_value + 1.0) / 2.0
    if normalized < deadzone:
        return -1.0
    return (normalized - deadzone) / (1.0 - deadzone) * 2.0 - 1.0


class OmniRobotIMUSPIController(OmniRobotController):
    def __init__(
        self,
        motor_max_rps: float,
        loop_hz: float,
        imu_bus: int | None,
        imu_auto_scan: bool,
        imu_addr: int | None,
        enable_mag: bool,
        mag_rate_hz: float,
        imu_calib_file: str | None,
        gyro_calib_samples: int,
        gyro_calib_interval: float,
        gyro_calib_outlier_threshold: float,
        verbose: bool = False,
    ) -> None:
        super().__init__(motor_max_rps=motor_max_rps)

        self.verbose = verbose
        self.braking = False
        self.move_angle_deg = 0.0
        self.x_pressed = False
        self.x_press_start_time = 0.0

        self.loop_hz = max(1.0, loop_hz)
        self.target_period_ns = int(1_000_000_000 / self.loop_hz)

        self.loop_stats = LoopStats()
        self._last_loop_start_ns: int | None = None

        self.imu_state = IMUState()
        self.enable_mag = enable_mag
        self.mag_rate_hz = max(0.0, mag_rate_hz)
        self._next_mag_read_ns = 0
        self.imu_available = False
        self.imu = None
        self.imu_bus = None
        self.imu_status = "未初始化"
        self.imu_error: str | None = None

        self._init_gpio_spi()
        self._init_imu(imu_bus, imu_addr, enable_mag, imu_auto_scan)

        if imu_calib_file and self.imu_available:
            self._load_imu_calibration_if_exists(imu_calib_file)

        if gyro_calib_samples > 0 and self.imu_available:
            bias = self._calibrate_gyro_bias(
                samples=gyro_calib_samples,
                interval_sec=gyro_calib_interval,
                outlier_threshold=gyro_calib_outlier_threshold,
            )
            self.imu.set_gyro_bias(bias)

    def _init_gpio_spi(self) -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for name, pin in CS_PINS.items():
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH)
            if self.verbose:
                print(f"Initialized {name} on GPIO {pin}")

        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 500000
        self.spi.mode = 0

    def _resolve_imu_address(self, bus, requested: int | None) -> int:
        if requested is not None:
            return requested

        found = detect_icm20948_addresses(bus)
        if not found:
            raise RuntimeError("找不到 ICM-20948 (0x68/0x69)")

        print("[IMU] Detected address(es):", ", ".join(hex(addr) for addr in found))
        return found[0]

    def _available_i2c_buses(self) -> list[int]:
        buses: list[int] = []
        for dev in sorted(glob.glob("/dev/i2c-*")):
            suffix = dev.rsplit("-", 1)[-1]
            if suffix.isdigit():
                buses.append(int(suffix))
        return buses

    def _candidate_i2c_buses(self, requested: int | None, auto_scan: bool) -> list[int]:
        available = self._available_i2c_buses()
        if requested is None:
            requested = 1

        if not auto_scan:
            return [requested]

        candidates = [requested]
        for bus_id in available:
            if bus_id != requested:
                candidates.append(bus_id)
        return candidates

    def _init_imu(
        self,
        imu_bus: int | None,
        imu_addr: int | None,
        enable_mag: bool,
        imu_auto_scan: bool,
    ) -> None:
        candidates = self._candidate_i2c_buses(imu_bus, imu_auto_scan)
        if not candidates:
            self.imu_available = False
            self.imu_status = "offline"
            self.imu_error = "找不到任何 /dev/i2c-*"
            print("[IMU] 找不到任何 I2C bus，將以無 IMU 模式繼續執行")
            return

        if self.verbose:
            scan_mode = "auto-scan" if imu_auto_scan else "fixed-bus"
            print(
                f"[IMU] Candidate bus ({scan_mode}):",
                ", ".join(f"/dev/i2c-{bus_id}" for bus_id in candidates),
            )

        last_error: str | None = None
        for bus_id in candidates:
            bus_handle = None
            try:
                bus_handle = open_i2c_bus(bus_id)
                address = self._resolve_imu_address(bus_handle, imu_addr)
                self.imu = ICM20948(bus_handle, address=address, enable_magnetometer=enable_mag)
                self.imu_bus = bus_handle
                self.imu_available = True
                self.imu_status = f"online /dev/i2c-{bus_id} @ 0x{address:02X}"
                self.imu_error = None
                print(f"[IMU] /dev/i2c-{bus_id} @ 0x{address:02X}, mag={'on' if enable_mag else 'off'}")
                if imu_auto_scan and imu_bus is not None and bus_id != imu_bus:
                    print(f"[IMU] 指定 bus /dev/i2c-{imu_bus} 不可用，已自動切換至 /dev/i2c-{bus_id}")
                return
            except FileNotFoundError:
                last_error = f"找不到 /dev/i2c-{bus_id}"
            except Exception as error:
                last_error = f"/dev/i2c-{bus_id} 初始化失敗: {error}"
            finally:
                if bus_handle is not None and bus_handle is not self.imu_bus:
                    try:
                        bus_handle.close()
                    except Exception:
                        pass

        self.imu_available = False
        self.imu_status = "offline"
        self.imu_error = last_error or "IMU 初始化失敗"
        print(
            "[IMU] 初始化失敗，將以無 IMU 模式繼續執行。"
            "請確認: SDA=GPIO2(pin3), SCL=GPIO3(pin5), NCS/CS 拉高到3.3V, "
            "AD0/SDO 依需求接 GND(0x68) 或 3.3V(0x69), 且已啟用 I2C"
        )
        print(f"[IMU] 原因: {self.imu_error}")

    def _load_imu_calibration_if_exists(self, file_path: str) -> None:
        if not os.path.exists(file_path):
            print(f"[IMU] 校正檔不存在，略過: {file_path}")
            return

        try:
            self.imu.load_calibration_file(file_path)
            print(f"[IMU] 已載入校正檔: {file_path}")
        except Exception as error:
            print(f"[IMU] 載入校正檔失敗: {error}")

    def _calibrate_gyro_bias(
        self,
        samples: int,
        interval_sec: float,
        outlier_threshold: float,
    ) -> tuple[float, float, float]:
        if samples < 10:
            raise ValueError("gyro_calib_samples 必須 >= 10")

        print("[IMU] Gyro bias calibration 開始，請保持靜止...")
        raw_samples: list[tuple[float, float, float]] = []

        for _ in range(samples):
            _, _, _, gx, gy, gz = self.imu.read_accel_gyro()
            raw_samples.append((gx, gy, gz))
            time.sleep(interval_sec)

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
            print("[IMU] Outlier 過多，回退使用全部樣本")
            filtered = raw_samples

        sx = sy = sz = 0.0
        for gx, gy, gz in filtered:
            sx += gx
            sy += gy
            sz += gz

        bias = (sx / len(filtered), sy / len(filtered), sz / len(filtered))
        print(
            "[IMU] Gyro bias = "
            f"({bias[0]:+.4f}, {bias[1]:+.4f}, {bias[2]:+.4f}) deg/s "
            f"accepted {len(filtered)}/{samples}"
        )
        return bias

    def rps_to_motor_params(self, rps: float) -> tuple[float, int, int]:
        direction = 1 if rps >= 0 else -1
        rps_abs = abs(rps)

        if rps_abs < 0.01:
            return 0.0, 0, 0

        p = int(self.motor_k / rps_abs)
        p = max(1, min(p, 4095))
        return rps_abs, p, direction

    def create_spi_data(self, p: int, direction: int, enable: bool = True) -> tuple[int, int]:
        enable_bit = 1 if enable else 0
        dir_bit = 1 if direction > 0 else 0

        byte1 = (enable_bit << 7) | (dir_bit << 6) | ((p >> 8) & 0x0F)
        byte2 = p & 0xFF
        return byte1, byte2

    def create_angle_spi_data(self, angle_deg: float) -> tuple[int, int, int]:
        angle_clamped = max(0.0, min(angle_deg, 360.0))
        angle_u16 = int(round((angle_clamped / 360.0) * 65535.0))
        return (angle_u16 >> 8) & 0xFF, angle_u16 & 0xFF, angle_u16

    def spi_transfer(self, cs_name: str, data: list[int]) -> list[int]:
        if cs_name not in CS_PINS:
            raise ValueError(f"Invalid CS name: {cs_name}")

        pin = CS_PINS[cs_name]
        try:
            GPIO.output(pin, GPIO.LOW)
            time.sleep(0.0001)
            response = self.spi.xfer2(data)
            GPIO.output(pin, GPIO.HIGH)
            return response
        except Exception as error:
            GPIO.output(pin, GPIO.HIGH)
            raise RuntimeError(f"SPI 傳輸失敗 ({cs_name}): {error}")

    def format_spi_binary(self, byte1: int, byte2: int) -> str:
        return f"{byte1:08b} {byte2:08b}"

    def calculate_move_angle_deg(self, vx: float, vy: float, min_speed: float = 0.02) -> float:
        speed = (vx * vx + vy * vy) ** 0.5
        if speed < min_speed:
            return 0.0
        angle = math.degrees(math.atan2(vy, vx))
        return (angle + 360.0) % 360.0

    def _update_loop_stats(self, loop_start_ns: int) -> None:
        if self._last_loop_start_ns is None:
            self._last_loop_start_ns = loop_start_ns
            return

        actual_period_ns = loop_start_ns - self._last_loop_start_ns
        self._last_loop_start_ns = loop_start_ns

        if actual_period_ns <= 0:
            return

        self.loop_stats.hz = 1_000_000_000.0 / actual_period_ns
        jitter_ns = actual_period_ns - self.target_period_ns
        self.loop_stats.jitter_ms = jitter_ns / 1_000_000.0
        self.loop_stats.worst_jitter_ms = max(
            self.loop_stats.worst_jitter_ms,
            abs(self.loop_stats.jitter_ms),
        )

    def _read_imu_fast(self, now_ns: int) -> None:
        if not self.imu_available or self.imu is None:
            return
        try:
            ax, ay, az, gx, gy, gz = self.imu.read_accel_gyro()
            self.imu_state.accel = (ax, ay, az)
            self.imu_state.gyro = (gx, gy, gz)

            if self.enable_mag and self.mag_rate_hz > 0.0 and now_ns >= self._next_mag_read_ns:
                self.imu_state.mag = self.imu.read_mag()
                self._next_mag_read_ns = now_ns + int(1_000_000_000 / self.mag_rate_hz)

            self.imu_state.temp_c = self.imu.read_temp()
        except Exception as error:
            self.imu_available = False
            self.imu_status = "offline"
            self.imu_error = f"讀取失敗: {error}"
            print(f"\n[IMU] {self.imu_error}，後續改為無 IMU 模式")

    def run(self) -> None:
        self.controller.wait_for_connection()
        print("[啟動] SPI + IMU 控制器啟動！(Ctrl+C 或長按X鍵離開)")
        print(
            f"Loop={self.loop_hz:.1f}Hz | "
            f"CS0={CS_PINS['CS0']} CS1={CS_PINS['CS1']} "
            f"CS2={CS_PINS['CS2']} CS3={CS_PINS['CS3']}"
        )

        self.controller.rumble(0.8, 0.8, 300)
        time.sleep(1)

        next_tick_ns = time.perf_counter_ns()

        try:
            while True:
                try:
                    loop_start_ns = time.perf_counter_ns()
                    self._update_loop_stats(loop_start_ns)

                    data = self.controller.read()
                    lx, ly = data["left_stick"]
                    lt = apply_trigger_deadzone(data["left_trigger"])
                    rt = apply_trigger_deadzone(data["right_trigger"])

                    lx = self.controller.apply_deadzone(lx)
                    ly = self.controller.apply_deadzone(ly)

                    buttons = data["buttons"]
                    self.braking = len(buttons) > BUTTON_A and buttons[BUTTON_A]

                    if len(buttons) > BUTTON_X and buttons[BUTTON_X]:
                        if not self.x_pressed:
                            self.x_pressed = True
                            self.x_press_start_time = time.time()
                        elif time.time() - self.x_press_start_time >= LONG_PRESS_DURATION:
                            print("\n[提示] 偵測到X鍵長按，準備退出...")
                            self.controller.rumble(1.0, 1.0, 200)
                            time.sleep(0.25)
                            self.controller.rumble(1.0, 1.0, 200)
                            break
                    else:
                        self.x_pressed = False

                    r = self.process_control_input(lx, ly, lt, rt)
                    w = r["wheels"]

                    self._read_imu_fast(loop_start_ns)

                    self.move_angle_deg = self.calculate_move_angle_deg(r["vx_pct"], r["vy_pct"])
                    angle_b1, angle_b2, angle_u16 = self.create_angle_spi_data(self.move_angle_deg)
                    self.spi_transfer("CS3", [angle_b1, angle_b2])

                    spi_data = []
                    for i, (rps, p, direction) in enumerate(
                        [
                            (w["rps1"], w["p1"], w["dir1"]),
                            (w["rps2"], w["p2"], w["dir2"]),
                            (w["rps3"], w["p3"], w["dir3"]),
                        ],
                        1,
                    ):
                        enable = (not self.braking) and abs(rps) > 0.01
                        b1, b2 = self.create_spi_data(p, direction, enable)
                        self.spi_transfer(f"CS{i-1}", [b1, b2])
                        spi_data.append((b1, b2, enable, direction > 0, p, rps))

                    sys.stdout.write("\033[H\033[J")
                    brake_status = " [BRAKE]" if self.braking else ""
                    print(f"=== SPI + IMU 輸出{brake_status} ===")
                    if self.imu_available:
                        print(f"IMU 狀態: {self.imu_status}")
                    else:
                        print(f"IMU 狀態: {self.imu_status} ({self.imu_error or '未知原因'})")
                    for i, (b1, b2, en, dir_pos, p, rps) in enumerate(spi_data, 1):
                        print(
                            f"輪{i}: {self.format_spi_binary(b1, b2)} | "
                            f"EN={int(en)} DIR={int(dir_pos)} p={p:4d} rps={rps:+.3f}"
                        )

                    print(
                        f"角度(CS3): {self.move_angle_deg:6.2f}° | "
                        f"{self.format_spi_binary(angle_b1, angle_b2)} (0x{angle_u16:04X})"
                    )
                    print(
                        f"控制輸入: target(X,Y,W)=({r['target_vx_pct']:+.2f}, {r['target_vy_pct']:+.2f}, {r['target_wz_pct']:+.2f}) "
                        f"smoothed=({r['vx_pct']:+.2f}, {r['vy_pct']:+.2f}, {r['wz_pct']:+.2f})"
                    )
                    if self.imu_available:
                        ax, ay, az = self.imu_state.accel
                        gx, gy, gz = self.imu_state.gyro
                        print(
                            f"IMU Accel(g): ({ax:+.3f}, {ay:+.3f}, {az:+.3f}) | "
                            f"Gyro(deg/s): ({gx:+.3f}, {gy:+.3f}, {gz:+.3f}) | "
                            f"Temp: {self.imu_state.temp_c:.1f}C"
                        )
                    else:
                        print("IMU Accel(g): (N/A, N/A, N/A) | Gyro(deg/s): (N/A, N/A, N/A) | Temp: N/A")

                    if self.imu_available and self.imu_state.mag is not None:
                        mx, my, mz = self.imu_state.mag
                        print(f"IMU Mag(uT): ({mx:+.2f}, {my:+.2f}, {mz:+.2f})")
                    else:
                        print("IMU Mag(uT): (disabled / no data)")

                    print(
                        f"Loop: target={self.loop_hz:.1f}Hz actual={self.loop_stats.hz:.1f}Hz "
                        f"jitter={self.loop_stats.jitter_ms:+.3f}ms "
                        f"worst={self.loop_stats.worst_jitter_ms:.3f}ms "
                        f"overrun={self.loop_stats.overrun_count}"
                    )

                    next_tick_ns += self.target_period_ns
                    now_ns = time.perf_counter_ns()
                    sleep_ns = next_tick_ns - now_ns

                    if sleep_ns > 0:
                        time.sleep(sleep_ns / 1_000_000_000.0)
                    else:
                        self.loop_stats.overrun_count += 1
                        next_tick_ns = now_ns

                except Exception as error:
                    print(f"\n錯誤: {error}")
                    time.sleep(0.2)

        except KeyboardInterrupt:
            print("\n\n[結束] 控制器已停止")
        finally:
            try:
                self.controller.rumble(1.0, 1.0, 150)
            except Exception:
                pass
            self.cleanup()

    def cleanup(self) -> None:
        print("\n正在清理資源...")

        try:
            b1, b2 = self.create_spi_data(0, 0, enable=False)
            for i in range(3):
                self.spi_transfer(f"CS{i}", [b1, b2])
            print("✓ 已停止所有馬達 (EN=0)")
        except Exception as error:
            print(f"停止馬達失敗: {error}")

        try:
            angle_b1, angle_b2, _ = self.create_angle_spi_data(0.0)
            self.spi_transfer("CS3", [angle_b1, angle_b2])
            print("✓ 已清除角度輸出 (CS3=0)")
        except Exception as error:
            print(f"清除角度輸出失敗: {error}")

        try:
            self.spi.close()
            print("✓ SPI 已關閉")
        except Exception as error:
            print(f"SPI 關閉失敗: {error}")

        try:
            if self.imu_available and self.imu is not None:
                self.imu.close()
                print("✓ IMU I2C bus 已關閉")
            else:
                print("✓ IMU 未啟用，略過關閉")
        except Exception as error:
            print(f"IMU 關閉失敗: {error}")

        try:
            GPIO.cleanup()
            print("✓ GPIO 已清理")
        except Exception as error:
            print(f"GPIO 清理失敗: {error}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--loop-hz", type=float, default=50.0, help="控制迴圈頻率")
    parser.add_argument("--motor-max-rps", type=float, default=MOTOR_MAX_RPS, help="最大馬達轉速")
    parser.add_argument(
        "--imu-bus",
        type=int,
        default=1,
        help="I2C bus 編號 (預設 1，對應 GPIO2/3 的 i2c_arm)",
    )
    parser.add_argument(
        "--imu-auto-scan",
        action="store_true",
        help="若指定 bus 不可用，才嘗試掃描其他 /dev/i2c-*",
    )
    parser.add_argument(
        "--imu-address",
        type=lambda value: int(value, 0),
        default=None,
        help="ICM-20948 I2C 位址 (預設自動偵測 0x68/0x69)",
    )
    parser.add_argument("--enable-mag", action="store_true", help="啟用磁力計低頻讀取")
    parser.add_argument("--mag-rate-hz", type=float, default=5.0, help="磁力計讀取頻率")
    parser.add_argument(
        "--imu-calib-file",
        type=str,
        default=DEFAULT_IMU_CALIB,
        help="IMU 校正檔路徑 (JSON)",
    )
    parser.add_argument(
        "--gyro-calib-samples",
        type=int,
        default=0,
        help="啟動時 gyro bias 校正樣本數，0=略過",
    )
    parser.add_argument(
        "--gyro-calib-interval",
        type=float,
        default=0.01,
        help="gyro bias 校正取樣間隔 (秒)",
    )
    parser.add_argument(
        "--gyro-calib-outlier-threshold",
        type=float,
        default=1.0,
        help="gyro bias outlier 門檻 (deg/s)",
    )
    parser.add_argument("--verbose", "-v", action="store_true", help="顯示初始化細節")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    robot = OmniRobotIMUSPIController(
        motor_max_rps=args.motor_max_rps,
        loop_hz=args.loop_hz,
        imu_bus=args.imu_bus,
        imu_auto_scan=args.imu_auto_scan,
        imu_addr=args.imu_address,
        enable_mag=args.enable_mag,
        mag_rate_hz=args.mag_rate_hz,
        imu_calib_file=args.imu_calib_file,
        gyro_calib_samples=args.gyro_calib_samples,
        gyro_calib_interval=args.gyro_calib_interval,
        gyro_calib_outlier_threshold=args.gyro_calib_outlier_threshold,
        verbose=args.verbose,
    )
    robot.run()


if __name__ == "__main__":
    main()
