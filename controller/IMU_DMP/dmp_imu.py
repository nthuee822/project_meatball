"""ICM-20948 DMP driver — quaternion + calibrated-gyro FIFO reader.

Initialization follows InvenSense app note AN-MAPPS (ICM-20648 DMP, identical
architecture to ICM-20948).  DMP firmware image sourced from SparkFun ICM-20948
Arduino Library (MIT).
"""

from __future__ import annotations

import dataclasses
import json
import math
import queue
import struct
import threading
import time
from collections import deque
from typing import Iterator

from IMU_DMP.icm20948 import ICM20948, open_i2c_bus
from IMU_DMP._dmp_firmware import DMP_FIRMWARE

# ---------------------------------------------------------------------------
# ICM-20948 hardware registers (Bank 0 unless noted)
# ---------------------------------------------------------------------------
_REG_PWR_MGMT_1       = 0x06   # Bank 0
_REG_PWR_MGMT_2       = 0x07
_REG_INT_ENABLE       = 0x10
_REG_INT_ENABLE_1     = 0x11
_REG_INT_ENABLE_2     = 0x12
_REG_INT_STATUS       = 0x19
_REG_INT_STATUS_1     = 0x1A   # DMP interrupt status
_REG_INT_STATUS_2     = 0x1B   # FIFO overflow status (bit 0)
_REG_FIFO_EN_1        = 0x66
_REG_FIFO_EN_2        = 0x67
_REG_FIFO_RST         = 0x68
_REG_FIFO_COUNT_H     = 0x70
_REG_FIFO_COUNT_L     = 0x71
_REG_FIFO_R_W         = 0x72
_REG_USER_CTRL        = 0x03
_REG_PRGM_STRT_ADDRH  = 0x50   # Bank 2
_REG_PRGM_STRT_ADDRL  = 0x51
_MEM_BANK_SEL         = 0x14   # DMP memory access registers (Bank 0)
_MEM_START_ADDR       = 0x15
_MEM_R_W              = 0x16
_REG_TIMEBASE_CORR_PLL = 0x28  # Bank 1

# Bank 2
_REG_GYRO_SMPLRT_DIV   = 0x00
_REG_GYRO_CONFIG_1     = 0x01
_REG_ACCEL_SMPLRT_DIV1 = 0x10
_REG_ACCEL_SMPLRT_DIV2 = 0x11
_REG_ACCEL_CONFIG      = 0x14

# ---------------------------------------------------------------------------
# DMP memory register addresses (bank*16 + offset, from app note Appendix I)
# ---------------------------------------------------------------------------
_DMP_DATA_OUT_CTL1    = 4 * 16        # 0x040
_DMP_DATA_OUT_CTL2    = 4 * 16 + 2    # 0x042
_DMP_DATA_INTR_CTL    = 4 * 16 + 12   # 0x04C
_DMP_MOTION_EVENT_CTL = 4 * 16 + 14   # 0x04E
_DMP_FIFO_WATERMARK   = 31 * 16 + 14  # 0x1FE
_DMP_ODR_QUAT6        = 10 * 16 + 12  # 0x0AC
_DMP_ODR_GYRO_CALIBR  = 11 * 16 + 8   # 0x0B8
_DMP_ODR_ACCEL        = 11 * 16 + 14  # 0x0BE
_DMP_ODR_GYRO         = 11 * 16 + 10  # 0x0BA
_DMP_ODR_CNTR_QUAT6   = 8 * 16 + 12   # 0x08C
_DMP_ODR_CNTR_GYRO_C  = 9 * 16 + 8    # 0x098
_DMP_GYRO_BIAS_X      = 139 * 16 + 4  # 0x8B4
_DMP_GYRO_BIAS_Y      = 139 * 16 + 8
_DMP_GYRO_BIAS_Z      = 139 * 16 + 12
_DMP_ACC_SCALE        = 30 * 16       # 0x1E0
_DMP_ACC_SCALE2       = 79 * 16 + 4   # 0x4F4
_DMP_GYRO_SF          = 19 * 16       # 0x130
_DMP_ACCEL_ONLY_GAIN  = 16 * 16 + 12  # 0x10C
_DMP_ACCEL_ALPHA_VAR  = 91 * 16       # 0x5B0
_DMP_ACCEL_A_VAR      = 92 * 16       # 0x5C0

# ---------------------------------------------------------------------------
# FIFO output config (DATA_OUT_CTL1 bitmask)
# Enabled: Accel(0x8000)|Gyro(0x4000)|Quat6(0x0800)|CalibratedGyro(0x0040)
# ---------------------------------------------------------------------------
_DATA_OUT_MASK  = 0xCC40
_DATA_INTR_MASK = 0x0800   # interrupt triggers on Quat6 packet
_MOTION_EVENTS  = 0x0300   # enable accel(0x0200) + gyro(0x0100) dynamic calibration

# DMP base clock 225 Hz; ODR value = (225 / target_hz) - 1
# 112 Hz → value = 1
_ODR_112HZ = 1

# ---------------------------------------------------------------------------
# Sensor scales
# ---------------------------------------------------------------------------
_GYRO_FSR_DPS        = 2000     # ±2000 dps configured for DMP
_ACCEL_FSR_G         = 4        # ±4g configured for DMP
_GYRO_SENSITIVITY    = 32768.0 / _GYRO_FSR_DPS   # LSB/dps = 16.384
_ACCEL_SENSITIVITY   = 32768.0 / _ACCEL_FSR_G    # LSB/g   = 8192
_CAL_GYRO_SCALE      = 1 << 15  # calibrated gyro: hw_unit * 2^15
_DEG2RAD             = math.pi / 180.0

# FIFO packet byte sizes for DATA_OUT_MASK = 0xCC40
_HEADER_SIZE   = 2
_ACCEL_SIZE    = 6   # int16 × 3
_GYRO_RAW_SIZE = 6   # int16 × 3
_QUAT6_SIZE    = 12  # int32 × 3 (Q30)
_CAL_GYRO_SIZE = 12  # int32 × 3
PACKET_SIZE    = _HEADER_SIZE + _ACCEL_SIZE + _GYRO_RAW_SIZE + _QUAT6_SIZE + _CAL_GYRO_SIZE
# = 38 bytes

# DMP firmware load start address (DMP internal memory offset)
_DMP_LOAD_START = 0x90


# ---------------------------------------------------------------------------
# Output data structure
# ---------------------------------------------------------------------------
@dataclasses.dataclass
class DMPFrame:
    """One DMP FIFO packet, fully decoded."""
    timestamp: float          # time.monotonic(), seconds
    dt: float                 # seconds since previous packet
    qw: float                 # quaternion w  (derived: sqrt(1-qx²-qy²-qz²))
    qx: float                 # quaternion x  (Q1, Q30/2^30)
    qy: float                 # quaternion y  (Q2)
    qz: float                 # quaternion z  (Q3)
    roll: float               # radians
    pitch: float              # radians
    yaw: float                # radians
    gyro_raw: tuple           # (gx, gy, gz) rad/s — raw 16-bit from FIFO
    gyro_cal: tuple           # (gx, gy, gz) rad/s — DMP-calibrated 32-bit
    accel_raw: tuple          # (ax, ay, az) g  — raw 16-bit from FIFO
    fifo_count: int           # FIFO byte count at read time
    fifo_overflow: bool       # True if FIFO overflow flag was set
    packet_interval_ms: float # dt × 1000


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _quat_to_euler(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    roll  = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw   = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def _calc_gyro_sf(pll: int) -> int:
    """Compute GYRO_SF from TIMEBASE_CORRECTION_PLL (app note Appendix II)."""
    BASE_SAMPLE_RATE = 1125
    DMP_FREQ         = 225
    DMP_DIVIDER      = BASE_SAMPLE_RATE // DMP_FREQ   # = 5
    t     = 102870 + 81 * pll
    a     = (1 << 30) // t
    r     = (1 << 30) - a * t
    value = a * 797 * DMP_DIVIDER
    value += (a * 1011387 * DMP_DIVIDER) >> 20
    value += r * 797 * DMP_DIVIDER // t
    value += ((r * 1011387 * DMP_DIVIDER) >> 20) // t
    value <<= 1
    return value & 0xFFFFFFFF


# ---------------------------------------------------------------------------
# Main class
# ---------------------------------------------------------------------------
class DMPIMU:
    """ICM-20948 DMP driver.

    Usage::

        imu = DMPIMU()
        imu.start()                         # blocks ~1 s for firmware load
        frame = imu.get_latest()            # non-blocking, may return None
        for frame in imu.iter_frames():     # blocking iterator
            ...
        imu.stop()
    """

    def __init__(
        self,
        i2c_bus: int = 1,
        i2c_addr: int = 0x68,
        calib_path: str | None = "IMU_DMP/imu_calibration.json",
    ) -> None:
        self._bus_id   = i2c_bus
        self._addr     = i2c_addr
        self._calib    = calib_path
        self._imu: ICM20948 | None = None
        self._running  = False
        self._thread: threading.Thread | None = None
        self._latest: DMPFrame | None = None
        self._queue: queue.Queue[DMPFrame] = queue.Queue(maxsize=0)
        self._initialized  = False
        self._use_i2c_rdwr = False   # set in init() after bus is open

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def init(self) -> bool:
        """Initialize hardware and load DMP firmware. Returns True on success."""
        bus = open_i2c_bus(self._bus_id)
        # Disable magnetometer — DMP does not use it for 6-axis mode
        self._imu = ICM20948(bus, self._addr, enable_magnetometer=False)
        # Prefer raw I2C (i2c_rdwr) over SMBus for DMP memory writes —
        # SMBus block writes sometimes NAK on ICM-20948 for large bursts.
        try:
            from smbus2 import i2c_msg  # noqa: F401
            self._use_i2c_rdwr = hasattr(bus, "i2c_rdwr")
        except ImportError:
            self._use_i2c_rdwr = False
        print(f"[DMPIMU] I2C backend: {'i2c_rdwr (raw)' if self._use_i2c_rdwr else 'write_i2c_block_data (smbus)'}")
        try:
            self._dmp_init()
            self._initialized = True
            return True
        except Exception as exc:
            import traceback
            print(f"[DMPIMU] init failed: {exc}")
            traceback.print_exc()
            return False

    def start(self) -> None:
        """Initialize (if needed) and start the background FIFO reader thread."""
        if not self._initialized:
            if not self.init():
                raise RuntimeError("DMP initialization failed")
        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True, name="dmp-reader")
        self._thread.start()

    def stop(self) -> None:
        """Stop the background reader thread."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def get_latest(self) -> DMPFrame | None:
        """Return the most recent DMPFrame without blocking (or None)."""
        return self._latest

    def iter_frames(self) -> Iterator[DMPFrame]:
        """Blocking iterator; yields each DMPFrame as it arrives."""
        while self._running or not self._queue.empty():
            try:
                yield self._queue.get(timeout=0.1)
            except queue.Empty:
                continue

    # ------------------------------------------------------------------
    # DMP initialization (per app note §3 + §4)
    # ------------------------------------------------------------------
    def _dmp_init(self) -> None:
        imu = self._imu

        # --- §3.1 Re-configure sensors for DMP ---
        # ICM20948.__init__ already woke the chip (PWR_MGMT_1=0x01) and
        # enabled sensors (PWR_MGMT_2=0x00). Set DMP-required FSR + sample rate.

        # GYRO_CONFIG_1 (Bank 2, 0x01): ±2000 dps, DLPF on
        # bits[4:3]=11=±2000dps, bit[0]=1=DLPF, bits[2:1]=00=DLPF config 0
        imu._write(2, _REG_GYRO_CONFIG_1, 0x19)   # 0b00011001

        # ACCEL_CONFIG (Bank 2, 0x14): ±4g, DLPF on
        # bits[2:1]=01=±4g → 0x02, bit[0]=1=DLPF → 0x03
        imu._write(2, _REG_ACCEL_CONFIG, 0x03)    # 0b00000011

        # Sample rate dividers → 225 Hz (must match DMP base clock)
        # GYRO_SMPLRT_DIV (Bank 2, 0x00): 1125/(div+1)=225 → div=4
        imu._write(2, _REG_GYRO_SMPLRT_DIV, 4)
        imu._write(2, _REG_ACCEL_SMPLRT_DIV1, 0)
        imu._write(2, _REG_ACCEL_SMPLRT_DIV2, 4)

        # --- §3.2 Configure FIFO and interrupts ---
        imu._write(0, _REG_INT_ENABLE_2, 0x01)   # enable FIFO overflow interrupt
        imu._write(0, _REG_FIFO_EN_1,    0x00)   # DMP owns FIFO, not hw sensors
        imu._write(0, _REG_FIFO_EN_2,    0x00)
        imu._write(0, _REG_INT_ENABLE_1, 0x00)   # disable data-ready interrupt

        # --- §3.3 Reset FIFO ---
        imu._write(0, _REG_FIFO_RST, 0x1F)
        time.sleep(0.01)
        imu._write(0, _REG_FIFO_RST, 0x00)

        # --- §3.5 Set DMP start address (0x1000) ---
        imu._write(2, _REG_PRGM_STRT_ADDRH, 0x10)
        imu._write(2, _REG_PRGM_STRT_ADDRL, 0x00)

        # --- §3.6 Load DMP firmware ---
        print("[DMPIMU] Loading DMP firmware …", flush=True)
        self._load_firmware()
        print(f"[DMPIMU] Firmware loaded ({len(DMP_FIRMWARE)} bytes)", flush=True)

        # --- §3.7 Configure accel scale to DMP (1g = 2^25 internal) ---
        # For ±4g FSR: ACC_SCALE=0x04000000, ACC_SCALE2=0x00040000
        self._write_dmp_reg(_DMP_ACC_SCALE,  struct.pack(">I", 0x04000000))
        self._write_dmp_reg(_DMP_ACC_SCALE2, struct.pack(">I", 0x00040000))

        # --- §4.5.1 Accel calibration gains for 112 Hz ---
        self._write_dmp_reg(_DMP_ACCEL_ONLY_GAIN, struct.pack(">I", 30504029))
        self._write_dmp_reg(_DMP_ACCEL_ALPHA_VAR, struct.pack(">I", 977872018))
        self._write_dmp_reg(_DMP_ACCEL_A_VAR,     struct.pack(">I", 95869806))

        # --- §4.5.10 GYRO_SF from TIMEBASE_CORRECTION_PLL ---
        pll_raw = imu._read(1, _REG_TIMEBASE_CORR_PLL, 1)[0]
        pll = pll_raw if pll_raw < 128 else pll_raw - 256   # signed 8-bit
        gyro_sf = _calc_gyro_sf(pll)
        self._write_dmp_reg(_DMP_GYRO_SF, struct.pack(">I", gyro_sf))

        # --- §4.6 Seed initial gyro bias from calibration file ---
        self._load_gyro_bias()

        # --- §4.2 Configure DMP FIFO outputs ---
        self._write_dmp_reg(_DMP_DATA_OUT_CTL1,  struct.pack(">H", _DATA_OUT_MASK))
        self._write_dmp_reg(_DMP_DATA_OUT_CTL2,  struct.pack(">H", 0x0000))
        self._write_dmp_reg(_DMP_DATA_INTR_CTL,  struct.pack(">H", _DATA_INTR_MASK))
        # FIFO watermark: ~800 bytes ≈ 21 packets headroom before overflow
        self._write_dmp_reg(_DMP_FIFO_WATERMARK, struct.pack(">H", 800))

        # --- §4.3 ODR = 112 Hz for all enabled outputs ---
        self._write_dmp_reg(_DMP_ODR_QUAT6,       struct.pack(">H", _ODR_112HZ))
        self._write_dmp_reg(_DMP_ODR_GYRO_CALIBR, struct.pack(">H", _ODR_112HZ))
        self._write_dmp_reg(_DMP_ODR_ACCEL,       struct.pack(">H", _ODR_112HZ))
        self._write_dmp_reg(_DMP_ODR_GYRO,        struct.pack(">H", _ODR_112HZ))
        # Reset ODR counters
        self._write_dmp_reg(_DMP_ODR_CNTR_QUAT6,  b"\x00\x00")
        self._write_dmp_reg(_DMP_ODR_CNTR_GYRO_C, b"\x00\x00")

        # --- §4.5 Enable dynamic accel + gyro calibration ---
        self._write_dmp_reg(_DMP_MOTION_EVENT_CTL, struct.pack(">H", _MOTION_EVENTS))

        # --- §3.9 Start DMP + enable FIFO ---
        # USER_CTRL: DMP_EN(7)|FIFO_EN(6)|DMP_RST(3) = 0xC8
        # DMP_RST pulses the DMP program counter to start from PRGM_STRT_ADDR (0x1000)
        # Do NOT set bit 2 (SRAM_RST) — that erases the firmware we just loaded
        imu._write(0, _REG_USER_CTRL, 0xC8)
        time.sleep(0.05)

        # --- §3.10 Enable DMP interrupt (bit 1) ---
        imu._write(0, _REG_INT_ENABLE, 0x02)

    # ------------------------------------------------------------------
    # DMP memory helpers
    # ------------------------------------------------------------------

    # Max bytes per DMP SRAM write — matches InvenSense INV_MAX_SERIAL_WRITE
    _DMP_WRITE_MAX = 16

    def _mem_write(self, data: list) -> None:
        """Raw I2C write of [reg, d0, d1, ...] to the device (no SMBus framing).
        Falls back to write_i2c_block_data if i2c_rdwr unavailable."""
        imu = self._imu
        if self._use_i2c_rdwr:
            from smbus2 import i2c_msg
            imu.bus.i2c_rdwr(i2c_msg.write(imu.addr, data))
        else:
            imu.bus.write_i2c_block_data(imu.addr, data[0], data[1:])

    def _write_dmp_reg(self, dmp_addr: int, data: bytes) -> None:
        """Write to DMP internal memory at dmp_addr, respecting bank boundaries."""
        imu = self._imu
        with imu._io_lock:
            imu._bank_sel(0)
            offset = 0
            while offset < len(data):
                mem_addr = dmp_addr + offset
                bank  = mem_addr >> 8
                start = mem_addr & 0xFF
                # Don't cross 256-byte DMP page boundary, and keep to write max
                chunk = min(256 - start, self._DMP_WRITE_MAX, len(data) - offset)
                imu.bus.write_byte_data(imu.addr, _MEM_BANK_SEL,   bank)
                imu.bus.write_byte_data(imu.addr, _MEM_START_ADDR, start)
                self._mem_write([_MEM_R_W] + list(data[offset:offset + chunk]))
                offset += chunk

    def _read_dmp_reg(self, dmp_addr: int, length: int) -> bytes:
        """Read from DMP internal memory at dmp_addr."""
        imu = self._imu
        with imu._io_lock:
            imu._bank_sel(0)
            bank  = dmp_addr >> 8
            start = dmp_addr & 0xFF
            imu.bus.write_byte_data(imu.addr, _MEM_BANK_SEL,   bank)
            imu.bus.write_byte_data(imu.addr, _MEM_START_ADDR, start)
            return bytes(imu.bus.read_i2c_block_data(imu.addr, _MEM_R_W, length))

    def _load_firmware(self) -> None:
        """Load DMP_FIRMWARE into DMP SRAM starting at _DMP_LOAD_START.

        Writes 16 bytes at a time (INV_MAX_SERIAL_WRITE), updating MEM_BANK_SEL
        and MEM_START_ADDR before every chunk so the DMP address pointer is always
        explicit — never relying on auto-increment across chunk boundaries.
        """
        fw   = DMP_FIRMWARE
        size = len(fw)
        imu  = self._imu
        with imu._io_lock:
            imu._bank_sel(0)
            offset = 0
            while offset < size:
                mem_addr = _DMP_LOAD_START + offset
                bank  = mem_addr >> 8
                start = mem_addr & 0xFF
                # Respect page boundary AND per-write size limit
                chunk = min(256 - start, self._DMP_WRITE_MAX, size - offset)
                imu.bus.write_byte_data(imu.addr, _MEM_BANK_SEL,   bank)
                imu.bus.write_byte_data(imu.addr, _MEM_START_ADDR, start)
                self._mem_write([_MEM_R_W] + list(fw[offset:offset + chunk]))
                offset += chunk

    def _load_gyro_bias(self) -> None:
        """Seed DMP gyro bias registers from calibration JSON (if present)."""
        if not self._calib:
            return
        try:
            with open(self._calib, "r") as fp:
                cal = json.load(fp)
            bias_dps = cal.get("gyro", {}).get("bias", [0.0, 0.0, 0.0])
        except (OSError, KeyError, json.JSONDecodeError):
            return

        # DMP GYRO_BIAS is in hardware units × 2^15 (32-bit signed, big-endian)
        # hw_unit = dps × GYRO_SENSITIVITY
        scale = _GYRO_SENSITIVITY * _CAL_GYRO_SCALE
        regs  = [_DMP_GYRO_BIAS_X, _DMP_GYRO_BIAS_Y, _DMP_GYRO_BIAS_Z]
        for reg, dps in zip(regs, bias_dps):
            val = max(-2**31, min(2**31 - 1, int(dps * scale)))
            self._write_dmp_reg(reg, struct.pack(">i", val))

    # ------------------------------------------------------------------
    # Background reader thread
    # ------------------------------------------------------------------
    def _reader_loop(self) -> None:
        imu      = self._imu
        last_ts  = time.monotonic()
        _err_count = 0

        while self._running:
            try:
                # Read FIFO byte count (Bank 0)
                with imu._io_lock:
                    imu._bank_sel(0)
                    count_h  = imu.bus.read_byte_data(imu.addr, _REG_FIFO_COUNT_H)
                    count_l  = imu.bus.read_byte_data(imu.addr, _REG_FIFO_COUNT_L)
                    overflow = bool(imu.bus.read_byte_data(imu.addr, _REG_INT_STATUS_2) & 0x01)
                count = (count_h << 8) | count_l
                _err_count = 0
            except OSError:
                _err_count += 1
                if _err_count >= 10:
                    print("[DMPIMU] reader: too many I2C errors, exiting thread")
                    self._running = False
                    return
                time.sleep(0.005)
                continue

            if count < PACKET_SIZE:
                time.sleep(0.001)
                continue

            try:
                # Read exactly one packet from FIFO
                with imu._io_lock:
                    imu._bank_sel(0)
                    raw = bytes(imu.bus.read_i2c_block_data(imu.addr, _REG_FIFO_R_W, PACKET_SIZE))
                ts = time.monotonic()
            except OSError:
                time.sleep(0.005)
                continue

            # --- Parse header ---
            header = struct.unpack_from(">H", raw, 0)[0]
            pos    = 2

            # Accel (bit 0x8000) — int16 × 3, big-endian
            ax = ay = az = 0.0
            if header & 0x8000:
                ax_i, ay_i, az_i = struct.unpack_from(">hhh", raw, pos)
                ax = ax_i / _ACCEL_SENSITIVITY
                ay = ay_i / _ACCEL_SENSITIVITY
                az = az_i / _ACCEL_SENSITIVITY
                pos += 6

            # Gyro raw (bit 0x4000) — int16 × 3
            gx_r = gy_r = gz_r = 0.0
            if header & 0x4000:
                gx_i, gy_i, gz_i = struct.unpack_from(">hhh", raw, pos)
                gx_r = (gx_i / _GYRO_SENSITIVITY) * _DEG2RAD
                gy_r = (gy_i / _GYRO_SENSITIVITY) * _DEG2RAD
                gz_r = (gz_i / _GYRO_SENSITIVITY) * _DEG2RAD
                pos += 6

            # 6-axis quaternion (bit 0x0800) — int32 × 3, Q30 format
            # Only Q1, Q2, Q3 stored; Q0 = sqrt(1 - Q1²-Q2²-Q3²)
            q1 = q2 = q3 = 0.0
            if header & 0x0800:
                q1i, q2i, q3i = struct.unpack_from(">iii", raw, pos)
                q1 = q1i / (1 << 30)
                q2 = q2i / (1 << 30)
                q3 = q3i / (1 << 30)
                pos += 12
            sq = q1 * q1 + q2 * q2 + q3 * q3
            q0 = math.sqrt(max(0.0, 1.0 - sq))

            # Calibrated gyro (bit 0x0040) — int32 × 3, hw_unit × 2^15
            gx_c = gy_c = gz_c = 0.0
            if header & 0x0040:
                gx_ci, gy_ci, gz_ci = struct.unpack_from(">iii", raw, pos)
                gx_c = (gx_ci / _CAL_GYRO_SCALE / _GYRO_SENSITIVITY) * _DEG2RAD
                gy_c = (gy_ci / _CAL_GYRO_SCALE / _GYRO_SENSITIVITY) * _DEG2RAD
                gz_c = (gz_ci / _CAL_GYRO_SCALE / _GYRO_SENSITIVITY) * _DEG2RAD
                pos += 12

            roll, pitch, yaw = _quat_to_euler(q0, q1, q2, q3)

            dt = ts - last_ts
            last_ts = ts

            frame = DMPFrame(
                timestamp=ts,
                dt=dt,
                qw=q0, qx=q1, qy=q2, qz=q3,
                roll=roll, pitch=pitch, yaw=yaw,
                gyro_raw=(gx_r, gy_r, gz_r),
                gyro_cal=(gx_c, gy_c, gz_c),
                accel_raw=(ax, ay, az),
                fifo_count=count,
                fifo_overflow=overflow,
                packet_interval_ms=dt * 1000.0,
            )
            self._latest = frame
            self._queue.put(frame)
