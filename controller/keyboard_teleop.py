"""三輪全向輪機器人 - 終端機鍵盤控制 (SPI 輸出)

沒有 F710 手把時使用。直接從終端機讀取按鍵（SSH / VSCode 遠端終端機皆可），
不需要圖形介面。沿用 OmniRobotSPIController 的逆運動學、平滑化與 SPI 輸出。

終端機收不到「放開按鍵」事件，因此以鍵盤自動重複 (auto-repeat) 判斷是否按住：
超過 hold_timeout 秒沒再收到按鍵就視為放開，目標速度歸零。
"""
import argparse
import os
import select
import signal
import sys
import termios
import time
import tty

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

from controller2spi import OmniRobotSPIController
from omni_robot import MOTOR_MAX_RPS, UPDATE_RATE

# ==================== 參數配置區 ====================
HOLD_TIMEOUT = 0.6     # 放開判定時間 (秒)，需大於作業系統的按鍵重複延遲
DEFAULT_SPEED = 0.5    # 預設速度比例 (0.1 ~ 1.0)
# =====================================================

DIAG = 0.7071  # 斜向分量 (1/√2)，讓斜走與直走速度一致

# 按鍵 → (lx, ly, wz)，lx/ly 採手把搖桿座標：lx 向右為正、ly 向上(前進)為負
MOVE_KEYS = {
    'w': (0.0, -1.0, 0.0), 's': (0.0, 1.0, 0.0),
    'a': (-1.0, 0.0, 0.0), 'd': (1.0, 0.0, 0.0),
    'q': (-DIAG, -DIAG, 0.0), 'e': (DIAG, -DIAG, 0.0),
    'z': (-DIAG, DIAG, 0.0), 'c': (DIAG, DIAG, 0.0),
    'UP': (0.0, -1.0, 0.0), 'DOWN': (0.0, 1.0, 0.0),
    'LEFT': (-1.0, 0.0, 0.0), 'RIGHT': (1.0, 0.0, 0.0),
    'j': (0.0, 0.0, -1.0),  # 自轉 −（同左扳機 LT）
    'l': (0.0, 0.0, 1.0),   # 自轉 +（同右扳機 RT）
}
KEY_LABELS = {'UP': '↑', 'DOWN': '↓', 'LEFT': '←', 'RIGHT': '→'}
ARROW_CODES = {'A': 'UP', 'B': 'DOWN', 'C': 'RIGHT', 'D': 'LEFT'}

HELP_LINES = [
    "=== 鍵盤控制 Omni Robot (SPI) ===",
    "  Q W E    斜前 / 前進 / 斜前      J / L  : 自轉 − / +",
    "  A   D    左移 /      / 右移      方向鍵 : 前後左右",
    "  Z S C    斜後 / 後退 / 斜後      1~9, 0 : 速度 10%~100%",
    "  Space : 立即停止                 Ctrl+C : 離開",
    "  (按住移動，放開即停；一次只能按一個移動鍵)",
]


def parse_keys(data):
    """把終端機讀到的字串拆成按鍵名稱（方向鍵轉成 UP/DOWN/LEFT/RIGHT）"""
    keys = []
    i = 0
    while i < len(data):
        ch = data[i]
        if ch == '\x1b' and i + 2 < len(data) and data[i + 1] in '[O':
            arrow = ARROW_CODES.get(data[i + 2])
            if arrow:
                keys.append(arrow)
            i += 3
            continue
        keys.append(ch.lower())
        i += 1
    return keys


def read_available_keys(fd):
    """非阻塞讀取目前緩衝區內所有按鍵"""
    data = b''
    while select.select([fd], [], [], 0)[0]:
        chunk = os.read(fd, 1024)
        if not chunk:
            break
        data += chunk
    return parse_keys(data.decode('utf-8', errors='ignore'))


class KeyboardSPIController(OmniRobotSPIController):
    """以終端機鍵盤取代 F710 手把"""

    def __init__(self, motor_max_rps=MOTOR_MAX_RPS, speed=DEFAULT_SPEED,
                 hold_timeout=HOLD_TIMEOUT):
        super().__init__(motor_max_rps=motor_max_rps)
        self.speed = speed
        self.hold_timeout = hold_timeout
        self.held_key = None
        self.last_key_time = 0.0
        self.stopped_flash_until = 0.0

    def handle_keys(self, keys, now):
        for key in keys:
            if key in MOVE_KEYS:
                self.held_key = key
                self.last_key_time = now
            elif key == ' ':
                self.emergency_stop(now)
            elif key in '123456789':
                self.speed = int(key) / 10.0
            elif key == '0':
                self.speed = 1.0

    def emergency_stop(self, now):
        """清除按鍵並跳過減速平滑，下一次輸出即全部 EN=0"""
        self.held_key = None
        self.current_lx = self.current_ly = self.current_wz = 0.0
        self.stopped_flash_until = now + 1.0

    def current_command(self, now):
        """回傳 (lx, ly, lt, rt)，格式與手把輸入相同"""
        if self.held_key is None or now - self.last_key_time > self.hold_timeout:
            self.held_key = None
            return 0.0, 0.0, -1.0, -1.0
        x, y, w = MOVE_KEYS[self.held_key]
        s = self.speed
        # 扳機範圍 -1(放開) ~ 1(按滿)
        lt = 2.0 * s * max(-w, 0.0) - 1.0
        rt = 2.0 * s * max(w, 0.0) - 1.0
        return x * s, y * s, lt, rt

    def send_outputs(self, r):
        """送出角度 (CS3) 與三輪 (CS0~CS2) SPI 數據"""
        self.move_angle_deg = self.calculate_move_angle_deg(r['vx_pct'], r['vy_pct'])
        angle_b1, angle_b2, angle_u16 = self.create_angle_spi_data(self.move_angle_deg)
        self.spi_transfer('CS3', [angle_b1, angle_b2])

        w = r['wheels']
        spi_data = []
        for i in range(1, 4):
            rps, p, d = w[f'rps{i}'], w[f'p{i}'], w[f'dir{i}']
            enable = abs(rps) > 0.01
            b1, b2 = self.create_spi_data(p, d, enable)
            self.spi_transfer(f'CS{i - 1}', [b1, b2])
            spi_data.append((b1, b2, enable, d > 0, p))
        return spi_data, (angle_b1, angle_b2, angle_u16)

    def stop_motors(self):
        """不輸出任何訊息地送出 EN=0（斷線時也能安全執行）"""
        b1, b2 = self.create_spi_data(0, 0, enable=False)
        for i in range(3):
            self.spi_transfer(f'CS{i}', [b1, b2])

    def draw(self, r, spi_data, angle, now):
        key = self.held_key
        key_text = KEY_LABELS.get(key, key.upper()) if key else '-'
        status = "  [STOP]" if now < self.stopped_flash_until else ""
        angle_b1, angle_b2, angle_u16 = angle
        lines = HELP_LINES + [
            "-" * 62,
            f"速度: {self.speed * 100:3.0f}%   按鍵: {key_text}{status}",
            f"目標輸入: X={r['target_vx_pct']:+.2f}, Y={r['target_vy_pct']:+.2f}, W={r['target_wz_pct']:+.2f}",
            f"平滑輸入: X={r['vx_pct']:+.2f}, Y={r['vy_pct']:+.2f}, W={r['wz_pct']:+.2f}",
            "-" * 62,
        ]
        wheel_names = ['輪1(前)  ', '輪2(左後)', '輪3(右後)']
        for name, (b1, b2, en, dir_pos, p) in zip(wheel_names, spi_data):
            lines.append(f"{name}: {self.format_spi_binary(b1, b2)} | EN={int(en)} DIR={int(dir_pos)} p={p:4d}")
        lines.append(
            f"角度(CS3): {self.move_angle_deg:6.2f}° | "
            f"{self.format_spi_binary(angle_b1, angle_b2)} (0x{angle_u16:04X})"
        )
        sys.stdout.write("\033[H" + "\033[K\n".join(lines) + "\033[K\n\033[J")
        sys.stdout.flush()

    def run(self):
        if not sys.stdin.isatty():
            print("錯誤: 請在終端機中執行（需要讀取鍵盤）")
            self.cleanup()
            return

        def on_signal(signum, frame):
            if signum == signal.SIGHUP:
                # SSH 斷線：終端機已消失，避免之後的 print 出錯
                sys.stdout = sys.stderr = open(os.devnull, 'w')
            raise KeyboardInterrupt

        signal.signal(signal.SIGHUP, on_signal)
        signal.signal(signal.SIGTERM, on_signal)

        fd = sys.stdin.fileno()
        old_attrs = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            sys.stdout.write("\033[2J\033[?25l")  # 清屏、隱藏游標
            next_tick = time.monotonic()
            while True:
                now = time.monotonic()
                self.handle_keys(read_available_keys(fd), now)
                lx, ly, lt, rt = self.current_command(now)
                r = self.process_control_input(lx, ly, lt, rt)
                spi_data, angle = self.send_outputs(r)
                self.draw(r, spi_data, angle, now)

                next_tick += UPDATE_RATE
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()
        except KeyboardInterrupt:
            pass
        finally:
            # 先停馬達，再處理終端機與其他資源
            try:
                self.stop_motors()
            except Exception:
                pass
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
            except Exception:
                pass
            try:
                sys.stdout.write("\033[?25h\n\n[結束] 鍵盤控制器已停止\n")
                self.cleanup()
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser(description="三輪全向輪機器人 - 終端機鍵盤控制 (SPI)")
    parser.add_argument('--speed', type=float, default=DEFAULT_SPEED,
                        help=f"初始速度比例 0.1~1.0 (預設 {DEFAULT_SPEED})")
    parser.add_argument('--hold-timeout', type=float, default=HOLD_TIMEOUT,
                        help=f"放開判定時間，秒 (預設 {HOLD_TIMEOUT})；按住時會一頓一頓就調大")
    args = parser.parse_args()

    speed = max(0.1, min(args.speed, 1.0))
    robot = KeyboardSPIController(speed=speed, hold_timeout=args.hold_timeout)
    robot.run()


if __name__ == "__main__":
    main()
