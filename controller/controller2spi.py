"""三輪全向輪機器人主程式 - SPI 輸出版本"""
import sys
import time
import math
try:
    import spidev
    import RPi.GPIO as GPIO
except ImportError:
    print("警告: 缺少 spidev 或 RPi.GPIO，請確保在 Raspberry Pi 上執行")
    sys.exit(1)

from omni_robot import OmniRobotController, MOTOR_MAX_RPS, UPDATE_RATE

# 扳機死區設定
TRIGGER_DEADZONE = 0.1  # 扳機死區 (0.0 ~ 1.0)

def apply_trigger_deadzone(trigger_value, deadzone=TRIGGER_DEADZONE):
    """處理扳機死區（-1~1 範圍）"""
    normalized = (trigger_value + 1.0) / 2.0  # 轉換到 0~1
    if normalized < deadzone:
        return -1.0
    # 重新映射到 -1~1，移除死區影響
    return (normalized - deadzone) / (1.0 - deadzone) * 2.0 - 1.0

# SPI Chip Select pins (from spi_cs.py)
CS_PINS = {
    'CS0': 17,  # 輪1 (前)
    'CS1': 27,  # 輪2 (左後)
    'CS2': 22,  # 輪3 (右後)
    'CS3': 5    # 角度輸出通道
}

# 按鈕配置 (Logitech F710)
BUTTON_A = 0  # A按鈕索引 (煞車)
BUTTON_X = 2  # X按鈕索引
LONG_PRESS_DURATION = 1.5  # 長按時間閾值（秒）

class OmniRobotSPIController(OmniRobotController):
    """繼承 OmniRobotController，添加 SPI 輸出功能"""
    
    def __init__(self, motor_max_rps=MOTOR_MAX_RPS, verbose=False):
        # 調用父類初始化（會自動初始化平滑控制參數）
        super().__init__(motor_max_rps=motor_max_rps)
        
        self.verbose = verbose
        
        # 煞車狀態 (A鍵)
        self.braking = False

        # 行進角度狀態（度，0~360）
        self.move_angle_deg = 0.0
        
        # 長按X鍵退出機制
        self.x_pressed = False
        self.x_press_start_time = 0
        
        # 初始化 GPIO 和 SPI
        try:
            self._init_gpio_spi()
        except Exception as e:
            print(f"錯誤: GPIO/SPI 初始化失敗: {e}")
            raise
    
    def _init_gpio_spi(self):
        """初始化 GPIO 和 SPI"""
        # 初始化 GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # 設置 chip select pins
        for name, pin in CS_PINS.items():
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH)
            if self.verbose:
                print(f"Initialized {name} on GPIO {pin}")
        
        # 初始化 SPI
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)  # Bus 0, Device 0
        self.spi.max_speed_hz = 500000
        self.spi.mode = 0
        
        if self.verbose:
            print("SPI initialized: Bus 0, Device 0, Speed 500kHz, Mode 0")
    
    def rps_to_motor_params(self, rps):
        """
        重寫父類方法：調整 p 的範圍為 12-bit (0-4095)
        :param rps: 目標轉速 (rotation per second)
        :return: (rps_abs, p, direction) 
        """
        # 方向判斷
        direction = 1 if rps >= 0 else -1
        rps_abs = abs(rps)
        
        # 低於閾值則停止馬達（p=0 表示停止）
        if rps_abs < 0.01:
            return 0.0, 0, 0
        
        # RPS → p 參數: RPS = K/p => p = K/RPS
        p = int(self.motor_k / rps_abs)
        p = max(1, min(p, 4095))  # 限制在 [1, 4095]，0 保留給停止狀態
        
        return rps_abs, p, direction
    
    def create_spi_data(self, p, direction, enable=True):
        """
        創建 16-bit SPI 數據
        格式: [enable(1bit)][DIR(1bit)][空(2bit)][p(12bit)]
        
        :param p: 週期參數 (12 bits)
        :param direction: 方向 (1=正轉, -1=反轉)
        :param enable: 是否啟用馬達
        :return: (byte1, byte2) 兩個 8-bit 數據
        """
        # 構建 16-bit 數據
        enable_bit = 1 if enable else 0
        dir_bit = 1 if direction > 0 else 0
        
        # 第一個 byte: [enable][DIR][0][0][p11][p10][p9][p8]
        byte1 = (enable_bit << 7) | (dir_bit << 6) | ((p >> 8) & 0x0F)
        
        # 第二個 byte: [p7][p6][p5][p4][p3][p2][p1][p0]
        byte2 = p & 0xFF
        
        return byte1, byte2
    
    def spi_transfer(self, cs_name, data):
        """
        執行 SPI 傳輸
        :param cs_name: Chip select 名稱 ('CS0', 'CS1', 或 'CS2')
        :param data: 要傳輸的數據列表
        :return: 接收到的數據
        """
        if cs_name not in CS_PINS:
            raise ValueError(f"Invalid CS name: {cs_name}")
        
        pin = CS_PINS[cs_name]
        
        try:
            # 啟動 chip select (拉低)
            GPIO.output(pin, GPIO.LOW)
            time.sleep(0.0001)  # 短暫延遲
            
            # 執行 SPI 傳輸
            response = self.spi.xfer2(data)
            
            # 停用 chip select (拉高)
            GPIO.output(pin, GPIO.HIGH)
            
            return response
        except Exception as e:
            GPIO.output(pin, GPIO.HIGH)  # 確保 CS 回到高位
            raise RuntimeError(f"SPI 傳輸失敗 ({cs_name}): {e}")
    
    def format_spi_binary(self, byte1, byte2):
        """格式化 SPI 數據為二進制字串"""
        return f"{byte1:08b} {byte2:08b}"

    def calculate_move_angle_deg(self, vx, vy, min_speed=0.02):
        """根據平移向量計算行進角度 (0~360 度)"""
        speed = (vx * vx + vy * vy) ** 0.5
        if speed < min_speed:
            return 0.0
        angle = math.degrees(math.atan2(vy, vx))
        return (angle + 360.0) % 360.0

    def create_angle_spi_data(self, angle_deg):
        """把 0~360 度角度編碼成 16-bit 無號整數並拆成兩個 byte"""
        angle_clamped = max(0.0, min(angle_deg, 360.0))
        angle_u16 = int(round((angle_clamped / 360.0) * 65535.0))
        byte1 = (angle_u16 >> 8) & 0xFF
        byte2 = angle_u16 & 0xFF
        return byte1, byte2, angle_u16
    
    def run(self):
        self.controller.wait_for_connection()
        print("[啟動] SPI 控制器啟動！ (按 Ctrl+C 或長按X鍵離開)")
        print(
            f"CS Pins: CS0={CS_PINS['CS0']}, CS1={CS_PINS['CS1']}, "
            f"CS2={CS_PINS['CS2']}, CS3={CS_PINS['CS3']}(角度)\n"
        )
        
        # 啟動震動提示
        self.controller.rumble(0.8, 0.8, 300)
        time.sleep(1)
        
        try:
            while True:
                try:
                    # 讀取手把
                    data = self.controller.read()
                    lx, ly = data['left_stick']
                    lt = data['left_trigger']
                    rt = data['right_trigger']
                    lx = self.controller.apply_deadzone(lx)
                    ly = self.controller.apply_deadzone(ly)
                    # 扳機死區處理
                    lt = apply_trigger_deadzone(lt)
                    rt = apply_trigger_deadzone(rt)
                    
                    # 檢測按鈕狀態
                    buttons = data['buttons']
                    
                    # 檢測A鍵煞車
                    self.braking = len(buttons) > BUTTON_A and buttons[BUTTON_A]
                    
                    # 檢測X鍵長按
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
                    
                    # 處理控制輸入
                    r = self.process_control_input(lx, ly, lt, rt)
                    w = r['wheels']

                    # 角度輸出（由平移向量計算）
                    self.move_angle_deg = self.calculate_move_angle_deg(r['vx_pct'], r['vy_pct'])
                    angle_b1, angle_b2, angle_u16 = self.create_angle_spi_data(self.move_angle_deg)
                    self.spi_transfer('CS3', [angle_b1, angle_b2])
                    
                    # 創建並傳送 SPI 數據
                    spi_data = []
                    for i, (rps, p, d) in enumerate([
                        (w['rps1'], w['p1'], w['dir1']),
                        (w['rps2'], w['p2'], w['dir2']),
                        (w['rps3'], w['p3'], w['dir3'])
                    ], 1):
                        # 煞車模式：強制 EN=0
                        if self.braking:
                            enable = False
                        else:
                            enable = abs(rps) > 0.01
                        # d 已經是方向 (1 or -1)
                        b1, b2 = self.create_spi_data(p, d, enable)
                        self.spi_transfer(f'CS{i-1}', [b1, b2])
                        spi_data.append((b1, b2, enable, d > 0, p))
                    
                    # 顯示輸出
                    sys.stdout.write("\033[H\033[J")
                    brake_status = "\n[BRAKE]" if self.braking else ""
                    if not self.verbose:
                        print(f"=== SPI 數據輸出 === {brake_status}")
                        for i, (b1, b2, en, dir_pos, p) in enumerate(spi_data, 1):
                            print(f"輪{i}: {self.format_spi_binary(b1, b2)} | EN={int(en)} DIR={int(dir_pos)} p={p:4d}")
                        print(
                            f"角度: {self.move_angle_deg:6.2f}° | "
                            f"{self.format_spi_binary(angle_b1, angle_b2)} "
                            f"(0x{angle_u16:04X})"
                        )
                        print(f"\n目標輸入: X={r['target_vx_pct']:+.2f}, Y={r['target_vy_pct']:+.2f}, W={r['target_wz_pct']:+.2f}")
                        print(f"平滑輸入: X={r['vx_pct']:+.2f}, Y={r['vy_pct']:+.2f}, W={r['wz_pct']:+.2f}")
                    else:
                        print(f"[控制器] 三輪全向輪機器人控制器 - SPI 輸出模式 {brake_status}")
                        print("=" * 70)
                        print(f"目標輸入: X={r['target_vx_pct']:+.2f}, Y={r['target_vy_pct']:+.2f}, W={r['target_wz_pct']:+.2f}")
                        print(f"平滑輸入: X={r['vx_pct']:+.2f}, Y={r['vy_pct']:+.2f}, W={r['wz_pct']:+.2f}")
                        print("-" * 70)
                        wheel_names = ['輪1(前)', '輪2(左後)', '輪3(右後)']
                        for i, name in enumerate(wheel_names):
                            rps_val = w[f'rps{i+1}']
                            p_val = w[f'p{i+1}']
                            b1, b2, en, dir_pos, _ = spi_data[i]
                            print(f"{name}:   RPS={abs(rps_val):.3f}  p={p_val:4d}")
                            print(f"  SPI: {self.format_spi_binary(b1, b2)}")
                            print(f"  → EN={int(en)} DIR={int(dir_pos)} (0x{b1:02X}{b2:02X})")
                        print(
                            f"角度通道(CS3): {self.move_angle_deg:6.2f}° | "
                            f"{self.format_spi_binary(angle_b1, angle_b2)} "
                            f"(0x{angle_u16:04X})"
                        )
                        print("=" * 70)
                    
                    time.sleep(UPDATE_RATE)
                
                except Exception as e:
                    print(f"\n錯誤: {e}")
                    time.sleep(0.5)
                    
        except KeyboardInterrupt:
            print("\n\n[結束] 控制器已停止")
        finally:
            # 停止震動提示
            try:
                self.controller.rumble(1.0, 1.0, 150)
            except:
                pass
            self.cleanup()
    
    def cleanup(self):
        """清理 GPIO 和 SPI 資源"""
        print("\n正在清理資源...")
        
        # 先停止所有馬達 (EN=0)
        try:
            b1, b2 = self.create_spi_data(0, 0, enable=False)
            for i in range(3):
                self.spi_transfer(f'CS{i}', [b1, b2])
            print("✓ 已停止所有馬達 (EN=0)")
        except Exception as e:
            print(f"⚠ 停止馬達失敗: {e}")

        # 清除角度輸出（寫入 0）
        try:
            angle_b1, angle_b2, _ = self.create_angle_spi_data(0.0)
            self.spi_transfer('CS3', [angle_b1, angle_b2])
            print("✓ 已清除角度輸出 (CS3=0)")
        except Exception as e:
            print(f"⚠ 清除角度輸出失敗: {e}")
        
        try:
            if hasattr(self, 'spi'):
                self.spi.close()
                print("✓ SPI 已關閉")
        except Exception as e:
            print(f"⚠ SPI 關閉失敗: {e}")
        
        try:
            GPIO.cleanup()
            print("✓ GPIO 已清理")
        except Exception as e:
            print(f"⚠ GPIO 清理失敗: {e}")

if __name__ == "__main__":
    # 預設 verbose=False，可通過命令列參數啟用
    verbose = "--verbose" in sys.argv or "-v" in sys.argv
    
    robot = OmniRobotSPIController(verbose=verbose)
    robot.run()
