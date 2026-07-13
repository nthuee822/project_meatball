"""三輪全向輪機器人主程式 - SPI 輸出版本 (抗機構干涉版)"""
import sys
import time
try:
    import spidev
    import RPi.GPIO as GPIO
except ImportError:
    print("警告: 缺少 spidev 或 RPi.GPIO，請確保在 Raspberry Pi 上執行")
    sys.exit(1)

from omni_robot import OmniRobotController, MOTOR_MAX_RPS, UPDATE_RATE

# 扳機死區設定
TRIGGER_DEADZONE = 0.1  

# 固定偏心力矩補償常數 (Fixed Yaw Compensation)
FIXED_YAW_COMP = -0.1
# 補償值的最大加速度 (RPS/s)，數值越小越平滑
COMP_ACCEL_LIMIT = 1.0

def apply_trigger_deadzone(trigger_value, deadzone=TRIGGER_DEADZONE):
    normalized = (trigger_value + 1.0) / 2.0  
    if normalized < deadzone:
        return -1.0
    return (normalized - deadzone) / (1.0 - deadzone) * 2.0 - 1.0

CS_PINS = {
    'CS0': 17,  # 輪1 (前)
    'CS1': 27,  # 輪2 (左後)
    'CS2': 22   # 輪3 (右後)
}

BUTTON_A = 0  
BUTTON_X = 2  
LONG_PRESS_DURATION = 1.5  

class OmniRobotSPIController(OmniRobotController):
    
    def __init__(self, motor_max_rps=MOTOR_MAX_RPS, verbose=False):
        super().__init__(motor_max_rps=motor_max_rps)
        self.verbose = verbose
        self.braking = False
        self.x_pressed = False
        self.x_press_start_time = 0
        self.current_comp_rps = 0.0
        try:
            self._init_gpio_spi()
        except Exception as e:
            print(f"錯誤: GPIO/SPI 初始化失敗: {e}")
            raise
    
    def _init_gpio_spi(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for name, pin in CS_PINS.items():
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH)
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 500000
        self.spi.mode = 0
    
    def rps_to_motor_params(self, rps):
        direction = 1 if rps >= 0 else -1
        rps_abs = abs(rps)
        if rps_abs < 0.01:
            return 0.0, 0, 0
        p = int(self.motor_k / rps_abs)
        p = max(1, min(p, 4095))
        return rps_abs, p, direction
    
    def create_spi_data(self, p, direction, enable=True):
        enable_bit = 1 if enable else 0
        dir_bit = 1 if direction > 0 else 0
        byte1 = (enable_bit << 7) | (dir_bit << 6) | ((p >> 8) & 0x0F)
        byte2 = p & 0xFF
        return byte1, byte2
    
    def spi_transfer(self, cs_name, data):
        pin = CS_PINS[cs_name]
        try:
            GPIO.output(pin, GPIO.LOW)
            time.sleep(0.0001)  
            response = self.spi.xfer2(data)
            GPIO.output(pin, GPIO.HIGH)
            return response
        except Exception as e:
            GPIO.output(pin, GPIO.HIGH)
            raise RuntimeError(f"SPI 傳輸失敗 ({cs_name}): {e}")
    
    def format_spi_binary(self, byte1, byte2):
        return f"{byte1:08b} {byte2:08b}"
    
    def run(self):
        self.controller.wait_for_connection()
        print("[啟動] SPI 控制器啟動！ (按 Ctrl+C 或長按X鍵離開)")
        self.controller.rumble(0.8, 0.8, 300)
        time.sleep(1)
        
        try:
            while True:
                try:
                    data = self.controller.read()
                    lt = apply_trigger_deadzone(data['left_trigger'])
                    rt = apply_trigger_deadzone(data['right_trigger'])

                    # 控制邏輯: 左右扳機控制 X 方向
                    lt_linear = (lt + 1.0) / 2.0
                    rt_linear = (rt + 1.0) / 2.0
                    input_x = rt_linear - lt_linear
                    input_y = 0.0
                    
                    buttons = data['buttons']
                    self.braking = len(buttons) > BUTTON_A and buttons[BUTTON_A]
                    
                    if len(buttons) > BUTTON_X and buttons[BUTTON_X]:
                        if not self.x_pressed:
                            self.x_pressed = True
                            self.x_press_start_time = time.time()
                        elif time.time() - self.x_press_start_time >= LONG_PRESS_DURATION:
                            break
                    else:
                        self.x_pressed = False
                    
                    # 獲取基礎輪速
                    r = self.process_control_input(input_x, input_y, -1.0, -1.0)
                    w = r['wheels']

                    # --- [修改區塊開始] 抗干涉平滑與等比例縮放邏輯 ---
                    target_comp_rps = 0.0
                    input_mag = abs(input_x)

                    # 1) 計算目標補償值 (加入漸進過渡，避免低速補償過大互相拉扯)
                    if input_mag > 0.05:
                        direction_sign = 1.0 if input_x > 0 else -1.0
                        # 搖桿輸入在 0.05 ~ 0.5 之間時，補償值漸增；超過 0.5 則給足 FIXED_YAW_COMP
                        comp_ratio = min((input_mag - 0.05) / 0.45, 1.0)
                        target_comp_rps = FIXED_YAW_COMP * direction_sign * comp_ratio

                    # 2) 對補償值做斜率限制，避免瞬間跳變
                    max_delta = COMP_ACCEL_LIMIT * UPDATE_RATE
                    if self.current_comp_rps < target_comp_rps:
                        self.current_comp_rps = min(self.current_comp_rps + max_delta, target_comp_rps)
                    elif self.current_comp_rps > target_comp_rps:
                        self.current_comp_rps = max(self.current_comp_rps - max_delta, target_comp_rps)

                    # 3) 將平滑後補償值疊加至三顆輪子，並找出當下最大轉速
                    max_rps_current = 0.0
                    for i in range(1, 4):
                        w[f'rps{i}'] += self.current_comp_rps
                        max_rps_current = max(max_rps_current, abs(w[f'rps{i}']))

                    # 4) 等比例縮放防護 (Normalization): 若總和超過馬達速限，依比例降速，維持向量不變形
                    scale = 1.0
                    if max_rps_current > self.motor_max_rps:
                        scale = self.motor_max_rps / max_rps_current

                    # 5) 轉換為 SPI 馬達參數
                    for i in range(1, 4):
                        w[f'rps{i}'] *= scale
                        _, w[f'p{i}'], w[f'dir{i}'] = self.rps_to_motor_params(w[f'rps{i}'])
                    # --- [修改區塊結束] ---
                    
                    spi_data = []
                    for i, (rps, p, d) in enumerate([
                        (w['rps1'], w['p1'], w['dir1']),
                        (w['rps2'], w['p2'], w['dir2']),
                        (w['rps3'], w['p3'], w['dir3'])
                    ], 1):
                        enable = False if self.braking else (abs(rps) > 0.01)
                        b1, b2 = self.create_spi_data(p, d, enable)
                        self.spi_transfer(f'CS{i-1}', [b1, b2])
                        spi_data.append((b1, b2, enable, d > 0, p))
                    
                    sys.stdout.write("\033[H\033[J")
                    if not self.verbose:
                        print(f"=== 實測數據輸出 ===")
                        print(f"X軸輸入: {input_x:+.2f} | 補償 RPS 目標/當前: {target_comp_rps:+.3f}/{self.current_comp_rps:+.3f}")
                        if scale < 1.0:
                            print(f"[警告] 超速防護啟動！轉速等比例縮小為 {scale*100:.1f}%，避免輪胎互相抵抗失步。")
                        for i, (b1, b2, en, dir_pos, p) in enumerate(spi_data, 1):
                            print(f"輪{i}: EN={int(en)} DIR={int(dir_pos)} p={p:4d} (實速: {w[f'rps{i}']:+.2f})")
                    
                    time.sleep(UPDATE_RATE)
                
                except Exception as e:
                    time.sleep(0.5)
                    
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()
    
    def cleanup(self):
        try:
            b1, b2 = self.create_spi_data(0, 0, enable=False)
            for i in range(3):
                self.spi_transfer(f'CS{i}', [b1, b2])
        except: pass
        try: self.spi.close()
        except: pass
        try: GPIO.cleanup()
        except: pass

if __name__ == "__main__":
    verbose = "--verbose" in sys.argv or "-v" in sys.argv
    robot = OmniRobotSPIController(verbose=verbose)
    robot.run()