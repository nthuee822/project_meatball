"""三輪全向輪機器人主程式 - 整合版本"""
import sys
import time
import numpy as np
try:
    from f710controller import F710Controller
    from inverse_kinematics import OmniKinematics
except ImportError as e:
    print(f"錯誤: 無法導入必要模組: {e}")
    sys.exit(1)

# ==================== 參數配置區 ====================
CONTROLLER_DEADZONE = 0.1   # 搖桿死區
BAR_LENGTH = 10             # 能量條長度
UPDATE_RATE = 0.05          # 更新週期 (秒) = 20Hz

# 馬達控制參數
MOTOR_K = 10000.0 / 64.0    # 馬達常數: RPS = K/p
MOTOR_MAX_RPS = 1.5         # 馬達安全轉速上限 (rotation per sec)
MOTOR_MIN_RPS = 0.0         # 最小輸出閾值 (低於此值停止馬達) - 設為0則不啟用

# 平滑控制參數
ACCEL_RATE_UP = 0.5            # 加速度限制：每秒允許變化的百分比 (1.0 = 100%/s)
ACCEL_RATE_DOWN = 5.0            # 加速度限制：每秒允許變化的百分比 (1.0 = 100%/s)
# =====================================================

class OmniRobotController:
    def __init__(self, motor_max_rps=MOTOR_MAX_RPS):
        if motor_max_rps <= 0:
            raise ValueError("最大轉速必須大於 0")
        
        self.controller = F710Controller(deadzone=CONTROLLER_DEADZONE)
        self.kinematics = OmniKinematics(mode='radial')
        self.bar_length = BAR_LENGTH
        self.motor_max_rps = motor_max_rps
        self.motor_k = MOTOR_K
        
        # 平滑控制狀態 (百分比 -1.0 ~ 1.0)
        self.current_lx = 0.0
        self.current_ly = 0.0
        self.current_wz = 0.0
        self.accel_rate_up = ACCEL_RATE_UP      # 加速時的加速度上限
        self.accel_rate_down = ACCEL_RATE_DOWN  # 減速時的加速度上限
    
    def slew_rate_limit(self, target, current, max_change_up, max_change_down):
        """限制速度變化率（平滑處理），加速和減速分開限制"""
        delta = target - current
        # 判斷是加速還是減速：速度絕對值增加為加速，減少為減速
        is_accelerating = abs(target) > abs(current)
        max_change = max_change_up if is_accelerating else max_change_down
        
        if abs(delta) <= max_change:
            return target
        return current + (max_change if delta > 0 else -max_change)
    
    def rps_to_motor_params(self, rps):
        """將轉速(RPS)轉換為馬達控制參數"""
        direction = 1 if rps >= 0 else -1
        rps_abs = abs(rps)
        
        # 低於閾值停止
        if rps_abs < max(MOTOR_MIN_RPS, 0.001):
            return 0.0, 9999, 0
        
        # RPS → p (RPS = K/p)
        p = int(self.motor_k / rps_abs)
        p = max(1, min(p, 9999))
        
        return rps_abs, p, direction
    
    def draw_bar(self, val, length=None):
        """雙向能量條顯示 (中間為0)"""
        if length is None:
            length = self.bar_length
        half = length // 2
        val_clamped = max(-1, min(1, val))
        
        if val_clamped >= 0:
            # 正值: 從中間往右填充
            fill = int(val_clamped * half)
            bar = "░" * half + "▓" * fill + "░" * (half - fill)
        else:
            # 負值: 從中間往左填充
            fill = int(-val_clamped * half)
            bar = "░" * (half - fill) + "▓" * fill + "░" * half
        
        return f"[{bar}] {val:+.2f}"
    
    def process_control_input(self, lx, ly, lt=-1.0, rt=-1.0):
        """
        處理控制輸入：平滑輸入 -> 逆運動學求比例 -> 換算轉速
        包含左右扳機控制的自轉計算
        :param lt: 左扳機 (-1.0 ~ 1.0)
        :param rt: 右扳機 (-1.0 ~ 1.0)
        """
        # 0. 自轉輸入處理
        # 將扳機輸入 (-1~1) 轉換為 (0~1)
        lt_val = (lt + 1.0) / 2.0
        rt_val = (rt + 1.0) / 2.0
        # 右扳機正轉，左扳機反轉，三輪疊加
        target_wz = rt_val - lt_val

        # 1. 輸入平滑化 (針對百分比)
        max_delta_up = self.accel_rate_up * UPDATE_RATE
        max_delta_down = self.accel_rate_down * UPDATE_RATE
        vx_pct = self.slew_rate_limit(lx, self.current_lx, max_delta_up, max_delta_down)
        vy_pct = self.slew_rate_limit(-ly, self.current_ly, max_delta_up, max_delta_down)
        wz_pct = self.slew_rate_limit(target_wz, self.current_wz, max_delta_up, max_delta_down)
        
        self.current_lx = vx_pct
        self.current_ly = vy_pct
        self.current_wz = wz_pct
        
        # 2. 逆運動學計算 (輸入百分比 -> 輸出各輪百分比)
        # 傳入 wz 參數
        v_raw = self.kinematics.calculate(vx_pct, vy_pct, wz_pct)
        # 歸一化: 確保沒有輪子超過 100%
        v_norm = self.kinematics.normalize(*v_raw, max_speed=1.0)
        
        # 3. 計算各輪參數
        wheels = {}
        # v_norm tuple 順序對應: v1(前), v2(左後), v3(右後)
        for i, v_pct in enumerate(v_norm, 1):
            # 百分比 * 最大RPS = 目標RPS
            target_rps = v_pct * self.motor_max_rps
            rps_abs, p, direction = self.rps_to_motor_params(target_rps)
            
            wheels[f'rps{i}'] = rps_abs * direction  # 帶符號RPS
            wheels[f'p{i}'] = p
            wheels[f'dir{i}'] = direction

        return {
            'target_vx_pct': lx, 'target_vy_pct': -ly, 'target_wz_pct': target_wz,
            'vx_pct': vx_pct, 'vy_pct': vy_pct, 'wz_pct': wz_pct,
            'wheels': wheels
        }
    
    def run(self):
        try:
            self.controller.wait_for_connection()
        except Exception as e:
            print(f"錯誤: 手把連接失敗: {e}")
            return
        
        print("[啟動] 儀表板啟動！ (按 Ctrl+C 離開)\n")
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
                    # 扳機無需死區
                    
                    # 處理控制輸入
                    r = self.process_control_input(lx, ly, lt, rt)
                    w = r['wheels']
                    
                    # 清屏並顯示
                    sys.stdout.write("\033[H\033[J")
                    print("[控制器] 三輪全向輪機器人控制器 (徑向模式 + 自轉)")
                    print("=" * 60)
                    print(f"[目標] 輸入: X={r['target_vx_pct']:+.2f}, Y={r['target_vy_pct']:+.2f}, W={r['target_wz_pct']:+.2f}")
                    print(f"[平滑] 輸入: X={r['vx_pct']:+.2f}, Y={r['vy_pct']:+.2f}, W={r['wz_pct']:+.2f}")
                    print("-" * 60)
                    print(f"  輪1(前):   {self.draw_bar(w['rps1']/self.motor_max_rps)}")
                    print(f"             → RPS={abs(w['rps1']):.3f}  p={w['p1']:4d}  方向={'正轉' if w['dir1']>0 else '反轉'}")
                    print(f"  輪2(左後): {self.draw_bar(w['rps2']/self.motor_max_rps)}")
                    print(f"             → RPS={abs(w['rps2']):.3f}  p={w['p2']:4d}  方向={'正轉' if w['dir2']>0 else '反轉'}")
                    print(f"  輪3(右後): {self.draw_bar(w['rps3']/self.motor_max_rps)}")
                    print(f"             → RPS={abs(w['rps3']):.3f}  p={w['p3']:4d}  方向={'正轉' if w['dir3']>0 else '反轉'}")
                    print("=" * 60)
                    print(f"【左搖桿平移】 X: {self.draw_bar(lx)}  Y: {self.draw_bar(ly)}")
                    print(f"【扳機自轉】   L: {lt:.2f}  R: {rt:.2f}  →  Net: {self.draw_bar(r['target_wz_pct'])}")
                    print("-" * 50)
                    
                    time.sleep(UPDATE_RATE)
                
                except Exception as e:
                    print(f"\n錯誤: {e}")
                    time.sleep(0.5)
                    
        except KeyboardInterrupt:
            print("\n\n[結束] 控制器已停止")

if __name__ == "__main__":
    robot = OmniRobotController(motor_max_rps=MOTOR_MAX_RPS)
    robot.run()
