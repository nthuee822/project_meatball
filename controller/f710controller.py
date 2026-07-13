"""F710手把控制器模組 - 負責讀取手把並轉換為機器人控制指令"""
import pygame
import os

os.environ["SDL_VIDEODRIVER"] = "dummy"

class F710Controller:
    def __init__(self, deadzone=0.1):
        try:
            pygame.init()
            pygame.joystick.init()
        except Exception as e:
            raise RuntimeError(f"Pygame 初始化失敗: {e}")
        
        self.joy = None
        self.deadzone = max(0.0, min(1.0, deadzone))  # 限制在 [0, 1]
        self._initialized = True
    
    def __del__(self):
        """清理 pygame 資源"""
        if getattr(self, '_initialized', False):
            try:
                pygame.joystick.quit()
                pygame.quit()
            except Exception:
                pass
    
    def apply_deadzone(self, value):
        """過濾微小漂移"""
        return 0.0 if abs(value) < self.deadzone else value
        
    def connect(self):
        """嘗試連接手把"""
        try:
            if pygame.joystick.get_count() == 0:
                return False
            self.joy = pygame.joystick.Joystick(0)
            self.joy.init()
            return True
        except Exception as e:
            print(f"手把連接錯誤: {e}")
            return False
    
    def wait_for_connection(self):
        """等待手把連接"""
        import sys, time
        print("[搜尋] 正在搜尋 Logitech F710...")
        attempts = 0
        max_attempts = 60  # 最多等待 60 秒
        
        while not self.connect():
            if attempts >= max_attempts:
                raise TimeoutError("等待手把連接超時（60秒）")
            
            sys.stdout.write("\r[等待] 未偵測到手把，請檢查接收器與電池...")
            sys.stdout.flush()
            time.sleep(1)
            attempts += 1
            pygame.joystick.quit()
            pygame.joystick.init()
        
        print(f"\n[成功] 成功連接: {self.joy.get_name()}")
    
    def rumble(self, low_freq=0.5, high_freq=0.5, duration_ms=200):
        """
        震動手把
        :param low_freq: 低頻馬達強度 (0.0-1.0)
        :param high_freq: 高頻馬達強度 (0.0-1.0)
        :param duration_ms: 震動持續時間 (毫秒)
        """
        if self.joy and hasattr(self.joy, 'rumble'):
            try:
                self.joy.rumble(low_freq, high_freq, duration_ms)
            except Exception as e:
                # 某些手把可能不支援震動
                pass
    
    def read(self):
        """
        讀取手把狀態
        :return: dict 包含所有輸入數據
        """
        if self.joy is None:
            raise RuntimeError("手把未連接，請先呼叫 wait_for_connection()")
        
        try:
            pygame.event.pump()
            
            # 讀取軸（確保至少有6個軸）
            num_axes = self.joy.get_numaxes()
            axes = [self.joy.get_axis(i) for i in range(min(num_axes, 6))]
            # 填充不足的軸
            axes.extend([0.0] * (6 - len(axes)))
            
            # 讀取按鈕
            num_btns = self.joy.get_numbuttons()
            buttons = [self.joy.get_button(i) for i in range(num_btns)]
            
            # 讀取十字鍵
            hat = (0, 0)
            if self.joy.get_numhats() > 0:
                hat = self.joy.get_hat(0)
            
            return {
                'left_stick': (axes[0], axes[1]),
                'right_stick': (axes[3], axes[4]),
                'left_trigger': axes[2],
                'right_trigger': axes[5],
                'dpad': hat,
                'buttons': buttons
            }
        except Exception as e:
            raise RuntimeError(f"讀取手把數據失敗: {e}")
