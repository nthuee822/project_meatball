"""
三輪全向輪逆運動學模組
輸入: 機器人期望速度比例 (vx, vy)
輸出: 三輪轉速比例 (v1, v2, v3)
"""
import numpy as np

class OmniKinematics:
    SUPPORTED_MODES = ('radial',)
    
    def __init__(self, mode='radial'):
        """
        :param mode: 'radial' 徑向安裝（無旋轉）
        """
        if mode not in self.SUPPORTED_MODES:
            raise ValueError(f"不支援的模式 '{mode}'，僅支援: {self.SUPPORTED_MODES}")
        self.mode = mode
        self.sqrt3_over_2 = np.sqrt(3) / 2  # ≈ 0.866025
        
    def calculate(self, vx, vy, wz=0):
        """
        三輪全向輪逆運動學計算 - 徑向安裝模式
        
        :param vx: X軸速度比例 (正值=右移)
        :param vy: Y軸速度比例 (正值=前移)
        :param wz: 自轉速度比例
        :return: (v1, v2, v3) 各輪速度比例
        """
        v1 = vy + wz
        # 修正：反轉 vx 符號以修正左右移動方向相反的問題
        # v2 (左後) 原本是 +vx 改為 -vx
        v2 = -0.5 * vy - self.sqrt3_over_2 * vx + wz
        # v3 (右後) 原本是 -vx 改為 +vx
        v3 = -0.5 * vy + self.sqrt3_over_2 * vx + wz
        
        return v1, v2, v3
    
    def normalize(self, v1, v2, v3, max_speed=1.0):
        """
        等比例縮放，避免單輪此例超過上限導致路徑偏移
        保持移動方向不變
        
        :param v1, v2, v3: 原始計算值
        :param max_speed: 最大允許比例 (通常為 1.0)
        :return: (v1_norm, v2_norm, v3_norm)
        """
        max_v = max(abs(v1), abs(v2), abs(v3))
        
        if max_v > max_speed:
            scale = max_speed / max_v
            return v1 * scale, v2 * scale, v3 * scale
        
        return v1, v2, v3
