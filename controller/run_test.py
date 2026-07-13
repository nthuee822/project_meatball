"""快速測試腳本"""
from inverse_kinematics import OmniKinematics

# 測試 OmniKinematics
print("=== 測試 OmniKinematics ===")

# 測試 1: 正常初始化
k = OmniKinematics()
print("[PASS] 正常初始化")

# 測試 2: 無效模式應該拋出錯誤
try:
    k_bad = OmniKinematics(mode='invalid')
    print("[FAIL] 無效模式未拋出錯誤")
except ValueError as e:
    print(f"[PASS] 無效模式拋出錯誤: {e}")

# 測試 3: calculate 計算
v = k.calculate(1.0, 0.5, 0.2)
print(f"[PASS] calculate(1.0, 0.5, 0.2) = {v}")

# 測試 4: normalize 正規化
v_norm = k.normalize(*v)
print(f"[PASS] normalize = {v_norm}")

# 測試 5: 驗證數學公式
v1, v2, v3 = k.calculate(0, 1.0, 0)  # 純前進
assert abs(v1 - 1.0) < 0.001 and abs(v2 + 0.5) < 0.001 and abs(v3 + 0.5) < 0.001
print("[PASS] 前進公式驗證正確")

print("\n=== 所有測試通過 ===")
