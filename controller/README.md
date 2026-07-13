# 三輪全向輪機器人控制器

模組化的三輪全向輪機器人控制系統（**徑向安裝模式**），支援Logitech F710手把。

## 系統特性

- 徑向驅動 - 輪子朝圓心方向滾動
- 純平移移動 - 支援前後左右全方向平移
- 可控自轉 - 利用線性扳機控制自轉 (需依賴輪子安裝偏差)
- 平滑控制 - 基於百分比的加速度限制
- SPI 輸出 - 支援樹莓派 SPI 介面控制馬達
- 自轉原理 - 利用三輪同向同速旋轉與製造誤差產生整體旋轉

## 檔案結構

```
controller/
├── inverse_kinematics.py  # 逆運動學計算（徑向模式）
├── f710controller.py      # F710手把讀取模組
├── omni_robot.py          # 主程式（顯示模式）
├── controller2spi.py      # SPI輸出版本
├── test.py                # 按鈕測試工具
├── run_test.py            # 快速測試腳本
└── README.md              # 本文檔
```

## 快速開始

### 環境需求

**基本版本（顯示模式）:**
```bash
pip install pygame numpy
```

**SPI 版本（需在 Raspberry Pi 上執行）:**
```bash
pip install pygame numpy spidev RPi.GPIO
```

### 運行機器人控制器（顯示模式）
```bash
python omni_robot.py
```

### 運行 SPI 輸出版本
```bash
# 簡潔模式（預設）
python controller2spi.py

# 詳細模式
python controller2spi.py --verbose
```

### 測試手把按鈕
```bash
python test.py
```

## 控制方式

### 主要控制

- **左搖桿 (Left Stick)**
  - X軸: 控制左右平移（正值=右移，負值=左移）
  - Y軸: 控制前後平移（正值=後退，負值=前進）
  - 死區設定: 0.1（可在 `CONTROLLER_DEADZONE` 調整）

- **左右扳機 (L2/R2 Triggers)**
  - 右扳機 (R2 / RT): 正向旋轉（疊加正速度到三輪）
  - 左扳機 (L2 / LT): 反向旋轉（疊加負速度到三輪）
  - 死區設定: 0.1（可在 `TRIGGER_DEADZONE` 調整）
  - 注意: 需依賴輪子安裝偏差才能產生實際旋轉效果

### 按鈕功能 (SPI 版本)

- **A 按鈕**: 緊急煞車
  - 按住時強制停止所有馬達（EN=0）
  - 鬆開後恢復正常控制
  
- **X 按鈕**: 安全退出
  - 長按 1.5 秒觸發退出程序
  - 退出時會震動兩次作為確認
  - 自動停止所有馬達並清理資源

### 未使用的控制

- **右搖桿 (Right Stick)**: 未使用
- **B, Y 按鈕**: 未使用
- **LB, RB (肩鍵)**: 未使用
- **十字鍵 (D-Pad)**: 未使用
- **Start, Back 按鈕**: 未使用

### 震動反饋

- 連接成功時: 短震動提示
- 退出程序時: 兩次短震動確認

## 逆運動學公式（徑向模式）

輸入為 -1.0 ~ 1.0 的比例值 (含自轉 w)：
```python
v1 = vy + w                      # 1號輪(前)
v2 = -0.5*vy - 0.866*vx + w     # 2號輪(左後) - 修正左右方向
v3 = -0.5*vy + 0.866*vx + w     # 3號輪(右後) - 修正左右方向
```

## 參數配置

關鍵參數位於 `omni_robot.py` 頂部：

```python
CONTROLLER_DEADZONE = 0.1   # 搖桿死區
MOTOR_MAX_RPS = 1.5         # 馬達安全轉速上限
ACCEL_RATE_UP = 0.5         # 加速時限制 (每秒變化50%)
ACCEL_RATE_DOWN = 5.0       # 減速時限制 (每秒變化500%)
MOTOR_MIN_RPS = 0.0         # 最小輸出閾值
TRIGGER_DEADZONE = 0.1      # 扳機死區
```

---

## SPI 輸出版本說明

### 程式架構
`controller2spi.py` 繼承自 `omni_robot.py` 的 `OmniRobotController`，添加 SPI 數據輸出功能。

### SPI 數據格式（16 bits）
```
位元分配: [EN][DIR][0][0][p11-p0]
- Bit 15: Enable (啟用馬達)
- Bit 14: Direction (旋轉方向: 1=正轉, 0=反轉)
- Bit 13-12: 保留 (固定為 0)
- Bit 11-0: 週期參數 p (12 bits, 範圍 0-4095)
  * p=0: 停止馬達
  * p=1-4095: RPS = 10000/64/p
```

### GPIO 配置
- CS0 (GPIO 17): 輪1 (前)
- CS1 (GPIO 27): 輪2 (左後)
- CS2 (GPIO 22): 輪3 (右後)

### SPI 輸出範例

#### 簡潔模式 (預設)
```
=== SPI 數據輸出 ===
輪1: 10000000 00000000 | EN=1 DIR=0 p=   0
輪2: 11001010 00110101 | EN=1 DIR=1 p=2613
輪3: 10010101 11001100 | EN=1 DIR=0 p=1484

目標輸入: X=+1.00, Y=-0.50
平滑輸入: X=+0.95, Y=-0.48
```

#### 詳細模式 (--verbose)
顯示完整的速度、RPS、SPI 二進制和十六進制數據。

---

## 在其他專案中使用

```python
from f710controller import F710Controller
from inverse_kinematics import OmniKinematics

# 初始化
controller = F710Controller(deadzone=0.1)
kinematics = OmniKinematics(mode='radial')

# 讀取速度指令（僅平移）
controller.wait_for_connection()
data = controller.read()
lx, ly = data['left_stick']

# 計算輪速比例
vx_pct = lx 
vy_pct = -ly
v1, v2, v3 = kinematics.calculate(vx_pct, vy_pct)
v1, v2, v3 = kinematics.normalize(v1, v2, v3, max_speed=1.0)
```

## 驗證測試

運行測試：
```bash
python run_test.py
```

預期結果：
- 直直往前 (vy=+1.0, vx=0): v1=+1.0, v2=-0.5, v3=-0.5
- 直直往右 (vx=+1.0, vy=0): v1=0, v2=+0.866, v3=-0.866
- 斜向移動 (vx=0.5, vy=0.5): v1=+0.5, v2=+0.183, v3=-0.683

## 安全性與錯誤處理

### 輸入驗證
- 輪子半徑必須 > 0
- 最大轉速必須 > 0
- 搖桿死區限制在 [0, 1]

### 資源管理
- 程式退出時自動清理 GPIO 和 SPI 資源
- Ctrl+C 安全中斷
- SPI 版本支援 X 鍵長按 1.5 秒退出

### 錯誤處理
- SPI 傳輸失敗時會拋出詳細錯誤訊息
- 手把連線超時（60秒）保護
- 模組導入失敗時顯示友善提示

## 疑難排解

### 手把無法連接
```bash
# 檢查 USB 接收器
lsusb | grep Logitech

# 測試手把
python test.py
```

### SPI 傳輸失敗
```bash
# 檢查 SPI 是否啟用
ls /dev/spidev*

# 啟用 SPI（需重啟）
sudo raspi-config
# Interface Options -> SPI -> Enable
```

### 馬達不轉
- 檢查 `MOTOR_MIN_RPS` 設定（設為 0 可關閉閾值）
- 確認 GPIO 腳位配置正確
- 使用 `--verbose` 模式檢查 p 值是否合理

## 依賴項

**基本版本:**
- pygame
- numpy

**SPI 版本（額外需求）:**
- spidev
- RPi.GPIO