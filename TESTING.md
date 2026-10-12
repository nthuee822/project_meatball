# 測試清單

> 建立日期：2026-10-05
> 標記方式：測過就把 `[ ]` 改成 `[x]`，有問題的寫在「備註」。
> 👀 = 需要親自看硬體表現　🔴 = 會讓馬達轉，**輪子先架空**
> 「Claude 結果」是 2026-10-05 在 Pi 上自動跑的結果，僅供參考，仍需自己再跑一次。

---

## ⚠️ 已知問題（先處理）

- [ ] **IMU I2C 不穩**：`i2cdetect` 看得到 0x68、單 byte 讀 `WHO_AM_I = 0xEA` 正常，但連續讀取（burst read）會失敗（`Errno 121` / `Errno 110`）。`dmesg` 出現 `SDA stuck at low`、`lost arbitration`。
  → 檢查 SDA/SCL 接線、pull-up、線長，然後 **IMU 斷電重開**，再跑 B 區。
- [ ] **F710 手把連不上**：`lsusb` 看不到 Logitech 接收器、沒有 `/dev/input/js*`。C 區先跳過，D 區改用鍵盤控制。
- [ ] **README 的 IK 預期值寫錯**（見 A3）
- [ ] **燒錄腳本路徑錯誤**（見 E9）

---

## A. 純軟體（不用接硬體）

| # | 測試 | 指令 | 相關檔案 | Claude 結果 | 備註 |
|---|---|---|---|---|---|
| [v] A1 | 所有 Python 檔語法編譯 | `python3 -m py_compile <file>` | `controller/*.py`、`controller/IMU_DMP/*.py`、`IMU/*.py`、`IMU/debug/*.py`、`stm32/adc_alignment_test/plot_adc.py` | ✅ 全部通過 | |
| [v] A2 | 逆運動學單元測試 | `cd controller && uv run python run_test.py` | `controller/run_test.py`、`controller/inverse_kinematics.py` | ✅ 通過（但只測前進） | |
| [!] A3 | IK 數值與 README 對照 | 手動對照 | `controller/inverse_kinematics.py`、`controller/README.md`（「驗證測試」段落、參數配置段落） | ❌ README 寫錯 | 見下方 |
| [v] A4 | ADC 資料繪圖 | `cd stm32/adc_alignment_test && uv run python plot_adc.py minicom.cap` | `stm32/adc_alignment_test/plot_adc.py`、`minicom.cap` | ✅ 通過（uv 環境已含 matplotlib） | 預設輸入 `adc_samples.txt` 不存在，需指定 `minicom.cap`；`default.csv` 是示波器資料，不是此腳本的輸入格式 |

**A3 細節**：程式實際算出來的值

| 輸入 | 程式結果 (v1, v2, v3) | README 寫的 |
|---|---|---|
| 前進 vy=1 | 1.0, -0.5, -0.5 | 1.0, -0.5, -0.5 ✅ |
| 右移 vx=1 | 0, **-0.866, +0.866** | 0, +0.866, -0.866 ❌ |
| 斜向 vx=vy=0.5 | 0.5, **-0.683, +0.183** | 0.5, +0.183, -0.683 ❌ |

README 的公式本身和程式一致，只有「預期結果」沒更新。另外參數表寫 `ACCEL_RATE_UP = 0.5`，程式已改成 `1.0`。
→ 實車測 D 區時順便確認：**哪一組才是對的實際方向**。

---

## B. IMU（只讀不動，安全）

前置：先解決上方 I2C 問題。

| # | 測試 | 指令 | 相關檔案 | Claude 結果 | 備註 |
|---|---|---|---|---|---|
| [v] B1 | I2C 偵測 | `sudo i2cdetect -y 1` → 看到 `68` | （系統工具） | ✅ 看到 0x68 | |
| [v] B2 | 最基本讀值 | `cd IMU/debug && python3 sensor_test.py` | `IMU/debug/sensor_test.py`、`IMU/debug/icm20948.py` | ❌ `Errno 110` timeout | |
| [v] B3 | 串流 + 陀螺儀校正 | `cd IMU && python3 icm20948_test.py` | `IMU/icm20948_test.py`、`IMU/icm20948.py` | ❌ warm-up 時 `Errno 121` | warm-up 30 秒期間**不要動板子** |
| [v] B4 | 加速度計校正 | `cd IMU && python3 accel_calib.py` | `IMU/accel_calib.py`、`IMU/icm20948.py` | 未跑（需翻轉板子） | 照提示擺不同方向 |
| [v] B5 👀 | 姿態角度 | `cd IMU && python3 imu_test_run.py` | `IMU/imu_test_run.py`、`IMU/imu.py`、`IMU/icm20948.py` | 未跑 | 轉板子看角度是否合理 |
| [v] B6 👀 | IMU → 控制量 | `cd IMU && python3 raw2ctrl.py` | `IMU/raw2ctrl.py`、`IMU/icm20948.py` | 未跑 | 傾斜方向和輸出符號是否對應 |
| [ ] B7 👀 | DMP 即時姿態 | `cd controller && python3 -m IMU_DMP.dmp_live` | `controller/IMU_DMP/dmp_live.py`、`dmp_imu.py`、`_dmp_firmware.py`、`icm20948.py` | ❌ 載入 firmware 時 I2C 失敗 | 四元數/角度跟實際轉動對得上 |
| [ ] B8 | DMP 診斷 | `cd controller && python3 -m IMU_DMP.dmp_diag` | `controller/IMU_DMP/dmp_diag.py`、`dmp_imu.py` | 未跑 | |
| [ ] B9 | DMP 取樣率 / 漂移 | `cd controller && python3 -m IMU_DMP.imu_benchmark --duration 60` | `controller/IMU_DMP/imu_benchmark.py`、`dmp_imu.py` | 未跑 | 板子靜置 |
| [v] B10 | （選用）暫存器除錯工具 | `cd IMU/debug && python3 debug_*.py` | `IMU/debug/debug_addresses.py`、`debug_banks.py`、`debug_minimal.py`、`debug_pwr.py`、`debug_registers.py`、`debug_user_ctrl.py`、`debug_write_methods.py` | 未跑 | I2C 有問題時用來查 |

---

## C. F710 手把（⏸ 目前連不上，先跳過）

前置：插上 USB 接收器、背面開關切到 **X**、`ls /dev/input/js*` 有東西。

| # | 測試 | 指令 | 相關檔案 | Claude 結果 | 備註 |
|---|---|---|---|---|---|
| [ ] C1 👀 | 按鈕 / 搖桿 / 扳機讀值 | `cd controller && python3 test.py` | `controller/test.py` | 未跑（無手把） | |
| [ ] C2 👀 | 顯示模式（不送 SPI） | `cd controller && python3 omni_robot.py` | `controller/omni_robot.py`、`f710controller.py`、`inverse_kinematics.py` | 未跑（無手把） | 看 v1/v2/v3 方向、加速平滑度（`ACCEL_RATE_UP` 已改 1.0）、連線震動 |
| [ ] C3 🔴👀 | 手把 + SPI | `cd controller && python3 controller2spi.py --verbose` | `controller/controller2spi.py`、`omni_robot.py`、`f710controller.py`、`inverse_kinematics.py` | 未跑 | 按住 A 三輪停（EN=0）、長按 X 1.5 秒退出並震兩次、RT/LT 自轉 |
| [ ] C4 🔴👀 | 抗機構干涉版 | `cd controller && python3 4test.py` | `controller/4test.py`、`omni_robot.py` | 未跑 | 先確認這支還要不要留 |

---

## D. 鍵盤控制 + SPI 送馬達（🔴 輪子先架空）

環境：`/dev/spidev0.0` ✅、RPi.GPIO（rpi-lgpio）可以設定 GPIO ✅
CS 腳位：GPIO 17 = 輪1（前）、GPIO 27 = 輪2（左後）、GPIO 22 = 輪3（右後）

| # | 測試 | 指令 | 相關檔案 | Claude 結果 | 備註 |
|---|---|---|---|---|---|
| [v] D1 🔴👀 | 終端機鍵盤控制（**建議先用這支，SSH 可跑**） | `cd controller && python3 keyboard_teleop.py --speed 0.3` | `controller/keyboard_teleop.py`、`controller2spi.py`、`omni_robot.py`、`inverse_kinematics.py` | 未跑 | 見下方 D1 細項 |
| [-] D2 🔴👀 | pygame 視窗鍵盤控制（需桌面 / VNC） | `cd controller && python3 keyboard_controller.py` | `controller/keyboard_controller.py`、`controller2spi.py`、`omni_robot.py`、`f710controller.py` | 未跑 | 見下方 D2 細項 |
| [ ] D3 🔴👀 | IMU 融合控制 | `cd controller && python3 ctrl_with_imu_2spi.py` | `controller/ctrl_with_imu_2spi.py`、`omni_robot.py`、`IMU_DMP/icm20948.py` | 未跑 | ⚠️ 會等手把連線，需 B 區、C 區都通過 |

keyboard control 的東西之前試過要用的東西 之後有碰到什麼 bug 再修

**D1 細項（keyboard_teleop.py）**
- [ ] W / S / A / D 或方向鍵：前進 / 後退 / 左移 / 右移，每顆輪子轉向正確
- [ ] Q / E / Z / C：四個斜向
- [ ] J / L：自轉 − / +
- [ ] 1~9、0：速度 10%~100%
- [ ] 放開按鍵後約 0.6 秒內停止（若按住時一頓一頓，調大 `--hold-timeout`）
- [ ] Space：立即停止（EN=0）
- [ ] Ctrl+C：離開，馬達停止、GPIO 釋放

**D2 細項（keyboard_controller.py）**
- [ ] WASD / 方向鍵：前後左右平移
- [ ] Q / E：自轉
- [ ] 按住 Space：馬達暫停，放開恢復
- [ ] Esc 或 X：安全退出，馬達停止

---

## E. STM32 韌體

這邊基本上是在電腦上面跑的 raspberry pi 上面不需要跑得動

⚠️ 這台 Pi **沒有 `arm-none-eabi-gcc` 和 `cmake`**，沒辦法重新 build。要在有 STM32CubeIDE 的電腦上 build，或在 Pi 上安裝 toolchain。各專案目前都有舊的 `build/Debug/*.elf`。

| # | 專案 | MCU | 相關檔案 | 要 build | 硬體驗證 👀 | 備註 |
|---|---|---|---|---|---|---|
| [ ] E1 | adc_alignment_test | STM32G431 | `stm32/adc_alignment_test/Core/Src/main.c`、`stm32/common_lib/as5600.c` | [ ] | [ ] ADC 對齊，接示波器看 | |
| [ ] E2 | angle_measurement_test | STM32F103 | `stm32/angle_measurement_test/Core/Src/main.c`、`Core/Src/as5600.c` | [ ] | [ ] 手轉磁鐵，AS5600 角度連續、不跳 | |
| [ ] E3 | angle_measurement_kalman（submodule） | STM32G431 | `stm32/angle_measurement_kalman/Core/Src/main.c`、`Core/Src/as5600.c` | [ ] | [ ] Kalman 濾波後角度平滑 | ⚠️ `main.c` 有未 commit 修改 |
| [ ] E4 | foc_v0 | STM32G431 | `stm32/foc_v0/Core/Src/main.c`、`stm32/common_lib/as5600.c` | [ ] | [ ] FOC 能轉 | |
| [ ] E5 | foc_v1 | STM32G431 | `stm32/foc_v1/Core/Src/main.c`、`Core/Src/as5600.c` | [ ] | [ ] FOC 閉環轉動 | |
| [ ] E6 | motor | STM32F103 | `stm32/motor/Core/Src/main.c`、`Core/Src/as5600.c` | [ ] | [ ] 馬達轉動 | |
| [ ] E7 | spi_stepper_v1 | STM32F103 | `stm32/spi_stepper_v1/Core/Src/main.c` | [ ] | [ ] 接收 Pi 的 16-bit SPI frame，轉速符合 RPS = 10000/64/p | 跟 D 區一起測 |
| [ ] E8 | common_lib | — | `stm32/common_lib/as5600.c`、`as5600.h`、`stm32-cordic.h` | 隨 E1、E4 一起 | — | |
| [ ] E9 | 燒錄腳本 | — | `stm32/upload.sh`、`stm32/motor/upload.sh`、`stm32/spi_stepper_v1/upload.sh` | — | [ ] 能成功燒錄 | ❌ 見下方 |

**E9 細節**：`upload.sh` 用的是 `-f /target/stm32f1x.cfg`，前面多了 `/`，這個路徑不存在，應該是 `target/stm32f1x.cfg`。而且它寫死 F1，只能燒 E2、E6、E7。G431 的專案（E1、E3、E4、E5）要用 `target/stm32g4x.cfg`。

---

## F. Repo 整理（非測試）

- [v] commit `controller/keyboard_controller.py`、`controller/keyboard_teleop.py`、`README.md`（目前未追蹤）
- [v] commit `controller/f710controller.py`、`omni_robot.py`、`README.md` 的修改
- [v] `controller/IMU_DMP` 是 gitlink，但 `.gitmodules` 沒登記，別人 clone 下來會是空資料夾
- [v] `controller/__pycache__/*.pyc` 被 git 追蹤了，建議加進 `.gitignore`
- [v] commit 或捨棄 `stm32/angle_measurement_kalman` submodule 裡 `main.c` 的修改
