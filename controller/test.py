"""F710 手把按鈕測試工具"""
import time
import sys

try:
    from f710controller import F710Controller
except ImportError as e:
    print(f"[錯誤] 無法導入 f710controller: {e}")
    sys.exit(1)


def main():
    controller = F710Controller(deadzone=0.0)
    
    try:
        controller.wait_for_connection()
    except TimeoutError as e:
        print(f"[錯誤] {e}")
        return 1
    except Exception as e:
        print(f"[錯誤] 連接失敗: {e}")
        return 1
    
    joy = controller.joy
    print(f"[資訊] 總共有 {joy.get_numbuttons()} 顆按鈕")
    print(f"[資訊] 總共有 {joy.get_numaxes()} 個軸")
    print(f"[資訊] 總共有 {joy.get_numhats()} 個方向鍵")
    print("\n請用力壓下左右搖桿 (L3/R3)，看看下面跳出幾號...")
    print("(按 Ctrl+C 結束)\n")

    try:
        last_buttons = [False] * joy.get_numbuttons()
        
        while True:
            data = controller.read()
            buttons = data['buttons']
            
            # 掃描所有按鈕（只在狀態改變時顯示）
            for i, pressed in enumerate(buttons):
                if pressed and not last_buttons[i]:
                    print(f"[按鈕] 偵測到按鈕 ID: {i}")
                last_buttons[i] = pressed
            
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\n\n[完成] 測試結束")
        return 0

if __name__ == "__main__":
    sys.exit(main())