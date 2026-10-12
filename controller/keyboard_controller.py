"""三輪全向輪機器人鍵盤控制器。"""
import os
import sys

# 鍵盤控制需要視窗事件；不要讓 F710 模組強制使用 dummy video driver。
os.environ.pop("SDL_VIDEODRIVER", None)

import pygame

from controller2spi import OmniRobotSPIController, MOTOR_MAX_RPS, UPDATE_RATE


WINDOW_SIZE = (520, 240)
WINDOW_TITLE = "Omni Robot Keyboard Controller"


class KeyboardRobotController(OmniRobotSPIController):
    """以鍵盤取代 F710 手把，沿用既有的逆運動學與 SPI 輸出。"""

    def __init__(self, motor_max_rps=MOTOR_MAX_RPS, verbose=False):
        super().__init__(motor_max_rps=motor_max_rps, verbose=verbose)
        self.window = pygame.display.set_mode(WINDOW_SIZE)
        pygame.display.set_caption(WINDOW_TITLE)
        self.clock = pygame.time.Clock()

    @staticmethod
    def read_keyboard_input():
        """回傳平移 X/Y、自轉 W，以及煞車和退出狀態。"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return 0.0, 0.0, 0.0, False, True

        keys = pygame.key.get_pressed()
        if keys[pygame.K_ESCAPE] or keys[pygame.K_x]:
            return 0.0, 0.0, 0.0, False, True

        right = keys[pygame.K_d] or keys[pygame.K_RIGHT]
        left = keys[pygame.K_a] or keys[pygame.K_LEFT]
        forward = keys[pygame.K_w] or keys[pygame.K_UP]
        backward = keys[pygame.K_s] or keys[pygame.K_DOWN]
        clockwise = keys[pygame.K_e]
        counterclockwise = keys[pygame.K_q]

        input_x = float(right) - float(left)
        input_y = float(forward) - float(backward)
        input_w = float(clockwise) - float(counterclockwise)
        braking = keys[pygame.K_SPACE]
        return input_x, input_y, input_w, braking, False

    def send_wheel_commands(self, result, braking):
        """將計算後的輪速送到三個馬達。"""
        wheel_data = []
        wheels = result['wheels']
        for index, (rps, period, direction) in enumerate(
            ((wheels['rps1'], wheels['p1'], wheels['dir1']),
             (wheels['rps2'], wheels['p2'], wheels['dir2']),
             (wheels['rps3'], wheels['p3'], wheels['dir3'])),
            1,
        ):
            enabled = not braking and abs(rps) > 0.01
            byte1, byte2 = self.create_spi_data(period, direction, enabled)
            self.spi_transfer(f'CS{index - 1}', [byte1, byte2])
            wheel_data.append((byte1, byte2, enabled, direction > 0, period))
        return wheel_data

    def draw_status(self, result, wheel_data, braking):
        """更新控制視窗中的狀態。"""
        self.window.fill((20, 24, 30))
        font = pygame.font.Font(None, 28)
        small_font = pygame.font.Font(None, 22)
        lines = [
            "Keyboard control: WASD / Arrow keys",
            "Q/E: rotate    Space: brake    Esc/X: quit",
            f"Input X={result['target_vx_pct']:+.0f}  Y={-result['target_vy_pct']:+.0f}  "
            f"W={result['target_wz_pct']:+.0f}  {'[BRAKE]' if braking else ''}",
        ]
        for line_number, text in enumerate(lines):
            surface = (font if line_number == 0 else small_font).render(
                text, True, (235, 235, 235)
            )
            self.window.blit(surface, (20, 20 + line_number * 32))

        wheel_text = "  ".join(
            f"W{index}: {'ON' if enabled else 'OFF'} p={period:4d}"
            for index, (_, _, enabled, _, period) in enumerate(wheel_data, 1)
        )
        surface = small_font.render(wheel_text, True, (150, 205, 220))
        self.window.blit(surface, (20, 135))
        pygame.display.flip()

    def run(self):
        print("[啟動] 鍵盤控制器已啟動")
        print("WASD/方向鍵平移，Q/E自轉，Space煞車，Esc或X退出")
        try:
            while True:
                input_x, input_y, input_w, braking, should_quit = self.read_keyboard_input()
                if should_quit:
                    break

                trigger_left = -input_w if input_w < 0 else -1.0
                trigger_right = input_w if input_w > 0 else -1.0
                result = self.process_control_input(
                    input_x, -input_y, trigger_left, trigger_right
                )
                wheel_data = self.send_wheel_commands(result, braking)
                self.draw_status(result, wheel_data, braking)
                self.clock.tick(round(1.0 / UPDATE_RATE))
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()
            pygame.quit()


if __name__ == "__main__":
    verbose = "--verbose" in sys.argv or "-v" in sys.argv
    robot = KeyboardRobotController(verbose=verbose)
    robot.run()