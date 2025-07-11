import pygame
import sys


class KeyboardController:
    def __init__(self):
        pygame.init()
        # 创建一个小窗口来接收键盘事件
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("Robot Keyboard Controller")
        
        # 模拟手柄状态
        self.lx = 0.0
        self.ly = 0.0
        self.rx = 0.0
        self.ry = 0.0
        self.button = [0] * 16
        
        self.button_states = [False] * 16
        self.button_pressed = [False] * 16 
        self.button_released = [False] * 16
        
        # 键盘按键映射到手柄按键
        self.key_mapping = {
            # 手柄按键映射
            pygame.K_r: 0,      # R1
            pygame.K_l: 1,      # L1
            pygame.K_RETURN: 2, # start
            pygame.K_ESCAPE: 3, # select
            pygame.K_t: 4,      # R2
            pygame.K_g: 5,      # L2
            pygame.K_F1: 6,     # F1
            pygame.K_F2: 7,     # F2
            pygame.K_j: 8,      # A
            pygame.K_k: 9,      # B
            pygame.K_i: 10,     # X
            pygame.K_u: 11,     # Y
            pygame.K_UP: 12,    # up
            pygame.K_RIGHT: 13, # right
            pygame.K_DOWN: 14,  # down
            pygame.K_LEFT: 15,  # left
        }
        
        # 摇杆控制键
        self.axis_keys = {
            pygame.K_w: ('ly', 1.0),   # 左摇杆前
            pygame.K_s: ('ly', -1.0),  # 左摇杆后
            pygame.K_a: ('lx', -1.0),  # 左摇杆左
            pygame.K_d: ('lx', 1.0),   # 左摇杆右
            pygame.K_q: ('rx', -1.0),  # 右摇杆左
            pygame.K_e: ('rx', 1.0),   # 右摇杆右
            pygame.K_z: ('ry', 1.0),   # 右摇杆前
            pygame.K_c: ('ry', -1.0),  # 右摇杆后
        }
        
        self.axis_speed = 0.8  # 摇杆灵敏度
        
        # 显示控制说明
        self._display_controls()
        
        print("键盘控制器初始化完成!")
        print("请保持pygame窗口为活动状态以接收键盘输入")

    def _display_controls(self):
        """在pygame窗口显示控制说明"""
        font = pygame.font.Font(None, 24)
        small_font = pygame.font.Font(None, 18)
        
        controls = [
            "Robot Keyboard Controller",
            "",
            "Basic Controls:",
            "R1: R    L1: L    Start: Enter",
            "A: J     B: K     X: I     Y: U",
            "Direction: Arrow Keys",
            "",
            "Joystick Controls:",
            "Left Stick: WASD",
            "Right Stick: QE(turn) ZC(tilt)",
            "",
            "Emergency: F1",
            "",
            "Mode Commands:",
            "R+X: Dance    R+A: Walk",
            "Enter: Fixed Pose",
            "Esc: Passive Mode"
        ]
        
        self.screen.fill((50, 50, 50))
        y_offset = 10
        
        for i, line in enumerate(controls):
            if i == 0:  # Title
                text = font.render(line, True, (255, 255, 0))
            elif line == "" or ":" in line:  # Headers
                text = font.render(line, True, (200, 200, 200))
            else:  # Content
                text = small_font.render(line, True, (255, 255, 255))
            
            self.screen.blit(text, (10, y_offset))
            y_offset += 18 if line else 10
        
        pygame.display.flip()

    def update(self):
        """更新键盘输入状态"""
        # 重置摇杆值
        self.lx = 0.0
        self.ly = 0.0
        self.rx = 0.0
        self.ry = 0.0
        
        # 重置按键释放状态
        self.button_released = [False] * 16
        
        # 处理pygame事件
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_h:  # 按H键显示帮助
                    self._display_controls()
        
        # 获取当前按键状态
        keys = pygame.key.get_pressed()
        
        # 更新按键状态
        for key, button_id in self.key_mapping.items():
            current_state = keys[key]
            
            # 检测按键释放
            if self.button_states[button_id] and not current_state:
                self.button_released[button_id] = True
            
            # 更新状态
            self.button_states[button_id] = current_state
            self.button[button_id] = 1 if current_state else 0
        
        # 更新摇杆状态
        for key_code, (axis, value) in self.axis_keys.items():
            if keys[key_code]:
                if axis == 'lx':
                    self.lx += value * self.axis_speed
                elif axis == 'ly':
                    self.ly += value * self.axis_speed
                elif axis == 'rx':
                    self.rx += value * self.axis_speed
                elif axis == 'ry':
                    self.ry += value * self.axis_speed
        
        # 限制摇杆值范围
        self.lx = max(-1.0, min(1.0, self.lx))
        self.ly = max(-1.0, min(1.0, self.ly))
        self.rx = max(-1.0, min(1.0, self.rx))
        self.ry = max(-1.0, min(1.0, self.ry))

    def is_button_pressed(self, button_id):
        """检测按键是否被按下"""
        if 0 <= button_id < 16:
            return self.button_states[button_id]
        return False

    def is_button_released(self, button_id):
        """检测按键是否被释放"""
        if 0 <= button_id < 16:
            return self.button_released[button_id]
        return False

    def get_axis_value(self, axis_id=None):
        """获取摇杆轴值"""
        return self.lx, self.rx, self.ry, self.ly

    def set(self, data=None):
        """兼容RemoteController接口，实际更新通过update()方法"""
        self.update()

    def close(self):
        """关闭pygame"""
        pygame.quit()


class KeyMap:
    """键盘映射常量，与RemoteController保持一致"""
    R1 = 0
    L1 = 1
    start = 2
    select = 3
    R2 = 4
    L2 = 5
    F1 = 6
    F2 = 7
    A = 8
    B = 9
    X = 10
    Y = 11
    up = 12
    right = 13
    down = 14
    left = 15
