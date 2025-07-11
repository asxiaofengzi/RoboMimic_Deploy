#!/usr/bin/env python3
"""
键盘控制器测试脚本
用于验证键盘输入是否正常工作
"""

import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent.absolute()))

from common.remote_controller import create_controller, KeyMap
import time


def test_keyboard_controller():
    """测试键盘控制器功能"""
    try:
        print("初始化键盘控制器...")
        controller = create_controller("keyboard")
        print("键盘控制器初始化成功!")
        print("\n测试说明:")
        print("- 按不同的键查看响应")
        print("- 使用WASD控制左摇杆")
        print("- 使用QE ZC控制右摇杆")
        print("- 按Esc键退出测试")
        print("-" * 50)
        
        running = True
        last_print_time = time.time()
        
        while running:
            controller.set()  # 更新控制器状态
            
            # 检查退出条件
            if controller.is_button_pressed(KeyMap.select):  # Esc键
                print("检测到Esc键，退出测试...")
                running = False
                break
            
            # 每500ms打印一次状态信息
            current_time = time.time()
            if current_time - last_print_time > 0.5:
                # 获取摇杆值
                lx, rx, ry, ly = controller.get_axis_value()
                
                # 检查按键状态
                pressed_buttons = []
                button_names = {
                    KeyMap.R1: "R1", KeyMap.L1: "L1", KeyMap.start: "Start",
                    KeyMap.A: "A", KeyMap.B: "B", KeyMap.X: "X", KeyMap.Y: "Y",
                    KeyMap.up: "Up", KeyMap.down: "Down", 
                    KeyMap.left: "Left", KeyMap.right: "Right",
                    KeyMap.F1: "F1"
                }
                
                for button_id, name in button_names.items():
                    if controller.is_button_pressed(button_id):
                        pressed_buttons.append(name)
                
                # 只在有输入时打印信息
                if pressed_buttons or abs(lx) > 0.1 or abs(ly) > 0.1 or abs(rx) > 0.1 or abs(ry) > 0.1:
                    print(f"摇杆: LX={lx:.2f} LY={ly:.2f} RX={rx:.2f} RY={ry:.2f} | 按键: {', '.join(pressed_buttons) if pressed_buttons else '无'}")
                
                last_print_time = current_time
            
            time.sleep(0.02)  # 50Hz更新频率
            
    except KeyboardInterrupt:
        print("\n用户中断测试")
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
    finally:
        if 'controller' in locals() and hasattr(controller, 'close'):
            controller.close()
        print("测试结束")


if __name__ == "__main__":
    test_keyboard_controller()
