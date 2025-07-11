import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent.absolute()))

from common.path_config import PROJECT_ROOT

import time
import mujoco.viewer
import mujoco
import numpy as np
import yaml
import os
import argparse
from common.ctrlcomp import *
from FSM.FSM import *
from common.utils import get_gravity_orientation
from common.joystick import JoyStick, JoystickButton
from common.remote_controller import create_controller, KeyMap


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd


class ControllerAdapter:
    """适配器类，统一不同控制器的接口"""
    
    def __init__(self, controller_type="joystick"):
        self.controller_type = controller_type
        
        if controller_type == "keyboard":
            self.controller = create_controller("keyboard")
            self._button_map = self._keyboard_button_map
        else:
            self.controller = JoyStick()
            self._button_map = self._joystick_button_map
    
    def _keyboard_button_map(self, button):
        """键盘按键映射"""
        mapping = {
            JoystickButton.A: KeyMap.A,
            JoystickButton.B: KeyMap.B, 
            JoystickButton.X: KeyMap.X,
            JoystickButton.Y: KeyMap.Y,
            JoystickButton.L1: KeyMap.L1,
            JoystickButton.R1: KeyMap.R1,
            JoystickButton.SELECT: KeyMap.select,
            JoystickButton.START: KeyMap.start,
            JoystickButton.L3: KeyMap.F1,  # 映射F1为L3功能
        }
        return mapping.get(button, button)
    
    def _joystick_button_map(self, button):
        """手柄按键直接映射"""
        return button
    
    def update(self):
        """更新控制器状态"""
        if self.controller_type == "keyboard":
            self.controller.set()  # 键盘控制器通过set方法更新
        else:
            self.controller.update()
    
    def is_button_pressed(self, button):
        """检测按键是否被按下"""
        mapped_button = self._button_map(button)
        return self.controller.is_button_pressed(mapped_button)
    
    def is_button_released(self, button):
        """检测按键是否被释放"""
        mapped_button = self._button_map(button)
        return self.controller.is_button_released(mapped_button)
    
    def get_axis_value(self, axis_id):
        """获取摇杆轴值"""
        if self.controller_type == "keyboard":
            lx, rx, ry, ly = self.controller.get_axis_value()
            axis_values = [lx, ly, rx, ry]  # 映射到joystick的轴顺序
            if 0 <= axis_id < len(axis_values):
                return axis_values[axis_id]
            return 0.0
        else:
            return self.controller.get_axis_value(axis_id)


if __name__ == "__main__":
    # 命令行参数解析
    parser = argparse.ArgumentParser(description='Robot Simulation with Keyboard/Joystick Control')
    parser.add_argument('--controller', 
                       choices=['keyboard', 'joystick'], 
                       default='joystick',
                       help='Controller type: keyboard or joystick (default: joystick)')
    args = parser.parse_args()
    
    current_dir = os.path.dirname(os.path.abspath(__file__))
    mujoco_yaml_path = os.path.join(current_dir, "config", "mujoco.yaml")
    with open(mujoco_yaml_path, "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        xml_path = os.path.join(PROJECT_ROOT, config["xml_path"])
        simulation_dt = config["simulation_dt"]
        control_decimation = config["control_decimation"]
        
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = simulation_dt
    mj_per_step_duration = simulation_dt * control_decimation
    num_joints = m.nu
    policy_output_action = np.zeros(num_joints, dtype=np.float32)
    kps = np.zeros(num_joints, dtype=np.float32)
    kds = np.zeros(num_joints, dtype=np.float32)
    sim_counter = 0
    
    state_cmd = StateAndCmd(num_joints)
    policy_output = PolicyOutput(num_joints)
    FSM_controller = FSM(state_cmd, policy_output)
    
    # 使用控制器适配器
    try:
        controller = ControllerAdapter(args.controller)
        print(f"使用 {args.controller} 控制器")
        if args.controller == "keyboard":
            print("键盘控制说明:")
            print("R+X: 舞蹈模式    R+A: 行走模式")
            print("R+Y: 武术动作    R+B: 踢腿动作")
            print("Enter: 位控模式  Esc: 阻尼模式")
            print("F1: 紧急保护    WASD: 移动")
            print("QE: 转向        ZC: 俯仰")
            print("按H键在pygame窗口查看完整帮助")
    except Exception as e:
        print(f"控制器初始化失败: {e}")
        if args.controller == "joystick":
            print("没有检测到手柄，尝试使用键盘控制器...")
            try:
                controller = ControllerAdapter("keyboard")
                print("已切换到键盘控制模式")
            except Exception as e2:
                print(f"键盘控制器也无法初始化: {e2}")
                sys.exit(1)
        else:
            sys.exit(1)
    
    Running = True
    with mujoco.viewer.launch_passive(m, d) as viewer:
        sim_start_time = time.time()
        while viewer.is_running() and Running:
            try:
                # 检查退出条件
                if controller.is_button_pressed(JoystickButton.SELECT):
                    Running = False

                controller.update()
                
                # 技能命令映射
                if controller.is_button_released(JoystickButton.L3):
                    state_cmd.skill_cmd = FSMCommand.PASSIVE
                if controller.is_button_released(JoystickButton.START):
                    state_cmd.skill_cmd = FSMCommand.POS_RESET
                if controller.is_button_released(JoystickButton.A) and controller.is_button_pressed(JoystickButton.R1):
                    state_cmd.skill_cmd = FSMCommand.LOCO
                    print("切换到行走模式")
                if controller.is_button_released(JoystickButton.X) and controller.is_button_pressed(JoystickButton.R1):
                    state_cmd.skill_cmd = FSMCommand.SKILL_1
                    print("切换到舞蹈模式")
                if controller.is_button_released(JoystickButton.Y) and controller.is_button_pressed(JoystickButton.R1):
                    state_cmd.skill_cmd = FSMCommand.SKILL_2
                    print("切换到武术模式")
                if controller.is_button_released(JoystickButton.B) and controller.is_button_pressed(JoystickButton.R1):
                    state_cmd.skill_cmd = FSMCommand.SKILL_3
                    print("切换到踢腿模式")
                if controller.is_button_released(JoystickButton.Y) and controller.is_button_pressed(JoystickButton.L1):
                    state_cmd.skill_cmd = FSMCommand.SKILL_4
                    print("切换到技能4模式")
                
                # 运动控制
                state_cmd.vel_cmd[0] = -controller.get_axis_value(1)  # 前进后退
                state_cmd.vel_cmd[1] = -controller.get_axis_value(0)  # 左右移动
                state_cmd.vel_cmd[2] = -controller.get_axis_value(3)  # 转向
                
                step_start = time.time()
                
                tau = pd_control(policy_output_action, d.qpos[7:], kps, np.zeros_like(kps), d.qvel[6:], kds)
                d.ctrl[:] = tau
                mujoco.mj_step(m, d)
                sim_counter += 1
                if sim_counter % control_decimation == 0:
                    
                    qj = d.qpos[7:]
                    dqj = d.qvel[6:]
                    quat = d.qpos[3:7]
                    
                    omega = d.qvel[3:6] 
                    gravity_orientation = get_gravity_orientation(quat)
                    
                    state_cmd.q = qj.copy()
                    state_cmd.dq = dqj.copy()
                    state_cmd.gravity_ori = gravity_orientation.copy()
                    state_cmd.ang_vel = omega.copy()
                    
                    FSM_controller.run()
                    policy_output_action = policy_output.actions.copy()
                    kps = policy_output.kps.copy()
                    kds = policy_output.kds.copy()
            except ValueError as e:
                print(str(e))
            except KeyboardInterrupt:
                print("用户中断程序")
                Running = False
            
            viewer.sync()
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
    
    # 清理资源
    if hasattr(controller.controller, 'close'):
        controller.controller.close()
    print("程序结束")
