#!/usr/bin/env python3
"""
游戏手柄校准脚本
用于检测游戏手柄的按键和摇杆映射，生成配置文件

作者: Han Jiang (jh18954242606@163.com)
日期: 2025-12
"""

import os
import sys
import struct
import time
import select
import glob
from typing import Dict, Optional, Tuple

# Linux joystick事件结构
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

# 按钮名称映射
BUTTON_NAMES = {
    'LB': 'button_lb',
    'RB': 'button_rb',
    'Back': 'button_back',
    'Start': 'button_start',
    'A': 'button_a',
    'B': 'button_b',
    'X': 'button_x',
    'Y': 'button_y',
}

# 摇杆名称映射
AXIS_NAMES = {
    '左摇杆X': 'axis_left_stick_x',
    '左摇杆Y': 'axis_left_stick_y',
    '右摇杆X': 'axis_right_stick_x',
    '右摇杆Y': 'axis_right_stick_y',
}

class GamepadCalibrator:
    def __init__(self, device_path: str):
        self.device_path = device_path
        self.fd = None
        self.button_map: Dict[str, int] = {}
        self.axis_map: Dict[str, int] = {}
        self.detected_buttons = set()
        self.detected_axes = set()
        
    def open_device(self) -> bool:
        """打开游戏手柄设备"""
        try:
            self.fd = os.open(self.device_path, os.O_RDONLY | os.O_NONBLOCK)
            print(f"✓ 成功打开设备: {self.device_path}")
            return True
        except OSError as e:
            print(f"✗ 无法打开设备 {self.device_path}: {e}")
            return False
    
    def close_device(self):
        """关闭设备"""
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
    
    def read_event(self, timeout: float = 1.0) -> Optional[Tuple[int, int, int]]:
        """
        读取一个joystick事件
        返回: (type, number, value) 或 None
        """
        if self.fd is None:
            return None
        
        # 使用select等待数据可用
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if not ready:
            return None
        
        try:
            # 读取js_event结构体 (8字节: time(4) + value(2) + type(1) + number(1))
            data = os.read(self.fd, 8)
            if len(data) != 8:
                return None
            
            # 解析事件 (小端序)
            time_ms, value, event_type, number = struct.unpack('<IhBB', data)
            
            # 清除JS_EVENT_INIT标志
            event_type &= ~JS_EVENT_INIT
            
            return (event_type, number, value)
        except OSError:
            return None
    
    def detect_all_inputs(self, duration: float = 3.0):
        """检测所有可用的按钮和摇杆"""
        print("\n正在检测所有输入...")
        print("请随意按下按钮和移动摇杆（持续3秒）...")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            event = self.read_event(0.1)
            if event:
                event_type, number, value = event
                if event_type == JS_EVENT_BUTTON:
                    self.detected_buttons.add(number)
                elif event_type == JS_EVENT_AXIS:
                    self.detected_axes.add(number)
        
        print(f"\n检测到 {len(self.detected_buttons)} 个按钮: {sorted(self.detected_buttons)}")
        print(f"检测到 {len(self.detected_axes)} 个摇杆轴: {sorted(self.detected_axes)}")
    
    def wait_for_button_press(self, button_name: str) -> Optional[int]:
        """等待按下指定按钮，返回按钮编号"""
        print(f"\n请按下 [{button_name}] 按钮...")
        
        pressed_buttons = set()
        start_time = time.time()
        timeout = 10.0
        
        while time.time() - start_time < timeout:
            event = self.read_event(0.1)
            if event:
                event_type, number, value = event
                if event_type == JS_EVENT_BUTTON:
                    if value == 1:  # 按下
                        if number not in pressed_buttons:
                            pressed_buttons.add(number)
                            print(f"  检测到按钮按下: 按钮 #{number}")
                    elif value == 0:  # 释放
                        if number in pressed_buttons:
                            pressed_buttons.remove(number)
                            print(f"  检测到按钮释放: 按钮 #{number}")
                            return number
        
        print(f"  ⚠️  超时，未检测到按钮按下")
        return None
    
    def wait_for_axis_movement(self, axis_name: str, exclude_axes: set = None) -> Optional[int]:
        """
        等待移动指定摇杆，返回摇杆轴编号
        
        Args:
            axis_name: 摇杆名称（用于显示）
            exclude_axes: 已映射的轴编号集合，用于排除
        """
        if exclude_axes is None:
            exclude_axes = set()
        
        print(f"\n请移动 [{axis_name}] 摇杆（向左/右或向前/后）...")
        print("  提示：请只移动一个方向，不要同时移动多个方向")
        
        axis_values = {}  # 记录每个轴的初始值
        axis_max_movement = {}  # 记录每个轴的最大移动幅度
        start_time = time.time()
        timeout = 8.0
        movement_threshold = 5000  # 移动阈值（原始值范围是-32767到32767）
        sample_duration = 2.0  # 采样持续时间（秒）
        
        # 首先清空输入缓冲区，获取初始值
        while time.time() - start_time < 0.5:
            event = self.read_event(0.05)
            if event:
                event_type, number, value = event
                if event_type == JS_EVENT_AXIS:
                    if number not in exclude_axes and number not in axis_values:
                        axis_values[number] = value
                        axis_max_movement[number] = 0
        
        # 等待用户开始移动摇杆
        print("  等待摇杆移动...")
        detected_movement = False
        while time.time() - start_time < timeout:
            event = self.read_event(0.05)
            if event:
                event_type, number, value = event
                if event_type == JS_EVENT_AXIS:
                    if number in exclude_axes:
                        continue
                    
                    if number not in axis_values:
                        axis_values[number] = value
                        axis_max_movement[number] = 0
                    
                    # 计算移动幅度
                    movement = abs(value - axis_values[number])
                    if movement > axis_max_movement[number]:
                        axis_max_movement[number] = movement
                    
                    # 如果检测到明显移动
                    if movement > movement_threshold:
                        detected_movement = True
                        # 继续采样一段时间以找到移动幅度最大的轴
                        sample_start = time.time()
                        while time.time() - sample_start < sample_duration:
                            event2 = self.read_event(0.05)
                            if event2:
                                event_type2, number2, value2 = event2
                                if event_type2 == JS_EVENT_AXIS and number2 == number:
                                    movement2 = abs(value2 - axis_values[number2])
                                    if movement2 > axis_max_movement[number2]:
                                        axis_max_movement[number2] = movement2
                        break
        
        if not detected_movement:
            print(f"  ⚠️  超时，未检测到摇杆移动")
            return None
        
        # 找到移动幅度最大的轴（排除已映射的轴）
        candidate_axes = {
            axis_id: movement 
            for axis_id, movement in axis_max_movement.items()
            if axis_id not in exclude_axes and movement > movement_threshold
        }
        
        if not candidate_axes:
            print(f"  ⚠️  未找到有效的摇杆轴")
            return None
        
        # 选择移动幅度最大的轴
        best_axis = max(candidate_axes.items(), key=lambda x: x[1])
        axis_id, max_movement = best_axis
        
        # 如果有多个候选轴，让用户选择
        if len(candidate_axes) > 1:
            print(f"  检测到多个轴移动:")
            sorted_axes = sorted(candidate_axes.items(), key=lambda x: x[1], reverse=True)
            for idx, (ax_id, mov) in enumerate(sorted_axes, 1):
                marker = " <-- 最大移动幅度" if ax_id == axis_id else ""
                print(f"    [{idx}] 轴 #{ax_id}: 移动幅度 {mov}{marker}")
            
            # 尝试让用户确认（非阻塞）
            try:
                choice = input(f"  请选择正确的轴 (1-{len(sorted_axes)})，直接按Enter使用最大幅度的轴: ").strip()
                if choice:
                    selected_idx = int(choice) - 1
                    if 0 <= selected_idx < len(sorted_axes):
                        axis_id = sorted_axes[selected_idx][0]
                        print(f"  已选择: 轴 #{axis_id}")
                else:
                    print(f"  使用默认选择: 轴 #{axis_id} (移动幅度最大)")
            except (ValueError, KeyboardInterrupt):
                print(f"  使用默认选择: 轴 #{axis_id} (移动幅度最大)")
        else:
            print(f"  检测到摇杆移动: 轴 #{axis_id} (移动幅度: {max_movement})")
        
        return axis_id
    
    def calibrate_buttons(self):
        """校准所有按钮"""
        print("\n" + "="*60)
        print("开始校准按钮")
        print("="*60)
        
        for display_name, config_name in BUTTON_NAMES.items():
            button_id = self.wait_for_button_press(display_name)
            if button_id is not None:
                self.button_map[config_name] = button_id
                print(f"  ✓ {display_name} -> 按钮 #{button_id}")
            else:
                print(f"  ✗ {display_name} -> 未检测到")
    
    def calibrate_axes(self):
        """校准所有摇杆轴"""
        print("\n" + "="*60)
        print("开始校准摇杆")
        print("="*60)
        print("注意：请按顺序移动每个摇杆轴，每次只移动一个方向")
        
        mapped_axes = set()  # 已映射的轴编号集合
        
        for display_name, config_name in AXIS_NAMES.items():
            axis_id = self.wait_for_axis_movement(display_name, exclude_axes=mapped_axes)
            if axis_id is not None:
                # 检查是否已经映射过
                if axis_id in mapped_axes:
                    print(f"  ⚠️  警告: 轴 #{axis_id} 已经被映射，跳过")
                    continue
                
                self.axis_map[config_name] = axis_id
                mapped_axes.add(axis_id)
                print(f"  ✓ {display_name} -> 轴 #{axis_id}")
            else:
                print(f"  ✗ {display_name} -> 未检测到")
    
    def generate_config(self) -> str:
        """生成配置文件内容"""
        config_lines = [
            "# 游戏手柄校准结果",
            "# 生成时间: " + time.strftime("%Y-%m-%d %H:%M:%S"),
            "",
            "gamepad:",
            "  device_type: \"gamepad\"",
            f"  joystick_device: \"{self.device_path}\"",
            "  at9s_port: \"/dev/ttyUSB0\"",
            "",
            "  # 摇杆映射（校准结果）",
        ]
        
        # 添加摇杆配置
        for config_name, axis_id in sorted(self.axis_map.items()):
            display_name = {v: k for k, v in AXIS_NAMES.items()}[config_name]
            config_lines.append(f"  {config_name}: {axis_id}  # {display_name}")
        
        config_lines.append("")
        config_lines.append("  # 按钮映射（校准结果）")
        
        # 添加按钮配置
        for config_name, button_id in sorted(self.button_map.items()):
            display_name = {v: k for k, v in BUTTON_NAMES.items()}[config_name]
            config_lines.append(f"  {config_name}: {button_id}  # {display_name}")
        
        config_lines.extend([
            "",
            "  deadzone: 0.1  # 摇杆死区（0.0到1.0）",
            "  speed_forward_max: 0.6  # 前进最大速度（m/s）",
            "  speed_backward_max: 0.8  # 后退最大速度（m/s）",
            "  speed_lateral_max: 0.6  # 左右最大速度（m/s）",
            "  speed_yaw_max: 1.3  # 角速度最大值（rad/s）",
        ])
        
        return "\n".join(config_lines)


def find_gamepad_devices() -> list:
    """查找所有可用的游戏手柄设备"""
    devices = []
    for device_path in glob.glob("/dev/input/js*"):
        if os.path.exists(device_path):
            devices.append(device_path)
    return sorted(devices)


def main():
    print("="*60)
    print("游戏手柄校准工具")
    print("="*60)
    print()
    
    # 查找可用设备
    devices = find_gamepad_devices()
    if not devices:
        print("✗ 未找到游戏手柄设备（/dev/input/js*）")
        print("  请确保游戏手柄已连接并具有读取权限")
        return 1
    
    print("找到以下游戏手柄设备:")
    for i, device in enumerate(devices, 1):
        print(f"  [{i}] {device}")
    
    # 选择设备
    if len(devices) == 1:
        selected_device = devices[0]
        print(f"\n自动选择: {selected_device}")
    else:
        try:
            choice = input(f"\n请选择设备 (1-{len(devices)}): ").strip()
            idx = int(choice) - 1
            if 0 <= idx < len(devices):
                selected_device = devices[idx]
            else:
                print("✗ 无效选择")
                return 1
        except (ValueError, KeyboardInterrupt):
            print("\n✗ 已取消")
            return 1
    
    # 检查权限
    if not os.access(selected_device, os.R_OK):
        print(f"\n⚠️  警告: 没有读取权限 {selected_device}")
        print("  尝试使用 sudo 运行此脚本，或执行:")
        print(f"  sudo chmod 666 {selected_device}")
        try:
            choice = input("\n是否继续? (y/n): ").strip().lower()
            if choice != 'y':
                return 1
        except KeyboardInterrupt:
            return 1
    
    # 创建校准器
    calibrator = GamepadCalibrator(selected_device)
    
    try:
        # 打开设备
        if not calibrator.open_device():
            return 1
        
        # 检测所有输入
        calibrator.detect_all_inputs()
        
        # 等待用户准备
        print("\n" + "="*60)
        print("准备开始校准")
        print("="*60)
        print("请确保游戏手柄已连接并准备就绪")
        try:
            input("\n按 Enter 键开始校准...")
        except KeyboardInterrupt:
            print("\n已取消")
            return 1
        
        # 校准按钮
        calibrator.calibrate_buttons()
        
        # 校准摇杆
        calibrator.calibrate_axes()
        
        # 显示结果
        print("\n" + "="*60)
        print("校准完成")
        print("="*60)
        print("\n按钮映射:")
        for config_name, button_id in sorted(calibrator.button_map.items()):
            display_name = {v: k for k, v in BUTTON_NAMES.items()}[config_name]
            print(f"  {display_name:8s} -> 按钮 #{button_id:2d} ({config_name})")
        
        print("\n摇杆映射:")
        for config_name, axis_id in sorted(calibrator.axis_map.items()):
            display_name = {v: k for k, v in AXIS_NAMES.items()}[config_name]
            print(f"  {display_name:10s} -> 轴 #{axis_id:2d} ({config_name})")
        
        # 生成配置
        config_content = calibrator.generate_config()
        
        # 保存配置
        print("\n" + "="*60)
        output_file = "gamepad_calibration.yaml"
        try:
            with open(output_file, 'w', encoding='utf-8') as f:
                f.write(config_content)
            print(f"✓ 配置已保存到: {output_file}")
            print("\n请将以下内容复制到 config.yaml 的 gamepad 部分:")
            print("-"*60)
            print(config_content)
            print("-"*60)
        except Exception as e:
            print(f"✗ 保存配置文件失败: {e}")
            print("\n配置内容:")
            print("-"*60)
            print(config_content)
            print("-"*60)
        
    except KeyboardInterrupt:
        print("\n\n已取消校准")
        return 1
    finally:
        calibrator.close_device()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

