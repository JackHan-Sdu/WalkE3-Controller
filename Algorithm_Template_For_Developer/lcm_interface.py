#!/usr/bin/env python3
"""
LCM通信接口模块
提供统一的LCM通信接口，供所有外部算法使用
"""

import sys
import os
import threading
import time

# 添加LCM Python模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), './lcm-types/python'))

try:
    import lcm
    from development_state_t import development_state_t
    from development_command_t import development_command_t
    LCM_AVAILABLE = True
except ImportError as e:
    LCM_AVAILABLE = False
    print(f"Warning: LCM modules not available: {e}")


class LCMInterface:
    """LCM通信接口类"""
    
    def __init__(self, config):
        """
        初始化LCM接口
        
        Args:
            config: 配置字典，包含以下键：
                - lcm.url: LCM URL（可选，空字符串表示使用默认）
                - lcm.state_channel: 状态通道名称
                - lcm.command_channel: 命令通道名称
                - lcm.robot_id: 机器人ID
        """
        if not LCM_AVAILABLE:
            raise RuntimeError("LCM modules not available. Please ensure LCM Python bindings are installed.")
        
        self.config = config
        self.lcm_instance = None
        self.running = False
        
        # 状态缓存
        self.latest_state = None
        self.state_lock = threading.Lock()
        self.last_state_time = None
        
        # 回调函数
        self.state_callback = None
        
        # 初始化LCM
        self._init_lcm()
    
    def _init_lcm(self):
        """初始化LCM实例"""
        # 检查配置中是否有lcm部分
        if 'lcm' not in self.config:
            raise KeyError("Configuration missing 'lcm' section. Please ensure your config.yaml contains an 'lcm' section with 'url', 'state_channel', 'command_channel', and 'robot_id'.")
        
        lcm_config = self.config['lcm']
        lcm_url = lcm_config.get('url', '')
        if lcm_url:
            self.lcm_instance = lcm.LCM(lcm_url)
        else:
            self.lcm_instance = lcm.LCM()
        
        # 订阅状态通道
        if 'state_channel' not in lcm_config:
            raise KeyError("Configuration missing 'lcm.state_channel'. Please specify the state channel name.")
        state_channel = lcm_config['state_channel']
        self.lcm_instance.subscribe(state_channel, self._on_state_received)
        print(f"[LCMInterface] Subscribed to state channel: {state_channel}")
    
    def _on_state_received(self, channel, data):
        """LCM状态消息回调（内部使用）"""
        try:
            msg = development_state_t.decode(data)
            
            # 检查机器人ID是否匹配
            lcm_config = self.config.get('lcm', {})
            robot_id = lcm_config.get('robot_id', '')
            if robot_id and msg.robot_id != robot_id:
                return
            
            # 更新状态缓存
            with self.state_lock:
                self.latest_state = msg
                self.last_state_time = time.time()
            
            # 调用用户注册的回调函数
            if self.state_callback:
                try:
                    self.state_callback(msg)
                except Exception as e:
                    print(f"[LCMInterface] Error in state callback: {e}")
                    
        except Exception as e:
            print(f"[LCMInterface] Error processing state message: {e}")
    
    def register_state_callback(self, callback):
        """
        注册状态消息回调函数
        
        Args:
            callback: 回调函数，接收一个 development_state_t 消息作为参数
        """
        self.state_callback = callback
    
    def get_latest_state(self):
        """
        获取最新的状态消息
        
        Returns:
            development_state_t: 最新的状态消息，如果没有则返回None
        """
        with self.state_lock:
            return self.latest_state
    
    def send_command(self, enable_development_mode, is_rl_mode, 
                     joint_positions=None, joint_velocities=None, 
                     joint_torques=None, joint_kp=None, joint_kd=None):
        """
        发送控制命令
        
        Args:
            enable_development_mode: bool, 是否启用开发模式
            is_rl_mode: bool, 是否为强化学习模式
            joint_positions: dict, 关节位置，格式：
                {
                    'L_Leg_q': [6个值],
                    'R_Leg_q': [6个值],
                    'waist_q': float,
                    'L_Arm_q': [4个值],
                    'R_Arm_q': [4个值]
                }
            joint_velocities: dict, 关节速度，格式同上
            joint_torques: dict, 关节扭矩，格式同上
            joint_kp: dict, 关节Kp，格式同上
            joint_kd: dict, 关节Kd，格式同上
        """
        if self.lcm_instance is None:
            return
        
        try:
            lcm_config = self.config.get('lcm', {})
            if 'robot_id' not in lcm_config:
                raise KeyError("Configuration missing 'lcm.robot_id'. Please specify the robot ID.")
            
            msg = development_command_t()
            msg.robot_id = lcm_config['robot_id']
            msg.enable_development_mode = 1 if enable_development_mode else 0
            msg.is_rl_mode = 1 if is_rl_mode else 0
            
            if enable_development_mode and joint_positions is not None:
                # 填充关节位置
                msg.L_Leg_q = list(joint_positions.get('L_Leg_q', [0.0] * 6))
                msg.R_Leg_q = list(joint_positions.get('R_Leg_q', [0.0] * 6))
                msg.waist_q = joint_positions.get('waist_q', 0.0)
                msg.L_Arm_q = list(joint_positions.get('L_Arm_q', [0.0] * 4))
                msg.R_Arm_q = list(joint_positions.get('R_Arm_q', [0.0] * 4))
                
                # 填充关节速度
                if joint_velocities is not None:
                    msg.L_Leg_qd = list(joint_velocities.get('L_Leg_qd', [0.0] * 6))
                    msg.R_Leg_qd = list(joint_velocities.get('R_Leg_qd', [0.0] * 6))
                    msg.waist_qd = joint_velocities.get('waist_qd', 0.0)
                    msg.L_Arm_qd = list(joint_velocities.get('L_Arm_qd', [0.0] * 4))
                    msg.R_Arm_qd = list(joint_velocities.get('R_Arm_qd', [0.0] * 4))
                else:
                    msg.L_Leg_qd = [0.0] * 6
                    msg.R_Leg_qd = [0.0] * 6
                    msg.waist_qd = 0.0
                    msg.L_Arm_qd = [0.0] * 4
                    msg.R_Arm_qd = [0.0] * 4
                
                # 填充关节扭矩
                if joint_torques is not None:
                    msg.L_Leg_tau = list(joint_torques.get('L_Leg_tau', [0.0] * 6))
                    msg.R_Leg_tau = list(joint_torques.get('R_Leg_tau', [0.0] * 6))
                    msg.waist_tau = joint_torques.get('waist_tau', 0.0)
                    msg.L_Arm_tau = list(joint_torques.get('L_Arm_tau', [0.0] * 4))
                    msg.R_Arm_tau = list(joint_torques.get('R_Arm_tau', [0.0] * 4))
                else:
                    msg.L_Leg_tau = [0.0] * 6
                    msg.R_Leg_tau = [0.0] * 6
                    msg.waist_tau = 0.0
                    msg.L_Arm_tau = [0.0] * 4
                    msg.R_Arm_tau = [0.0] * 4
                
                # 填充Kp和Kd
                if joint_kp is not None:
                    msg.L_Leg_kp = list(joint_kp.get('L_Leg_kp', [0.0] * 6))
                    msg.R_Leg_kp = list(joint_kp.get('R_Leg_kp', [0.0] * 6))
                    msg.waist_kp = joint_kp.get('waist_kp', 0.0)
                    msg.L_Arm_kp = list(joint_kp.get('L_Arm_kp', [0.0] * 4))
                    msg.R_Arm_kp = list(joint_kp.get('R_Arm_kp', [0.0] * 4))
                else:
                    msg.L_Leg_kp = [0.0] * 6
                    msg.R_Leg_kp = [0.0] * 6
                    msg.waist_kp = 0.0
                    msg.L_Arm_kp = [0.0] * 4
                    msg.R_Arm_kp = [0.0] * 4
                
                if joint_kd is not None:
                    msg.L_Leg_kd = list(joint_kd.get('L_Leg_kd', [0.0] * 6))
                    msg.R_Leg_kd = list(joint_kd.get('R_Leg_kd', [0.0] * 6))
                    msg.waist_kd = joint_kd.get('waist_kd', 0.0)
                    msg.L_Arm_kd = list(joint_kd.get('L_Arm_kd', [0.0] * 4))
                    msg.R_Arm_kd = list(joint_kd.get('R_Arm_kd', [0.0] * 4))
                else:
                    msg.L_Leg_kd = [0.0] * 6
                    msg.R_Leg_kd = [0.0] * 6
                    msg.waist_kd = 0.0
                    msg.L_Arm_kd = [0.0] * 4
                    msg.R_Arm_kd = [0.0] * 4
            else:
                # 清零所有命令
                msg.L_Leg_q = [0.0] * 6
                msg.R_Leg_q = [0.0] * 6
                msg.waist_q = 0.0
                msg.L_Arm_q = [0.0] * 4
                msg.R_Arm_q = [0.0] * 4
                msg.L_Leg_qd = [0.0] * 6
                msg.R_Leg_qd = [0.0] * 6
                msg.waist_qd = 0.0
                msg.L_Arm_qd = [0.0] * 4
                msg.R_Arm_qd = [0.0] * 4
                msg.L_Leg_tau = [0.0] * 6
                msg.R_Leg_tau = [0.0] * 6
                msg.waist_tau = 0.0
                msg.L_Arm_tau = [0.0] * 4
                msg.R_Arm_tau = [0.0] * 4
                msg.L_Leg_kp = [0.0] * 6
                msg.R_Leg_kp = [0.0] * 6
                msg.waist_kp = 0.0
                msg.L_Arm_kp = [0.0] * 4
                msg.R_Arm_kp = [0.0] * 4
                msg.L_Leg_kd = [0.0] * 6
                msg.R_Leg_kd = [0.0] * 6
                msg.waist_kd = 0.0
                msg.L_Arm_kd = [0.0] * 4
                msg.R_Arm_kd = [0.0] * 4
            
            # 发布命令
            lcm_config = self.config.get('lcm', {})
            if 'command_channel' not in lcm_config:
                raise KeyError("Configuration missing 'lcm.command_channel'. Please specify the command channel name.")
            command_channel = lcm_config['command_channel']
            self.lcm_instance.publish(command_channel, msg.encode())
            
        except Exception as e:
            print(f"[LCMInterface] Error sending command: {e}")
            import traceback
            traceback.print_exc()
    
    def start(self):
        """启动LCM处理线程"""
        self.running = True
    
    def stop(self):
        """停止LCM处理"""
        self.running = False
    
    def handle(self, timeout_ms=100):
        """
        处理LCM消息（需要在主循环中调用）
        
        Args:
            timeout_ms: 超时时间（毫秒）
        """
        if self.running and self.lcm_instance:
            self.lcm_instance.handle_timeout(timeout_ms)

