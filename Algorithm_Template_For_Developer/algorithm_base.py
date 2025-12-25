#!/usr/bin/env python3
"""
算法基类
所有外部算法都应该继承这个基类并实现必要的方法
"""

import os
import sys
import time
import yaml
import numpy as np
import threading
from abc import ABC, abstractmethod

# 添加当前目录到路径，以便导入lcm_interface
_current_dir = os.path.dirname(os.path.abspath(__file__))
if _current_dir not in sys.path:
    sys.path.insert(0, _current_dir)

from lcm_interface import LCMInterface

# 尝试导入ONNX Runtime和PyTorch
try:
    import onnxruntime
    ONNX_AVAILABLE = True
except ImportError:
    ONNX_AVAILABLE = False

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

try:
    import onnx
    ONNX_PARSER_AVAILABLE = True
except ImportError:
    ONNX_PARSER_AVAILABLE = False


class AlgorithmBase(ABC):
    """算法基类"""
    
    def __init__(self, config_path):
        """
        初始化算法
        
        Args:
            config_path: 配置文件路径
        """
        # 处理配置文件路径（参考 mujoco_simulator.py）
        if not os.path.isabs(config_path):
            # 如果是相对路径，基于脚本所在目录
            script_dir = os.path.dirname(os.path.abspath(__file__))
            # 如果config_path是相对于算法目录的，先尝试算法目录
            algorithm_dir = os.path.dirname(script_dir)
            potential_path = os.path.join(algorithm_dir, config_path)
            if os.path.exists(potential_path):
                config_path = potential_path
            else:
                # 否则基于当前工作目录
                config_path = os.path.abspath(config_path)
        
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Configuration file not found: {config_path}")
        
        # 加载配置（参考 mujoco_simulator.py）
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)
        
        if self.config is None:
            raise ValueError(f"Configuration file is empty or invalid: {config_path}")
        
        self.config_dir = os.path.dirname(os.path.abspath(config_path))
        
        # 检查是否打印配置信息
        print_config = self.config.get('debug', {}).get('print_config', False)
        if print_config:
            print(f"[AlgorithmBase] Loaded config from: {config_path}")
            print("[AlgorithmBase] Configuration:")
            print("=" * 80)
            print(yaml.dump(self.config, default_flow_style=False, allow_unicode=True, sort_keys=False))
            print("=" * 80)
        
        # 初始化变量
        self.policy = None
        self.running = False
        self.development_mode_active = False
        self.execution_start_time = None
        self.lcm_started = False  # LCM接口启动标志
        
        # 线程同步锁
        self.state_lock = threading.Lock()  # 保护 development_mode_active, running, execution_start_time
        self.action_buffer_lock = threading.Lock()  # 保护 action_buffer
        self.latest_command_lock = threading.Lock()  # 保护 latest_command
        
        # LCM接口
        self.lcm_interface = LCMInterface(self.config)
        self.lcm_interface.register_state_callback(self._on_state_received)
        
        # 先初始化模型参数（_load_policy 中的 _load_onnx_metadata 可能需要访问）
        self._init_model_params()
        
        # 加载策略（可能会更新 model_params）
        self._load_policy()
        
        # 启动LCM接口以接收状态消息（预热需要真实状态）
        if not self.lcm_started:
            self.lcm_interface.start()
            self.lcm_started = True
        
        # 预热策略模型（使用真实状态）
        self._warmup_policy()
        
        # 执行控制
        self.execution_frequency = self.config['execution']['frequency']  # 策略推理频率（如50Hz）
        self.execution_period = 1.0 / self.execution_frequency if self.execution_frequency > 0 else 0.02
        self.lcm_send_frequency = self.config['execution'].get('lcm_send_frequency', 500.0)  # LCM发送频率（500Hz）
        self.lcm_send_period = 1.0 / self.lcm_send_frequency if self.lcm_send_frequency > 0 else 0.002
        self.auto_start = self.config['execution']['auto_start']
        self.auto_end = self.config['execution']['auto_end']
        self.max_execution_time = self.config['execution']['max_execution_time']
        
        # 动作缓冲区（用于某些需要历史动作的模型）
        self.action_buffer = np.zeros(self.model_params['num_actions'], dtype=np.float32)
        
        # 最新的控制命令缓存（用于500Hz发送）
        self.latest_command = None
        
        print(f"[{self.__class__.__name__}] Initialized: {self.config['algorithm_name']}")
        print(f"[{self.__class__.__name__}] Policy type: {self.config['policy']['type']}")
        print(f"[{self.__class__.__name__}] Policy inference frequency: {self.execution_frequency} Hz")
        print(f"[{self.__class__.__name__}] LCM send frequency: {self.lcm_send_frequency} Hz")
    
    def _load_policy(self):
        """加载策略模型"""
        policy_type = self.config['policy']['type']
        policy_path = self.config['policy']['path']
        
        # 处理相对路径
        if not os.path.isabs(policy_path):
            policy_path = os.path.join(self.config_dir, policy_path)
        
        if not os.path.exists(policy_path):
            raise FileNotFoundError(f"Policy file not found: {policy_path}")
        
        print(f"[{self.__class__.__name__}] Loading policy from: {policy_path}")
        
        if policy_type == "onnx":
            if not ONNX_AVAILABLE:
                raise RuntimeError("onnxruntime not available, cannot load ONNX model")
            
            self.policy = onnxruntime.InferenceSession(policy_path)
            self.policy_type = "onnx"
            
            # 读取ONNX metadata（如果启用）
            if self.config['policy'].get('read_metadata', False) and ONNX_PARSER_AVAILABLE:
                self._load_onnx_metadata(policy_path)
                
        elif policy_type == "pt":
            if not TORCH_AVAILABLE:
                raise RuntimeError("PyTorch not available, cannot load .pt model")
            
            self.policy = torch.jit.load(policy_path)
            self.policy.eval()
            self.policy_type = "pt"
        else:
            raise ValueError(f"Unknown policy type: {policy_type}")
        
        print(f"[{self.__class__.__name__}] Policy loaded successfully")
    
    def _warmup_policy(self):
        """预热策略模型（使用真实状态进行推理，检测action异常）"""
        warmup_count = self.config.get('policy', {}).get('warmup_count', 0)
        
        if warmup_count <= 0:
            print(f"[{self.__class__.__name__}] Skipping policy warmup (warmup_count={warmup_count})")
            return
        
        if self.policy is None:
            print(f"[{self.__class__.__name__}] Warning: Policy is None, cannot warmup")
            return
        
        # 获取action阈值配置
        action_threshold = self.config.get('policy', {}).get('action_threshold', 100.0)
        
        print(f"[{self.__class__.__name__}] Warming up policy with {warmup_count} real state inferences...")
        print(f"[{self.__class__.__name__}] Action threshold: {action_threshold}")
        
        try:
            last_obs = None
            last_action = None
            last_violations = None
            successful_inferences = 0
            
            # 进度条宽度
            bar_width = 50
            
            for i in range(warmup_count):
                # 等待接收状态消息
                state = None
                max_wait_time = 5.0  # 最多等待5秒
                wait_start = time.time()
                
                while state is None:
                    # 处理LCM消息
                    self.lcm_interface.handle(10)  # 100ms超时
                    
                    # 获取最新状态
                    state = self.lcm_interface.get_latest_state()
                    
                    # 检查超时
                    if time.time() - wait_start > max_wait_time:
                        break
                
                if state is None:
                    continue
                
                # 计算观测（使用子类的compute_observation方法）
                obs = self.compute_observation(state)
                
                if obs is None:
                    continue
                
                # 运行推理
                action = self._run_inference(obs)
                
                if action is None:
                    continue
                
                successful_inferences += 1
                
                # 检测action异常
                violations = self._check_action(action, action_threshold)
                
                if violations:
                    # 打印异常信息
                    print(f"\n[{self.__class__.__name__}] Action anomaly detected in warmup iteration {i+1}/{warmup_count}:")
                    print(f"  Action: {action}")
                    print(f"  Violations (indices exceeding threshold {action_threshold}):")
                    for idx, value in violations:
                        print(f"    Index {idx}: {value} (threshold: {action_threshold})")
                    
                    # 保存最后一帧的信息
                    last_obs = obs.copy()
                    last_action = action.copy()
                    last_violations = violations
                
                # 更新进度条
                progress = (i + 1) / warmup_count
                filled = int(bar_width * progress)
                bar = '=' * filled + '-' * (bar_width - filled)
                percent = progress * 100
                print(f'\r[{self.__class__.__name__}] Warmup: [{bar}] {percent:.1f}% ({i+1}/{warmup_count}, {successful_inferences} successful)', end='', flush=True)
            
            # 完成进度条
            print()  # 换行
            
            # 检查最后一帧是否异常
            if last_violations:
                print(f"[{self.__class__.__name__}] ERROR: Action anomaly detected in final warmup iteration!")
                print(f"[{self.__class__.__name__}] Final observation:")
                print(f"  Obs shape: {last_obs.shape}")
                print(f"  Obs values: {last_obs}")
                print(f"[{self.__class__.__name__}] Final action:")
                print(f"  Action shape: {last_action.shape}")
                print(f"  Action values: {last_action}")
                print(f"[{self.__class__.__name__}] Violations:")
                for idx, value in last_violations:
                    print(f"    Index {idx}: {value} (threshold: {action_threshold})")
                print(f"[{self.__class__.__name__}] Terminating program due to action anomaly in warmup")
                sys.exit(1)
            
            print(f"[{self.__class__.__name__}] Policy warmup completed successfully ({successful_inferences}/{warmup_count} successful inferences)")
            
        except KeyboardInterrupt:
            print(f"\n[{self.__class__.__name__}] Warmup interrupted by user")
            sys.exit(1)
        except Exception as e:
            print(f"\n[{self.__class__.__name__}] Error during policy warmup: {e}")
            import traceback
            traceback.print_exc()
            sys.exit(1)
    
    def _check_action(self, action, threshold):
        """
        检查action是否超出阈值
        
        Args:
            action: np.ndarray, 模型输出的action
            threshold: float, 阈值
            
        Returns:
            list: 异常索引和值的列表，格式为 [(index, value), ...]，如果无异常返回空列表
        """
        violations = []
        
        for idx, value in enumerate(action):
            if abs(value) > threshold:
                violations.append((idx, value))
        
        return violations
    
    def _parse_metadata_float_array(self, value):
        """
        解析metadata中的浮点数组值
        支持两种格式：
        1. 逗号分隔: "1.0,2.0,3.0"
        2. 列表字符串: "[1.0, 2.0, 3.0]" 或 "[-0.0883508175611496, ...]"
        """
        # 去除首尾空白字符
        value = value.strip()
        
        # 如果是列表格式，去除外层方括号
        if value.startswith('[') and value.endswith(']'):
            value = value[1:-1].strip()
        
        # 按逗号分割并转换为浮点数
        # 过滤空字符串（处理多个连续逗号或空格的情况）
        parts = []
        for x in value.split(","):
            x = x.strip()
            if not x:
                continue
            # 去除每个部分可能残留的方括号（处理边缘情况）
            if x.startswith('['):
                x = x[1:]
            if x.endswith(']'):
                x = x[:-1]
            x = x.strip()
            if x:
                parts.append(x)
        
        return np.array([float(x) for x in parts], dtype=np.float32)
    
    def _parse_metadata_string_array(self, value):
        """
        解析metadata中的字符串数组值
        支持两种格式：
        1. 逗号分隔: "joint1,joint2,joint3"
        2. 列表字符串: "['joint1', 'joint2', 'joint3']" 或 '["joint1", "joint2"]'
        """
        # 去除首尾空白字符
        value = value.strip()
        
        # 如果是列表格式，去除外层方括号
        if value.startswith('[') and value.endswith(']'):
            value = value[1:-1].strip()
        
        # 按逗号分割并处理每个部分
        parts = []
        for part in value.split(","):
            part = part.strip()
            if not part:
                continue
            # 去除每个部分可能残留的方括号（处理边缘情况）
            if part.startswith('['):
                part = part[1:]
            if part.endswith(']'):
                part = part[:-1]
            part = part.strip()
            # 去除引号（单引号或双引号）
            if (part.startswith("'") and part.endswith("'")) or (part.startswith('"') and part.endswith('"')):
                part = part[1:-1]
            if part:
                parts.append(part)
        
        return parts
    
    def _load_onnx_metadata(self, policy_path):
        """从ONNX模型metadata读取配置"""
        try:
            model = onnx.load(policy_path)
            loaded_metadata = {}
            
            for prop in model.metadata_props:
                key = prop.key
                value = prop.value
                
                if key == "joint_names":
                    joint_seq = self._parse_metadata_string_array(value)
                    self.config['joint_mapping']['model_joint_order'] = joint_seq
                    loaded_metadata['joint_names'] = {
                        'count': len(joint_seq),
                        'joints': joint_seq
                    }
                    
                elif key == "default_joint_pos":
                    joint_pos_array_seq = self._parse_metadata_float_array(value)
                    self.model_params['default_joint_pos'] = joint_pos_array_seq
                    loaded_metadata['default_joint_pos'] = {
                        'count': len(joint_pos_array_seq),
                        'values': joint_pos_array_seq.tolist()
                    }
                    
                elif key == "joint_stiffness":
                    stiffness_array_seq = self._parse_metadata_float_array(value)
                    self.model_params['joint_stiffness'] = stiffness_array_seq
                    loaded_metadata['joint_stiffness'] = {
                        'count': len(stiffness_array_seq),
                        'values': stiffness_array_seq.tolist()
                    }
                    
                elif key == "joint_damping":
                    damping_array_seq = self._parse_metadata_float_array(value)
                    self.model_params['joint_damping'] = damping_array_seq
                    loaded_metadata['joint_damping'] = {
                        'count': len(damping_array_seq),
                        'values': damping_array_seq.tolist()
                    }
                    
                elif key == "action_scale":
                    action_scale = self._parse_metadata_float_array(value)
                    self.model_params['action_scale'] = action_scale
                    loaded_metadata['action_scale'] = {
                        'count': len(action_scale),
                        'values': action_scale.tolist()
                    }
                    
                elif key == "num_actions":
                    self.model_params['num_actions'] = int(value)
                    loaded_metadata['num_actions'] = int(value)
                    
                elif key == "num_obs":
                    self.model_params['num_obs'] = int(value)
                    loaded_metadata['num_obs'] = int(value)
            
            # 检查是否打印metadata信息
            print_metadata = self.config.get('debug', {}).get('print_metadata', False)
            if print_metadata:
                # 打印所有从metadata中加载的配置
                if loaded_metadata:
                    print(f"[{self.__class__.__name__}] Loaded ONNX metadata from: {policy_path}")
                    print("[{0}] ONNX Metadata Configuration:".format(self.__class__.__name__))
                    print("=" * 80)
                    print(yaml.dump(loaded_metadata, default_flow_style=False, allow_unicode=True, sort_keys=False))
                    print("=" * 80)
                else:
                    print(f"[{self.__class__.__name__}] No metadata found in ONNX model")
                    
        except Exception as e:
            print(f"[{self.__class__.__name__}] Warning: Failed to load ONNX metadata: {e}")
    
    def _init_model_params(self):
        """初始化模型参数"""
        self.model_params = self.config.get('model_params', {})
        
        # 确保参数存在
        if 'num_actions' not in self.model_params:
            self.model_params['num_actions'] = 21
        if 'num_obs' not in self.model_params:
            self.model_params['num_obs'] = 114
        
        # 转换为numpy数组
        for key in ['default_joint_pos', 'joint_stiffness', 'joint_damping', 'action_scale']:
            if key in self.model_params:
                if isinstance(self.model_params[key], list):
                    self.model_params[key] = np.array(self.model_params[key], dtype=np.float32)
                else:
                    self.model_params[key] = np.array([self.model_params[key]] * self.model_params['num_actions'], dtype=np.float32)
    
    def _on_state_received(self, state_msg):
        """状态消息回调（子类可以重写）"""
        # 检查属性是否已初始化（可能在预热阶段回调时还未初始化）
        if not hasattr(self, 'auto_start') or not hasattr(self, 'development_mode_active'):
            return

        # 如果自动开始且未激活，则开始开发模式
        # 注意：_start_development_mode 内部已加锁，这里不要再持锁调用，避免死锁
        if self.auto_start:
            self._start_development_mode()
    
    def _start_development_mode(self):
        """开始开发模式（线程安全）"""
        with self.state_lock:
            if self.development_mode_active:
                return
            
            print(f"[{self.__class__.__name__}] Starting development mode...")
            self.development_mode_active = True
            self.execution_start_time = time.time()
        
        # 调用子类的初始化方法（在锁外调用，避免死锁）
        self.on_development_mode_start()
        
        # 初始化命令缓存（空命令，只是启用标志）
        with self.latest_command_lock:
            self.latest_command = {
                'enable_development_mode': True,
                'is_rl_mode': self.config['rl_mode']['is_rl_mode']
            }
    
    def _end_development_mode(self):
        """结束开发模式（线程安全）"""
        with self.state_lock:
            if not self.development_mode_active:
                return
            
            print(f"[{self.__class__.__name__}] Ending development mode...")
            self.development_mode_active = False
        
        # 调用子类的清理方法（在锁外调用，避免死锁）
        self.on_development_mode_end()
        
        # 更新命令缓存为结束命令（清零所有命令）
        with self.latest_command_lock:
            self.latest_command = {
                'enable_development_mode': False,
                'is_rl_mode': False
            }
    
    def _run_inference(self, obs):
        """运行模型推理"""
        if self.policy is None:
            return None
        
        # 检查观测是否有效
        if obs is None:
            print(f"[{self.__class__.__name__}] WARNING: Invalid observation (None), skipping inference")
            return None
        
        # 检查观测数据范围
        obs_min = np.min(obs)
        obs_max = np.max(obs)
        obs_mean = np.mean(obs)
        obs_std = np.std(obs)
        
        if np.any(np.isnan(obs)) or np.any(np.isinf(obs)):
            print(f"[{self.__class__.__name__}] WARNING: Observation contains NaN or Inf, skipping inference")
            print(f"  obs stats: min={obs_min}, max={obs_max}, mean={obs_mean}, std={obs_std}")
            return None
        
        # 检查观测数据是否过大（可能导致推理溢出）
        obs_clip_threshold = 100.0  # 如果观测值超过100，可能有问题
        if obs_max > obs_clip_threshold or obs_min < -obs_clip_threshold:
            print(f"[{self.__class__.__name__}] WARNING: Observation values too large (min={obs_min}, max={obs_max}), clipping...")
            obs = np.clip(obs, -obs_clip_threshold, obs_clip_threshold)
        
        try:
            if self.policy_type == "onnx":
                # ONNX推理
                input_name = self.policy.get_inputs()[0].name
                obs_input = obs.reshape(1, -1).astype(np.float32)
                
                # 检查是否有时间步输入
                inputs = {input_name: obs_input}
                input_names = [inp.name for inp in self.policy.get_inputs()]
                if 'time_step' in input_names:
                    # 如果有时间步，需要提供（这里简化处理）
                    inputs['time_step'] = np.array([[0]], dtype=np.float32)
                
                outputs = self.policy.run(None, inputs)
                action = outputs[0].flatten()
                
            elif self.policy_type == "pt":
                # PyTorch推理
                obs_tensor = torch.from_numpy(obs).unsqueeze(0).float()
                with torch.no_grad():
                    action = self.policy(obs_tensor).cpu().numpy().flatten()
            else:
                return None
            
            action = action.astype(np.float32)
            
            # 检查推理结果是否包含 NaN 或 Inf
            if np.any(np.isnan(action)) or np.any(np.isinf(action)):
                print(f"[{self.__class__.__name__}] ERROR: Inference output contains NaN or Inf: {action}")
                return None
            
            return action
            
        except Exception as e:
            print(f"[{self.__class__.__name__}] Error during inference: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def _execution_loop(self):
        """执行循环"""
        print(f"[{self.__class__.__name__}] Execution loop started")
        
        while True:
            # 线程安全地检查 running 标志
            with self.state_lock:
                if not self.running:
                    break
            
            try:
                # 检查是否超时（线程安全地读取 execution_start_time）
                with self.state_lock:
                    execution_start = self.execution_start_time
                    if self.max_execution_time > 0 and execution_start is not None:
                        elapsed = time.time() - execution_start
                        if elapsed > self.max_execution_time:
                            print(f"[{self.__class__.__name__}] Max execution time reached ({self.max_execution_time}s)")
                            if self.auto_end:
                                self._end_development_mode()
                            break
                
                # 获取最新状态
                state = self.lcm_interface.get_latest_state()
                
                # 线程安全地检查 development_mode_active
                with self.state_lock:
                    dev_mode_active = self.development_mode_active
                
                if state is None or not dev_mode_active:
                    time.sleep(self.execution_period)
                    continue
                
                # 计算观测（子类实现）
                obs = self.compute_observation(state)
                
                if obs is None:
                    time.sleep(self.execution_period)
                    continue
                
                # 检查观测是否包含 NaN 或 Inf
                if np.any(np.isnan(obs)) or np.any(np.isinf(obs)):
                    print(f"[{self.__class__.__name__}] WARNING: Observation contains NaN/Inf, skipping inference")
                    time.sleep(self.execution_period)
                    continue
                
                # 运行推理
                action = self._run_inference(obs)
                
                if action is not None:
                    # 再次检查 action 是否有效（防止推理返回 NaN）
                    if np.any(np.isnan(action)) or np.any(np.isinf(action)):
                        print(f"[{self.__class__.__name__}] WARNING: Inference returned NaN/Inf action, skipping")
                        time.sleep(self.execution_period)
                        continue
                    
                    # 更新动作缓冲区（线程安全）
                    with self.action_buffer_lock:
                        self.action_buffer = action.copy()
                    
                    # 处理动作并更新命令缓存（子类实现，不直接发送）
                    self.process_action(state, action)
                
                # 控制策略推理频率
                time.sleep(self.execution_period)
                
            except KeyboardInterrupt:
                print(f"[{self.__class__.__name__}] Execution loop interrupted")
                break
            except Exception as e:
                print(f"[{self.__class__.__name__}] Error in execution loop: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(self.execution_period)
        
        print(f"[{self.__class__.__name__}] Execution loop ended")
    
    def _lcm_send_loop(self):
        """LCM发送循环（500Hz）"""
        print(f"[{self.__class__.__name__}] LCM send loop started ({self.lcm_send_frequency} Hz)")
        
        # 用于跟踪结束命令发送次数
        end_command_sent_count = 0
        end_command_target_count = int(self.lcm_send_frequency * 0.1)  # 发送100ms（约50次@500Hz）
        
        while True:
            # 线程安全地检查 running 标志
            with self.state_lock:
                running = self.running
            
            if not running and end_command_sent_count >= end_command_target_count:
                break
            try:
                # 获取最新的命令并发送
                with self.latest_command_lock:
                    command = self.latest_command
                
                if command is not None:
                    # 线程安全地检查 development_mode_active
                    with self.state_lock:
                        dev_mode_active = self.development_mode_active
                    
                    # 如果命令中包含 enable_development_mode=False，即使 development_mode_active 为 False 也要发送
                    # 这样可以确保结束命令被发送
                    should_send = (dev_mode_active or 
                                 (not command.get('enable_development_mode', True)))
                    if should_send:
                        # 发送命令
                        self.lcm_interface.send_command(**command)
                        # 如果是结束命令，增加计数
                        if not command.get('enable_development_mode', True):
                            end_command_sent_count += 1
                
                # 控制LCM发送频率（500Hz）
                time.sleep(self.lcm_send_period)
                
            except KeyboardInterrupt:
                print(f"[{self.__class__.__name__}] LCM send loop interrupted")
                break
            except Exception as e:
                print(f"[{self.__class__.__name__}] Error in LCM send loop: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(self.lcm_send_period)
        
        print(f"[{self.__class__.__name__}] LCM send loop ended")
    
    def run(self):
        """运行算法"""
        print(f"[{self.__class__.__name__}] Starting algorithm runner...")
        
        # 线程安全地设置 running 标志
        with self.state_lock:
            self.running = True
        
        # LCM接口已在_warmup_policy中启动，这里不需要再次启动
        if not self.lcm_started:
            self.lcm_interface.start()
            self.lcm_started = True
        
        # 启动策略推理线程（按配置频率运行）
        execution_thread = threading.Thread(target=self._execution_loop, daemon=True)
        execution_thread.start()
        
        # 启动LCM发送线程（500Hz）
        lcm_send_thread = threading.Thread(target=self._lcm_send_loop, daemon=True)
        lcm_send_thread.start()
        
        try:
            # LCM处理循环（主线程）
            while True:
                # 线程安全地检查 running 标志
                with self.state_lock:
                    if not self.running:
                        break
                
                self.lcm_interface.handle(10)  # 100ms超时
        except KeyboardInterrupt:
            print(f"\n[{self.__class__.__name__}] Shutting down...")
        finally:
            # 线程安全地设置 running 标志
            with self.state_lock:
                self.running = False
                dev_mode_active = self.development_mode_active
            
            if dev_mode_active:
                self._end_development_mode()
            self.lcm_interface.stop()
            print(f"[{self.__class__.__name__}] Shutdown complete")
    
    # ========== 抽象方法（子类必须实现）==========
    
    @abstractmethod
    def compute_observation(self, state):
        """
        根据状态计算观测
        
        Args:
            state: development_state_t 消息对象
            
        Returns:
            np.ndarray: 观测向量
        """
        pass
    
    @abstractmethod
    def process_action(self, state, action):
        """
        处理动作并发送控制命令
        
        Args:
            state: development_state_t 消息对象
            action: np.ndarray, 模型输出的动作向量
        """
        pass
    
    # ========== 可选方法（子类可以重写）==========
    
    def on_development_mode_start(self):
        """开发模式开始时的回调（可选）"""
        pass
    
    def on_development_mode_end(self):
        """开发模式结束时的回调（可选）"""
        pass


