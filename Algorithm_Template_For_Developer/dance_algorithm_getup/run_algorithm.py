#!/usr/bin/env python3
"""
跳舞算法实现
基于 deploy_mujoco_1Step.py，通过LCM接收机器人状态，执行跳舞动作
"""

import sys
import os
import numpy as np

# 添加父目录到路径
_parent_dir = os.path.join(os.path.dirname(__file__), '..')
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)

from algorithm_base import AlgorithmBase


class DanceAlgorithm(AlgorithmBase):
    """跳舞算法类"""
    
    def __init__(self, config_path):
        """初始化跳舞算法"""
        # 先读取配置以获取 joint_xml_order（在调用 super().__init__ 之前）
        # 因为 super().__init__ 会调用 _load_policy()，而 _load_onnx_metadata() 需要这些属性
        import yaml
        with open(config_path, 'r', encoding='utf-8') as f:
            temp_config = yaml.safe_load(f)
        
        # 检查是否打印配置信息
        print_config = temp_config.get('debug', {}).get('print_config', False)
        if print_config:
            # 打印读取的配置（在父类打印之前）
            print(f"[DanceAlgorithm] Reading config from: {config_path}")
            print("[DanceAlgorithm] Configuration:")
            print("=" * 80)
            print(yaml.dump(temp_config, default_flow_style=False, allow_unicode=True, sort_keys=False))
            print("=" * 80)
        
        # 初始化必要的属性
        self.joint_xml_order = temp_config.get('dance', {}).get('joint_xml_order', [])
        if not self.joint_xml_order:
            raise ValueError("Configuration missing 'dance.joint_xml_order'")
        
        self.joint_seq = []  # 从metadata读取
        self.joint_pos_array_seq = None
        self.joint_pos_array = None
        
        # 在调用父类初始化之前，先初始化可能被预热阶段访问的属性
        self.execution_complete = False  # 执行完成标志
        self.timestep = temp_config.get('dataset', {}).get('start_timestep', 50)
        self.loop = False  # 不循环，执行完就结束
        self.motion_input_pos = None  # 数据集加载后初始化
        self.motion_input_vel = None
        self.motion_pos = None
        self.motion_quat = None
        self.yaw_quat = None
        self.robot_initial_yaw = None  # 记录robot的初始yaw
        self.motion_initial_yaw = None  # 记录motion的初始yaw
        
        # 调用父类初始化（会加载配置并调用 _load_policy，可能会调用 _warmup_policy）
        super().__init__(config_path)
        
        # 加载数据集（在预热之后）
        self._load_dataset()
        
        # 重新初始化状态（使用实际配置）
        self.timestep = self.config['dataset'].get('start_timestep', 50)
        
        # 初始yaw四元数（用于对齐）
        self._init_yaw_quat()
        
        print(f"[DanceAlgorithm] Initialized with dataset: {self.dataset_path}")
        print(f"[DanceAlgorithm] Start timestep: {self.timestep}, Loop: {self.loop}")
    
    def _load_dataset(self):
        """加载数据集"""
        dataset_path = self.config['dataset']['path']
        if not os.path.isabs(dataset_path):
            dataset_path = os.path.join(self.config_dir, dataset_path)
        
        if not os.path.exists(dataset_path):
            raise FileNotFoundError(f"Dataset file not found: {dataset_path}")
        
        self.dataset_path = dataset_path
        self.motion_data = np.load(dataset_path)
        
        self.motion_pos = self.motion_data["body_pos_w"]
        self.motion_quat = self.motion_data["body_quat_w"]
        self.motion_input_pos = self.motion_data["joint_pos"]
        self.motion_input_vel = self.motion_data["joint_vel"]
        
        print(f"[DanceAlgorithm] Dataset loaded: {self.motion_input_pos.shape[0]} frames")
    
    def _init_yaw_quat(self):
        """
        初始化yaw四元数（用于对齐）
        与deploy_mujoco_1Step.py中的实现一致：
        - 使用motion_quat[0, 0, :]作为初始帧
        - 计算初始帧的yaw角度
        - 构造仅绕z轴旋转的四元数
        """
        if self.motion_quat is not None and self.motion_quat.shape[0] > 0:
            # 使用第0帧作为初始帧（与deploy_mujoco_1Step.py一致）
            motion_initial_quat = self.motion_quat[0, 0, :]
            # 计算motion初始帧的yaw角度
            _, _, self.motion_initial_yaw = self._quat_to_euler(motion_initial_quat)
            print(f"[DanceAlgorithm] Motion initial yaw: {self.motion_initial_yaw:.4f} rad")
            # 注意：yaw_quat 会在开发模式开始时根据robot初始yaw计算
            self.yaw_quat = None
        else:
            # 如果数据集还没有加载，设置默认值
            self.yaw_quat = None
            self.motion_initial_yaw = None
            print("[DanceAlgorithm] WARNING: motion_quat not available, yaw_quat set to None")
    
    def _load_onnx_metadata(self, policy_path):
        """重写metadata加载，保存关节顺序信息"""
        super()._load_onnx_metadata(policy_path)
        
        # 保存关节顺序信息
        if 'joint_mapping' in self.config and 'model_joint_order' in self.config['joint_mapping']:
            self.joint_seq = self.config['joint_mapping']['model_joint_order']
            
            # 计算关节位置数组映射（确保 model_params 和 joint_xml_order 已初始化）
            if (self.joint_seq and 
                hasattr(self, 'model_params') and 
                'default_joint_pos' in self.model_params and
                hasattr(self, 'joint_xml_order') and 
                self.joint_xml_order):
                joint_pos_array_seq = self.model_params['default_joint_pos']
                # 映射到XML顺序
                try:
                    self.joint_pos_array = np.array([
                        joint_pos_array_seq[self.joint_seq.index(joint)] 
                        if joint in self.joint_seq else 0.0
                        for joint in self.joint_xml_order
                    ])
                    self.joint_pos_array_seq = np.array(joint_pos_array_seq)
                    print(f"[DanceAlgorithm] Joint position array mapped: {len(self.joint_pos_array)} joints")
                except Exception as e:
                    print(f"[DanceAlgorithm] Warning: Failed to map joint positions: {e}")
    
    def _quat_to_euler(self, q):
        """四元数转欧拉角 (w, x, y, z) -> (roll, pitch, yaw)"""
        w, x, y, z = q[0], q[1], q[2], q[3]
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * (np.pi / 2)
        else:
            pitch = np.arcsin(sinp)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw
    
    def _quat_from_yaw(self, yaw):
        """从yaw角度创建四元数"""
        half = 0.5 * yaw
        return np.array([np.cos(half), 0.0, 0.0, np.sin(half)], dtype=np.float64)
    
    def _quaternion_multiply(self, q1, q2):
        """四元数乘法"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return np.array([w, x, y, z], dtype=np.float64)
    
    def _quaternion_conjugate(self, q):
        """四元数共轭"""
        return np.array([q[0], -q[1], -q[2], -q[3]])
    
    def _quat_to_rotmat(self, quat):
        """
        将四元数转换为旋转矩阵（3x3）
        与mujoco.mju_quat2Mat等效的实现
        """
        w, x, y, z = quat[0], quat[1], quat[2], quat[3]
        # 旋转矩阵的第一列
        r00 = 1 - 2 * (y*y + z*z)
        r10 = 2 * (x*y + w*z)
        r20 = 2 * (x*z - w*y)
        # 旋转矩阵的第二列
        r01 = 2 * (x*y - w*z)
        r11 = 1 - 2 * (x*x + z*z)
        r21 = 2 * (y*z + w*x)
        # 旋转矩阵的第三列
        r02 = 2 * (x*z + w*y)
        r12 = 2 * (y*z - w*x)
        r22 = 1 - 2 * (x*x + y*y)
        return np.array([[r00, r01, r02],
                         [r10, r11, r12],
                         [r20, r21, r22]], dtype=np.float64)
    
    def _subtract_frame_transforms(self, pos_a, quat_a, pos_b, quat_b):
        """
        计算从坐标系A到坐标系B的相对变换
        与deploy_mujoco_1Step.py中的subtract_frame_transforms_mujoco完全一致
        """
        # 计算相对位置: pos_B_to_A = R_A^T * (pos_B - pos_A)
        rotm_a = self._quat_to_rotmat(quat_a)
        rel_pos = rotm_a.T @ (pos_b - pos_a)
        
        # 计算相对旋转: quat_B_to_A = quat_A^* ⊗ quat_B
        rel_quat = self._quaternion_multiply(
            self._quaternion_conjugate(quat_a), 
            quat_b
        )
        # 归一化，防止除以零
        quat_norm = np.linalg.norm(rel_quat)
        if quat_norm > 1e-6:
            rel_quat = rel_quat / quat_norm
        else:
            # 如果归一化失败，返回单位四元数
            rel_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        return rel_pos, rel_quat
    
    def _quat_rotate_inverse(self, q, v):
        """四元数逆旋转向量"""
        q_w = q[0]
        q_vec = q[1:]
        a = v * (2.0 * q_w**2 - 1.0)
        b = np.cross(q_vec, v) * q_w * 2.0
        dot_product = np.dot(q_vec, v)
        c = q_vec * dot_product * 2.0
        return a - b + c
    
    def compute_observation(self, state):
        """
        计算观测（基于deploy_mujoco_1Step.py的逻辑）
        
        观测结构：
        - motioninput (num_actions*2): 动作序列的位置和速度
        - anchor_ori (6): anchor orientation的前两列
        - angvel (3): 角速度
        - qpos (num_actions): 关节位置（相对于默认位置）
        - qvel (num_actions): 关节速度
        - action_buffer (num_actions): 历史动作
        """
        # 如果执行完成，不再计算观测
        if hasattr(self, 'execution_complete') and self.execution_complete:
            return None
        
        # 如果数据集还没有加载（预热阶段），返回None
        if self.motion_input_pos is None or self.motion_input_vel is None:
            return None
        
        obs = np.zeros(self.model_params['num_obs'], dtype=np.float32)
        
        # 获取当前时间步的动作数据
        if self.timestep >= self.motion_input_pos.shape[0]:
            # 执行完成，不再返回观测
            self.execution_complete = True
            return None
        
        motion_input = np.concatenate([
            self.motion_input_pos[self.timestep, :],
            self.motion_input_vel[self.timestep, :]
        ], axis=0)
        
        motion_pos_current = self.motion_pos[self.timestep, 0, :]
        motion_quat_current = self.motion_quat[self.timestep, 0, :]
        
        # 计算anchor orientation
        robot_pos = np.array([0.0, 0.0, 0.0])  # 简化：假设在原点
        robot_quat = np.array([
            state.quat[0], state.quat[1], state.quat[2], state.quat[3]
        ])
        
        # 确保四元数有效
        if self.yaw_quat is None:
            print("[DanceAlgorithm] WARNING: yaw_quat is None, using identity quaternion")
            self.yaw_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        
        # 归一化四元数
        robot_quat_norm = np.linalg.norm(robot_quat)
        if robot_quat_norm > 1e-6:
            robot_quat = robot_quat / robot_quat_norm
        else:
            print("[DanceAlgorithm] WARNING: robot_quat norm too small, using identity")
            robot_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        
        robot_quat_adj = self._quaternion_multiply(self.yaw_quat, robot_quat)
        # 归一化结果
        quat_adj_norm = np.linalg.norm(robot_quat_adj)
        if quat_adj_norm > 1e-6:
            robot_quat_adj = robot_quat_adj / quat_adj_norm
        else:
            robot_quat_adj = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        
        _, anchor_quat = self._subtract_frame_transforms(
            robot_pos, robot_quat_adj,
            motion_pos_current, motion_quat_current
        )
        
        # 归一化 anchor_quat
        anchor_quat_norm = np.linalg.norm(anchor_quat)
        if anchor_quat_norm > 1e-6:
            anchor_quat = anchor_quat / anchor_quat_norm
        else:
            anchor_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        
        # 将anchor_quat转换为旋转矩阵的前两列（与deploy_mujoco_1Step.py一致）
        # 使用与mujoco.mju_quat2Mat等效的方法
        anchor_ori_flat = np.zeros(9, dtype=np.float64)
        anchor_rotmat = self._quat_to_rotmat(anchor_quat)
        anchor_ori_flat = anchor_rotmat.flatten()  # 3x3矩阵展平为9个元素
        anchor_ori = anchor_ori_flat.reshape(3, 3)[:, :2].reshape(-1)  # 取前两列，展平为6个元素
        anchor_ori = anchor_ori.astype(np.float32)
        
        # 构建观测
        offset = 0
        
        # 1. motioninput (num_actions*2)
        obs[offset:offset + self.model_params['num_actions']*2] = motion_input
        offset += self.model_params['num_actions']*2
        
        # 2. anchor_ori (6)
        obs[offset:offset + 6] = anchor_ori
        offset += 6
        
        # 3. angvel (3) - 角速度（body坐标系）
        # 将世界坐标系的角速度转换到body坐标系（与deploy_mujoco_1Step.py中的quat_rotate_inverse_np一致）
        omega_body = np.array([state.omega[0], state.omega[1], state.omega[2]], dtype=np.float64)

        obs[offset:offset + 3] = omega_body.astype(np.float32)
        offset += 3
        
        # 4. qpos (num_actions) - 关节位置（相对于默认位置）
        joint_pos = np.concatenate([
            state.L_Leg_q,
            state.R_Leg_q,
            [state.waist_q],
            state.L_Arm_q,
            state.R_Arm_q
        ])
        
        # 映射到模型关节顺序
        if self.joint_seq and self.joint_pos_array_seq is not None:
            # 从XML顺序映射到模型顺序
            joint_pos_model = np.array([
                joint_pos[self.joint_xml_order.index(joint)] 
                if joint in self.joint_xml_order else 0.0
                for joint in self.joint_seq
            ])
            obs[offset:offset + self.model_params['num_actions']] = (
                joint_pos_model - self.joint_pos_array_seq[:self.model_params['num_actions']]
            )
        else:
            default_pos = self.model_params['default_joint_pos']
            obs[offset:offset + self.model_params['num_actions']] = (
                joint_pos[:self.model_params['num_actions']] - default_pos[:self.model_params['num_actions']]
            )
        offset += self.model_params['num_actions']
        
        # 5. qvel (num_actions) - 关节速度
        joint_vel = np.concatenate([
            state.L_Leg_qd,
            state.R_Leg_qd,
            [state.waist_qd],
            state.L_Arm_qd,
            state.R_Arm_qd
        ])
        
        # 映射到模型关节顺序
        if self.joint_seq:
            joint_vel_model = np.array([
                joint_vel[self.joint_xml_order.index(joint)] 
                if joint in self.joint_xml_order else 0.0
                for joint in self.joint_seq
            ])
            obs[offset:offset + self.model_params['num_actions']] = joint_vel_model
        else:
            obs[offset:offset + self.model_params['num_actions']] = joint_vel[:self.model_params['num_actions']]
        offset += self.model_params['num_actions']
        
        # 6. action_buffer (num_actions) - 线程安全地读取
        with self.action_buffer_lock:
            obs[offset:offset + self.model_params['num_actions']] = self.action_buffer.copy()
        offset += self.model_params['num_actions']
        
        # 检查观测数据是否包含异常值
        if np.any(np.isnan(obs)) or np.any(np.isinf(obs)):
            print(f"[DanceAlgorithm] WARNING: Observation contains NaN/Inf before clipping")
            print(f"  obs min: {np.min(obs)}, max: {np.max(obs)}")
            print(f"  motion_input range: [{np.min(motion_input)}, {np.max(motion_input)}]")
            print(f"  anchor_ori range: [{np.min(anchor_ori)}, {np.max(anchor_ori)}]")
            print(f"  omega_body range: [{np.min(omega_body)}, {np.max(omega_body)}]")
            print(f"  joint_pos_model range: [{np.min(joint_pos_model) if 'joint_pos_model' in locals() else 'N/A'}, {np.max(joint_pos_model) if 'joint_pos_model' in locals() else 'N/A'}]")
            print(f"  joint_vel_model range: [{np.min(joint_vel_model) if 'joint_vel_model' in locals() else 'N/A'}, {np.max(joint_vel_model) if 'joint_vel_model' in locals() else 'N/A'}]")
            with self.action_buffer_lock:
                action_buffer_copy = self.action_buffer.copy()
            print(f"  action_buffer range: [{np.min(action_buffer_copy)}, {np.max(action_buffer_copy)}]")
        
        # 裁剪观测数据到合理范围（防止数值溢出）
        obs_clip_range = self.model_params.get('obs_clip_range', 10.0)  # 默认裁剪到 [-10, 10]
        obs = np.clip(obs, -obs_clip_range, obs_clip_range)
        
        return obs
    
    def process_action(self, state, action):
        """
        处理动作并发送控制命令
        
        动作处理流程：
        1. 将动作缩放并加上默认位置
        2. 映射回XML关节顺序
        3. 发送控制命令
        """
        # 如果执行完成，不再处理动作
        if self.execution_complete:
            return
        
        # 检查 action 是否有效
        if action is None or len(action) == 0:
            print("[DanceAlgorithm] WARNING: Invalid action received, skipping...")
            return
        
        # 检查 action 中是否包含 NaN 或 Inf
        if np.any(np.isnan(action)) or np.any(np.isinf(action)):
            print(f"[DanceAlgorithm] WARNING: Action contains NaN or Inf: {action}")
            print("[DanceAlgorithm] Using previous action buffer instead")
            # 使用上一次的 action_buffer 作为备用（线程安全）
            with self.action_buffer_lock:
                action = self.action_buffer.copy()
            if np.any(np.isnan(action)) or np.any(np.isinf(action)):
                print("[DanceAlgorithm] ERROR: Action buffer also contains NaN/Inf, cannot proceed")
                return
        
        # 计算目标关节位置
        action_scale = self.model_params.get('action_scale', np.ones(len(action), dtype=np.float32))
        if isinstance(action_scale, list):
            action_scale = np.array(action_scale, dtype=np.float32)
        
        # 确保 action_scale 长度匹配
        if len(action_scale) != len(action):
            print(f"[DanceAlgorithm] WARNING: action_scale length ({len(action_scale)}) != action length ({len(action)}), using first {len(action)} elements")
            action_scale = action_scale[:len(action)]
        
        default_pos_seq = self.joint_pos_array_seq if self.joint_pos_array_seq is not None else self.model_params.get('default_joint_pos', np.zeros(len(action), dtype=np.float32))
        if isinstance(default_pos_seq, list):
            default_pos_seq = np.array(default_pos_seq, dtype=np.float32)
        
        # 确保 default_pos_seq 长度匹配
        if len(default_pos_seq) < len(action):
            print(f"[DanceAlgorithm] WARNING: default_pos_seq length ({len(default_pos_seq)}) < action length ({len(action)}), padding with zeros")
            default_pos_seq = np.pad(default_pos_seq, (0, len(action) - len(default_pos_seq)), 'constant')
        
        target_dof_pos_seq = action * action_scale + default_pos_seq[:len(action)]
        
        # 检查计算结果是否包含 NaN
        if np.any(np.isnan(target_dof_pos_seq)) or np.any(np.isinf(target_dof_pos_seq)):
            print(f"[DanceAlgorithm] ERROR: target_dof_pos_seq contains NaN/Inf after calculation")
            print(f"  action: {action}")
            print(f"  action_scale: {action_scale}")
            print(f"  default_pos_seq[:len(action)]: {default_pos_seq[:len(action)]}")
            print(f"  target_dof_pos_seq: {target_dof_pos_seq}")
            return
        
        # 映射到XML关节顺序
        if self.joint_seq:
            try:
                target_dof_pos = np.array([
                    target_dof_pos_seq[self.joint_seq.index(joint)] 
                    if joint in self.joint_seq and self.joint_seq.index(joint) < len(target_dof_pos_seq) else 0.0
                    for joint in self.joint_xml_order
                ])
            except (ValueError, IndexError) as e:
                print(f"[DanceAlgorithm] ERROR: Failed to map joints: {e}")
                print(f"  joint_seq length: {len(self.joint_seq) if self.joint_seq else 0}")
                print(f"  target_dof_pos_seq length: {len(target_dof_pos_seq)}")
                print(f"  joint_xml_order length: {len(self.joint_xml_order)}")
                return
        else:
            target_dof_pos = target_dof_pos_seq
        
        # 检查映射后的结果是否包含 NaN
        if np.any(np.isnan(target_dof_pos)) or np.any(np.isinf(target_dof_pos)):
            print(f"[DanceAlgorithm] ERROR: target_dof_pos contains NaN/Inf after mapping")
            print(f"  target_dof_pos: {target_dof_pos}")
            return
        
        # 确保 target_dof_pos 长度正确
        if len(target_dof_pos) != 21:
            print(f"[DanceAlgorithm] ERROR: target_dof_pos length ({len(target_dof_pos)}) != 21")
            return
        
        # 组织关节命令
        joint_positions = {
            'L_Leg_q': target_dof_pos[0:6].tolist(),
            'R_Leg_q': target_dof_pos[6:12].tolist(),
            'waist_q': float(target_dof_pos[12]),
            'L_Arm_q': target_dof_pos[13:17].tolist(),
            'R_Arm_q': target_dof_pos[17:21].tolist()
        }
        
        # 获取Kp和Kd（模型顺序，确保是numpy数组）
        stiffness_seq = self.model_params.get('joint_stiffness', np.zeros(21, dtype=np.float32))
        damping_seq = self.model_params.get('joint_damping', np.zeros(21, dtype=np.float32))
        
        # 如果已经是numpy数组，直接使用；如果是列表，转换为numpy数组
        if isinstance(stiffness_seq, list):
            stiffness_seq = np.array(stiffness_seq, dtype=np.float32)
        if isinstance(damping_seq, list):
            damping_seq = np.array(damping_seq, dtype=np.float32)
        
        # 映射到XML关节顺序（与角度映射方式相同）
        if self.joint_seq:
            stiffness = np.array([
                stiffness_seq[self.joint_seq.index(joint)] 
                if joint in self.joint_seq else 0.0
                for joint in self.joint_xml_order
            ])
            damping = np.array([
                damping_seq[self.joint_seq.index(joint)] 
                if joint in self.joint_seq else 0.0
                for joint in self.joint_xml_order
            ])
        else:
            stiffness = stiffness_seq
            damping = damping_seq
        
        joint_kp = {
            'L_Leg_kp': stiffness[0:6].tolist(),
            'R_Leg_kp': stiffness[6:12].tolist(),
            'waist_kp': float(stiffness[12]),
            'L_Arm_kp': stiffness[13:17].tolist(),
            'R_Arm_kp': stiffness[17:21].tolist()
        }
        
        joint_kd = {
            'L_Leg_kd': damping[0:6].tolist(),
            'R_Leg_kd': damping[6:12].tolist(),
            'waist_kd': float(damping[12]),
            'L_Arm_kd': damping[13:17].tolist(),
            'R_Arm_kd': damping[17:21].tolist()
        }
        
        # 期望速度全部设置为0
        target_dof_vel = np.zeros(21, dtype=np.float32)
        
        joint_velocities = {
            'L_Leg_qd': target_dof_vel[0:6].tolist(),
            'R_Leg_qd': target_dof_vel[6:12].tolist(),
            'waist_qd': float(target_dof_vel[12]),
            'L_Arm_qd': target_dof_vel[13:17].tolist(),
            'R_Arm_qd': target_dof_vel[17:21].tolist()
        }
        
        # 更新命令缓存（不直接发送，由500Hz发送循环发送）
        with self.latest_command_lock:
            self.latest_command = {
                'enable_development_mode': True,
                'is_rl_mode': self.config['rl_mode']['is_rl_mode'],
                'joint_positions': joint_positions,
                'joint_velocities': joint_velocities,
                'joint_kp': joint_kp,
                'joint_kd': joint_kd
            }
        
        # 更新时间步
        self.timestep += 1
        
        # 检查是否执行完成（执行完最后一个timestep后）
        if self.timestep >= self.motion_input_pos.shape[0]:
            # 执行完成，标记完成并发送结束信号
            self.execution_complete = True
            print(f"[DanceAlgorithm] Execution complete "
                  f"({self.motion_input_pos.shape[0]} timesteps). "
                  f"Sending end signal...")
            # 更新命令缓存为结束信号（enable_development_mode=False）
            # 注意：不在这里sleep，避免阻塞50Hz的执行循环
            # 结束命令会由500Hz的LCM发送循环持续发送，直到程序退出
            with self.latest_command_lock:
                self.latest_command = {
                    'enable_development_mode': False,
                    'is_rl_mode': False
                }
            # 设置运行标志为False，让程序退出（线程安全）
            # 执行循环会在下一次迭代时检测到并退出
            with self.state_lock:
                self.running = False
    
    def on_development_mode_start(self):
        """开发模式开始时的回调"""
        self.timestep = self.config['dataset'].get('start_timestep', 50)
        self.execution_complete = False  # 重置完成标志
        
        # 记录robot的初始yaw（从当前状态获取）
        state = self.lcm_interface.get_latest_state()
        if state is not None:
            robot_quat = np.array([
                state.quat[0], state.quat[1], state.quat[2], state.quat[3]
            ])
            # 归一化
            robot_quat_norm = np.linalg.norm(robot_quat)
            if robot_quat_norm > 1e-6:
                robot_quat = robot_quat / robot_quat_norm
                _, _, self.robot_initial_yaw = self._quat_to_euler(robot_quat)
                print(f"[DanceAlgorithm] Robot initial yaw: {self.robot_initial_yaw:.4f} rad")
                
                # 计算yaw差值：motion_initial_yaw - robot_initial_yaw
                if self.motion_initial_yaw is not None:
                    yaw_diff = self.motion_initial_yaw - self.robot_initial_yaw
                    # 构造yaw对齐四元数
                    self.yaw_quat = self._quat_from_yaw(yaw_diff)
                    print(f"[DanceAlgorithm] Yaw alignment initialized: yaw_diff = {yaw_diff:.4f} rad")
                else:
                    print("[DanceAlgorithm] WARNING: motion_initial_yaw not available, using motion yaw only")
                    # 如果没有motion初始yaw，使用motion的yaw（向后兼容）
                    if self.motion_quat is not None and self.motion_quat.shape[0] > 0:
                        motion_initial_quat = self.motion_quat[0, 0, :]
                        _, _, motion_yaw = self._quat_to_euler(motion_initial_quat)
                        self.yaw_quat = self._quat_from_yaw(motion_yaw)
            else:
                print("[DanceAlgorithm] WARNING: Invalid robot quaternion, yaw_quat set to identity")
                self.robot_initial_yaw = None
                self.yaw_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        else:
            print("[DanceAlgorithm] WARNING: No state available, yaw_quat set to identity")
            self.robot_initial_yaw = None
            self.yaw_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        
        print(f"[DanceAlgorithm] Reset timestep to {self.timestep}")
    
    def on_development_mode_end(self):
        """开发模式结束时的回调"""
        print("[DanceAlgorithm] Development mode ended. Exiting program...")
        # 确保程序退出（线程安全）
        with self.state_lock:
            self.running = False


def main():
    """主函数"""
    import argparse
    import os
    
    parser = argparse.ArgumentParser(description='Dance Algorithm Runner')
    parser.add_argument('--config', type=str, default='config.yaml',
                       help='Path to configuration file (default: config.yaml)')
    args = parser.parse_args()
    
    # 处理配置文件路径（参考 mujoco_simulator.py）
    config_path = args.config
    if not os.path.isabs(config_path):
        # 如果是相对路径，基于脚本所在目录
        script_dir = os.path.dirname(os.path.abspath(__file__))
        potential_path = os.path.join(script_dir, config_path)
        if os.path.exists(potential_path):
            config_path = potential_path
        else:
            # 否则基于当前工作目录
            config_path = os.path.abspath(config_path)
    
    print(f"[DanceAlgorithm] Using config file: {config_path}")
    
    # 创建算法实例
    try:
        algorithm = DanceAlgorithm(config_path)
        algorithm.run()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()


