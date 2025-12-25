"""
MuJoCo 仿真器主程序
作者: Han Jiang (jh18954242606@163.com)
日期: 2025-12
功能: 运行 MuJoCo 仿真并通过 LCM 与控制器通信
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import sys
import os
import threading
from typing import Optional

# 添加项目根目录到路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(project_root, 'lcm-types', 'python'))

LCM_AVAILABLE = False
try:
    import lcm
    from humanoid_joint_state_t import humanoid_joint_state_t
    from humanoid_joint_command_t import humanoid_joint_command_t
    from microstrain_lcmt import microstrain_lcmt
    LCM_AVAILABLE = True
except ImportError as e:
    print(f"警告: LCM 模块导入失败: {e}")
    LCM_AVAILABLE = False

# LCM 通道名称（默认值，将从配置文件读取）
DEFAULT_JOINT_STATE_CHANNEL = "HUMANOID_JOINT_STATE_jh"
DEFAULT_JOINT_COMMAND_CHANNEL = "HUMANOID_JOINT_COMMAND_jh"
DEFAULT_IMU_DATA_CHANNEL = "MICROSTRAIN_IMU_DATA_jh"


class MuJoCoSimulator:
    """MuJoCo 仿真器类"""
    
    def __init__(self, xml_path: str, lcm_url: str = "", simulation_dt: float = 0.002,
                 joint_state_channel: str = DEFAULT_JOINT_STATE_CHANNEL,
                 joint_command_channel: str = DEFAULT_JOINT_COMMAND_CHANNEL,
                 imu_data_channel: str = DEFAULT_IMU_DATA_CHANNEL):
        """
        初始化 MuJoCo 仿真器
        
        Args:
            xml_path: MuJoCo XML 文件路径
            lcm_url: LCM URL（空字符串表示使用默认）
            simulation_dt: 仿真时间步长（秒）
            joint_state_channel: 关节状态 LCM 通道名
            joint_command_channel: 关节命令 LCM 通道名
            imu_data_channel: IMU 数据 LCM 通道名
        """
        self.joint_state_channel = joint_state_channel
        self.joint_command_channel = joint_command_channel
        self.imu_data_channel = imu_data_channel
        self.xml_path = xml_path
        self.simulation_dt = simulation_dt
        self.model = None
        self.data = None
        self.viewer = None
        
        # LCM 通信
        self.lcm = None
        self.lcm_thread = None
        self.running = False
        
        # 关节索引映射（根据 MuJoCo 模型定义）
        # 需要根据实际模型调整
        self.joint_indices = {
            'L_Leg': [0, 1, 2, 3, 4, 5],      # 左腿 6 个关节
            'R_Leg': [6, 7, 8, 9, 10, 11],    # 右腿 6 个关节
            'waist': [12],                     # 腰部 1 个关节
            'L_Arm': [13, 14, 15, 16],        # 左臂 4 个关节
            'R_Arm': [17, 18, 19, 20],        # 右臂 4 个关节
            'Head': [21]                       # 头部 1 个关节（如果存在）
        }
        
        # 当前接收到的控制命令
        self.command_lock = threading.Lock()
        self.current_command = None
        
        # 初始化 LCM
        if LCM_AVAILABLE:
            try:
                if lcm_url:
                    self.lcm = lcm.LCM(lcm_url)
                else:
                    self.lcm = lcm.LCM()
                print("[MuJoCoSimulator] LCM initialized successfully")
                print(f"[MuJoCoSimulator] LCM URL: "
                      f"{lcm_url if lcm_url else 'default'}")
                print("[MuJoCoSimulator] Publishing channels:")
                print(f"  - {self.joint_state_channel}")
                print(f"  - {self.imu_data_channel}")
                print("[MuJoCoSimulator] Subscribing channels:")
                print(f"  - {self.joint_command_channel}")
            except Exception as e:
                print(f"[MuJoCoSimulator] LCM initialization failed: {e}")
                import traceback
                traceback.print_exc()
                self.lcm = None
        
        # 加载 MuJoCo 模型
        self._load_model()
    
    def _load_model(self):
        """加载 MuJoCo 模型"""
        try:
            if not os.path.isabs(self.xml_path):
                # 如果是相对路径，相对于项目根目录
                self.xml_path = os.path.join(project_root, self.xml_path)
            
            if not os.path.exists(self.xml_path):
                raise FileNotFoundError(f"MuJoCo XML file not found: {self.xml_path}")
            
            self.model = mujoco.MjModel.from_xml_path(self.xml_path)
            self.data = mujoco.MjData(self.model)
            print(f"[MuJoCoSimulator] Model loaded from: {self.xml_path}")
            print(f"[MuJoCoSimulator] Model has {self.model.nq} position DOFs, {self.model.nv} velocity DOFs")
            print(f"[MuJoCoSimulator] Model has {self.model.nu} actuators")
            print(f"[MuJoCoSimulator] Model has {self.model.njnt} joints")
            
            # 打印所有关节名称和索引
            print("[MuJoCoSimulator] 关节索引映射:")
            for i in range(self.model.njnt):
                jnt_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
                if jnt_name:
                    print(f"  关节索引 {i}: {jnt_name}")
            
            # 打印执行器信息
            print("[MuJoCoSimulator] 执行器信息:")
            for i in range(self.model.nu):
                act_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                if act_name:
                    print(f"  执行器索引 {i}: {act_name}")
                else:
                    print(f"  执行器索引 {i}: (unnamed)")
        except Exception as e:
            print(f"[MuJoCoSimulator] Failed to load model: {e}")
            raise
    
    def _get_joint_indices_from_model(self):
        """从 MuJoCo 模型中获取关节索引"""
        # 这里需要根据实际的 MuJoCo 模型来映射关节名称
        # 假设关节名称遵循特定命名规则
        joint_map = {}
        
        # 遍历所有关节，根据名称分类
        for i in range(self.model.njnt):
            jnt_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if jnt_name:
                jnt_name_lower = jnt_name.lower()
                # 根据命名规则分类（需要根据实际模型调整）
                if 'left_leg' in jnt_name_lower or 'l_leg' in jnt_name_lower:
                    if 'L_Leg' not in joint_map:
                        joint_map['L_Leg'] = []
                    joint_map['L_Leg'].append(i)
                elif 'right_leg' in jnt_name_lower or 'r_leg' in jnt_name_lower:
                    if 'R_Leg' not in joint_map:
                        joint_map['R_Leg'] = []
                    joint_map['R_Leg'].append(i)
                elif 'waist' in jnt_name_lower:
                    joint_map['waist'] = [i]
                elif 'left_arm' in jnt_name_lower or 'l_arm' in jnt_name_lower:
                    if 'L_Arm' not in joint_map:
                        joint_map['L_Arm'] = []
                    joint_map['L_Arm'].append(i)
                elif 'right_arm' in jnt_name_lower or 'r_arm' in jnt_name_lower:
                    if 'R_Arm' not in joint_map:
                        joint_map['R_Arm'] = []
                    joint_map['R_Arm'].append(i)
                elif 'head' in jnt_name_lower:
                    joint_map['Head'] = [i]
        
        return joint_map
    
    def _handle_joint_command(self, channel, data):
        """处理接收到的关节控制命令"""
        try:
            msg = humanoid_joint_command_t.decode(data)
            with self.command_lock:
                self.current_command = msg
        except Exception as e:
            print(f"[MuJoCoSimulator] Failed to decode command: {e}")
            import traceback
            traceback.print_exc()
    
    def _publish_joint_state(self):
        """发布关节状态到 LCM"""
        if not LCM_AVAILABLE or not self.lcm:
            return
        
        try:
            msg = humanoid_joint_state_t()
            
            # 从 MuJoCo 数据中提取关节状态
            # 注意：qpos[0:3]是位置，qpos[3:7]是四元数，qpos[7:]是关节角度
            #      qvel[0:6]是根身体速度（线速度3个+角速度3个），qvel[6:]是关节速度
            #      ctrl 使用执行器索引
            ctrl_idx = 0
            
            # 左腿（6个关节）
            for i in range(6):
                if i < len(self.joint_indices['L_Leg']):
                    q_idx = self.joint_indices['L_Leg'][i]
                    # 关节角度从qpos[7]开始，所以需要加7
                    joint_q_idx = q_idx + 7
                    if joint_q_idx < self.model.nq:
                        msg.L_Leg_q[i] = float(self.data.qpos[joint_q_idx])
                    # 关节速度从qvel[6]开始，所以需要加6
                    joint_qd_idx = q_idx + 6
                    if joint_qd_idx < self.model.nv:
                        msg.L_Leg_qd[i] = float(self.data.qvel[joint_qd_idx])
                    if ctrl_idx < self.model.nu:
                        msg.L_Leg_tau[i] = float(self.data.ctrl[ctrl_idx])
                    ctrl_idx += 1
            
            # 右腿（6个关节）
            for i in range(6):
                if i < len(self.joint_indices['R_Leg']):
                    q_idx = self.joint_indices['R_Leg'][i]
                    joint_q_idx = q_idx + 7
                    if joint_q_idx < self.model.nq:
                        msg.R_Leg_q[i] = float(self.data.qpos[joint_q_idx])
                    joint_qd_idx = q_idx + 6
                    if joint_qd_idx < self.model.nv:
                        msg.R_Leg_qd[i] = float(self.data.qvel[joint_qd_idx])
                    if ctrl_idx < self.model.nu:
                        msg.R_Leg_tau[i] = float(self.data.ctrl[ctrl_idx])
                    ctrl_idx += 1
            
            # 腰部（1个关节）
            if 'waist' in self.joint_indices and len(self.joint_indices['waist']) > 0:
                q_idx = self.joint_indices['waist'][0]
                joint_q_idx = q_idx + 7
                if joint_q_idx < self.model.nq:
                    msg.waist_q = float(self.data.qpos[joint_q_idx])
                joint_qd_idx = q_idx + 6
                if joint_qd_idx < self.model.nv:
                    msg.waist_qd = float(self.data.qvel[joint_qd_idx])
                if ctrl_idx < self.model.nu:
                    msg.waist_tau = float(self.data.ctrl[ctrl_idx])
                ctrl_idx += 1
            
            # 左臂（4个关节）
            for i in range(4):
                if i < len(self.joint_indices['L_Arm']):
                    q_idx = self.joint_indices['L_Arm'][i]
                    joint_q_idx = q_idx + 7
                    if joint_q_idx < self.model.nq:
                        msg.L_Arm_q[i] = float(self.data.qpos[joint_q_idx])
                    joint_qd_idx = q_idx + 6
                    if joint_qd_idx < self.model.nv:
                        msg.L_Arm_qd[i] = float(self.data.qvel[joint_qd_idx])
                    if ctrl_idx < self.model.nu:
                        msg.L_Arm_tau[i] = float(self.data.ctrl[ctrl_idx])
                    ctrl_idx += 1
            
            # 右臂（4个关节）
            for i in range(4):
                if i < len(self.joint_indices['R_Arm']):
                    q_idx = self.joint_indices['R_Arm'][i]
                    joint_q_idx = q_idx + 7
                    if joint_q_idx < self.model.nq:
                        msg.R_Arm_q[i] = float(self.data.qpos[joint_q_idx])
                    joint_qd_idx = q_idx + 6
                    if joint_qd_idx < self.model.nv:
                        msg.R_Arm_qd[i] = float(self.data.qvel[joint_qd_idx])
                    if ctrl_idx < self.model.nu:
                        msg.R_Arm_tau[i] = float(self.data.ctrl[ctrl_idx])
                    ctrl_idx += 1
            
            # 头部（如果存在）
            if 'Head' in self.joint_indices and len(self.joint_indices['Head']) > 0:
                q_idx = self.joint_indices['Head'][0]
                joint_q_idx = q_idx + 7
                if joint_q_idx < self.model.nq:
                    msg.Head_q = float(self.data.qpos[joint_q_idx])
                joint_qd_idx = q_idx + 6
                if joint_qd_idx < self.model.nv:
                    msg.Head_qd = float(self.data.qvel[joint_qd_idx])
                if ctrl_idx < self.model.nu:
                    msg.Head_tau = float(self.data.ctrl[ctrl_idx])
            
            try:
                self.lcm.publish(self.joint_state_channel, msg.encode())
            except Exception as pub_error:
                print(f"[MuJoCoSimulator] Failed to publish joint state: {pub_error}")
        except Exception as e:
            print(f"[MuJoCoSimulator] Failed to create/publish joint state: {e}")
            import traceback
            traceback.print_exc()
    
    def _publish_imu_data(self):
        """发布 IMU 数据到 LCM"""
        if not LCM_AVAILABLE or not self.lcm:
            return
        
        try:
            msg = microstrain_lcmt()
            
            # 参考 deploy_mujoco_plot_gamepad.py 的实现
            # MuJoCo 中根身体的姿态和速度存储在 qpos 和 qvel 的前几个元素
            # qpos[0:3] 是位置 (x, y, z)
            # qpos[3:7] 是四元数 (w, x, y, z)
            # qvel[0:3] 是线速度 (vx, vy, vz)
            # qvel[3:6] 是角速度 (wx, wy, wz)
            
            # 获取四元数（从 qpos[3:7]）
            if self.model.nq >= 7:
                quat = self.data.qpos[3:7]
                msg.quat[3] = float(quat[0])  # w
                msg.quat[0] = float(quat[1])  # x
                msg.quat[1] = float(quat[2])  # y
                msg.quat[2] = float(quat[3])  # z
                
                # 计算 RPY（从四元数）
                rpy = self._quat_to_rpy(quat)
                msg.rpy[0] = float(rpy[0])  # roll
                msg.rpy[1] = float(rpy[1])  # pitch
                msg.rpy[2] = float(rpy[2])  # yaw
            else:
                # 如果模型没有足够的自由度，使用默认值
                msg.quat[0] = 1.0
                msg.quat[1] = 0.0
                msg.quat[2] = 0.0
                msg.quat[3] = 0.0
                msg.rpy[0] = 0.0
                msg.rpy[1] = 0.0
                msg.rpy[2] = 0.0
            
            # 获取角速度（从 qvel[3:6]）
            if self.model.nv >= 6:
                omega = self.data.qvel[3:6]
                msg.omega[0] = float(omega[0])  # wx
                msg.omega[1] = float(omega[1])  # wy
                msg.omega[2] = float(omega[2])  # wz
            else:
                msg.omega[0] = 0.0
                msg.omega[1] = 0.0
                msg.omega[2] = 0.0
            
            # 获取加速度（从 qacc 中提取，简化处理）
            # 注意：qacc 是关节加速度，对于根身体可能需要考虑重力
            # 这里简化处理，使用前3个 qacc 作为线加速度
            if self.model.nv >= 3:
                acc = self.data.qacc[:3]
                msg.acc[0] = float(acc[0])
                msg.acc[1] = float(acc[1])
                msg.acc[2] = float(acc[2])
            else:
                msg.acc[0] = 0.0
                msg.acc[1] = 0.0
                msg.acc[2] = 0.0
            
            try:
                self.lcm.publish(self.imu_data_channel, msg.encode())
            except Exception as pub_error:
                print(f"[MuJoCoSimulator] Failed to publish IMU data: {pub_error}")
        except Exception as e:
            print(f"[MuJoCoSimulator] Failed to create/publish IMU data: {e}")
            import traceback
            traceback.print_exc()
    
    def _quat_to_rpy(self, quat):
        """四元数转 RPY"""
        w, x, y, z = quat[0], quat[1], quat[2], quat[3]
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return np.array([roll, pitch, yaw])
    
    def _apply_control(self):
        """应用控制命令到 MuJoCo"""
        with self.command_lock:
            if self.current_command is None:
                return
        
        cmd = self.current_command
        
        # 应用 PD 控制: tau = kp*(qdes - q) + kd*(qddes - qd) + tff
        ctrl_idx = 0
        
        # 调试信息已关闭
        
        # 左腿控制（前4个关节的kp和kd乘以10，最后2个关节不乘）
        for i in range(6):
            if ctrl_idx < self.model.nu:
                q_des = cmd.L_Leg_q[i]
                qd_des = cmd.L_Leg_qd[i]
                # 前4个关节（索引0-3）乘以10，最后2个关节（索引4-5，踝关节）不乘
                if i < 4:
                    kp = cmd.L_Leg_kp[i] * 10.0  # kp 乘以 10
                    kd = cmd.L_Leg_kd[i] * 10.0  # kd 乘以 10
                else:
                    kp = cmd.L_Leg_kp[i]  # 最后2个关节不乘以10
                    kd = cmd.L_Leg_kd[i]
                tau_ff = cmd.L_Leg_tau[i]
                
                # 检查输入值的有效性
                if not np.isfinite(q_des):
                    q_des = 0.0
                if not np.isfinite(qd_des):
                    qd_des = 0.0
                if not np.isfinite(kp):
                    kp = 0.0
                if not np.isfinite(kd):
                    kd = 0.0
                if not np.isfinite(tau_ff):
                    tau_ff = 0.0
                
                if i < len(self.joint_indices['L_Leg']):
                    q_idx = self.joint_indices['L_Leg'][i]
                    # 关节角度从qpos[7]开始
                    joint_q_idx = q_idx + 7
                    if joint_q_idx < self.model.nq:
                        q = self.data.qpos[joint_q_idx]
                    else:
                        q = 0.0
                    # 关节速度从qvel[6]开始
                    joint_qd_idx = q_idx + 6
                    if joint_qd_idx < self.model.nv:
                        qd = self.data.qvel[joint_qd_idx]
                    else:
                        qd = 0.0
                    
                    tau = kp * (q_des - q) + kd * (qd_des - qd) + tau_ff
                    
                    # 检查并处理 NaN/Inf
                    if not np.isfinite(tau) or abs(tau) > 1000.0:
                        tau = 0.0
                        if not np.isfinite(kp) or not np.isfinite(kd):
                            print(f"[MuJoCoSimulator] Warning: Invalid kp={kp} or kd={kd} at actuator {ctrl_idx}")
                    
                    self.data.ctrl[ctrl_idx] = tau
                ctrl_idx += 1
        
        # 右腿控制（前4个关节的kp和kd乘以10，最后2个关节不乘）
        for i in range(6):
            if ctrl_idx < self.model.nu:
                q_des = cmd.R_Leg_q[i]
                qd_des = cmd.R_Leg_qd[i]
                # 前4个关节（索引0-3）乘以10，最后2个关节（索引4-5，踝关节）不乘
                if i < 4:
                    kp = cmd.R_Leg_kp[i] * 10.0  # kp 乘以 10
                    kd = cmd.R_Leg_kd[i] * 10.0  # kd 乘以 10
                else:
                    kp = cmd.R_Leg_kp[i]  # 最后2个关节不乘以10
                    kd = cmd.R_Leg_kd[i]
                tau_ff = cmd.R_Leg_tau[i]
                
                # 检查输入值的有效性
                if not np.isfinite(q_des):
                    q_des = 0.0
                if not np.isfinite(qd_des):
                    qd_des = 0.0
                if not np.isfinite(kp):
                    kp = 0.0
                if not np.isfinite(kd):
                    kd = 0.0
                if not np.isfinite(tau_ff):
                    tau_ff = 0.0
                
                if i < len(self.joint_indices['R_Leg']):
                    q_idx = self.joint_indices['R_Leg'][i]
                    joint_q_idx = q_idx + 7
                    if joint_q_idx < self.model.nq:
                        q = self.data.qpos[joint_q_idx]
                    else:
                        q = 0.0
                    joint_qd_idx = q_idx + 6
                    if joint_qd_idx < self.model.nv:
                        qd = self.data.qvel[joint_qd_idx]
                    else:
                        qd = 0.0
                    
                    tau = kp * (q_des - q) + kd * (qd_des - qd) + tau_ff
                    
                    # 检查并处理 NaN/Inf
                    if not np.isfinite(tau) or abs(tau) > 1000.0:
                        tau = 0.0
                        if not np.isfinite(kp) or not np.isfinite(kd):
                            print(f"[MuJoCoSimulator] Warning: Invalid kp={kp} or kd={kd} at actuator {ctrl_idx}")
                    
                    self.data.ctrl[ctrl_idx] = tau
                ctrl_idx += 1
        
        # 腰部控制（kp 和 kd 乘以 10）
        if 'waist' in self.joint_indices and len(self.joint_indices['waist']) > 0 and ctrl_idx < self.model.nu:
            q_des = cmd.waist_q
            qd_des = cmd.waist_qd
            kp = cmd.waist_kp * 10.0  # kp 乘以 10
            kd = cmd.waist_kd * 10.0  # kd 乘以 10
            tau_ff = cmd.waist_tau
            
            # 检查输入值的有效性
            if not np.isfinite(q_des):
                q_des = 0.0
            if not np.isfinite(qd_des):
                qd_des = 0.0
            if not np.isfinite(kp):
                kp = 0.0
            if not np.isfinite(kd):
                kd = 0.0
            if not np.isfinite(tau_ff):
                tau_ff = 0.0
            
            q_idx = self.joint_indices['waist'][0]
            joint_q_idx = q_idx + 7
            if joint_q_idx < self.model.nq:
                q = self.data.qpos[joint_q_idx]
            else:
                q = 0.0
            joint_qd_idx = q_idx + 6
            if joint_qd_idx < self.model.nv:
                qd = self.data.qvel[joint_qd_idx]
            else:
                qd = 0.0
            
            tau = kp * (q_des - q) + kd * (qd_des - qd) + tau_ff
            self.data.ctrl[ctrl_idx] = tau
            ctrl_idx += 1
        
        # 左臂控制
        for i in range(4):
            if ctrl_idx < self.model.nu:
                q_des = cmd.L_Arm_q[i]
                qd_des = cmd.L_Arm_qd[i]
                kp = cmd.L_Arm_kp[i]
                kd = cmd.L_Arm_kd[i]
                tau_ff = cmd.L_Arm_tau[i]
                
                # 检查输入值的有效性
                if not np.isfinite(q_des):
                    q_des = 0.0
                if not np.isfinite(qd_des):
                    qd_des = 0.0
                if not np.isfinite(kp):
                    kp = 0.0
                if not np.isfinite(kd):
                    kd = 0.0
                if not np.isfinite(tau_ff):
                    tau_ff = 0.0
                
                if i < len(self.joint_indices['L_Arm']):
                    q_idx = self.joint_indices['L_Arm'][i]
                    joint_q_idx = q_idx + 7
                    if joint_q_idx < self.model.nq:
                        q = self.data.qpos[joint_q_idx]
                    else:
                        q = 0.0
                    joint_qd_idx = q_idx + 6
                    if joint_qd_idx < self.model.nv:
                        qd = self.data.qvel[joint_qd_idx]
                    else:
                        qd = 0.0
                    
                    tau = kp * (q_des - q) + kd * (qd_des - qd) + tau_ff
                    
                    # 检查并处理 NaN/Inf
                    if not np.isfinite(tau) or abs(tau) > 1000.0:
                        tau = 0.0
                        if not np.isfinite(kp) or not np.isfinite(kd):
                            print(f"[MuJoCoSimulator] Warning: Invalid kp={kp} or kd={kd} at actuator {ctrl_idx}")
                    
                    self.data.ctrl[ctrl_idx] = tau
                ctrl_idx += 1
        
        # 右臂控制
        for i in range(4):
            if ctrl_idx < self.model.nu:
                q_des = cmd.R_Arm_q[i]
                qd_des = cmd.R_Arm_qd[i]
                kp = cmd.R_Arm_kp[i]
                kd = cmd.R_Arm_kd[i]
                tau_ff = cmd.R_Arm_tau[i]
                
                # 检查输入值的有效性
                if not np.isfinite(q_des):
                    q_des = 0.0
                if not np.isfinite(qd_des):
                    qd_des = 0.0
                if not np.isfinite(kp):
                    kp = 0.0
                if not np.isfinite(kd):
                    kd = 0.0
                if not np.isfinite(tau_ff):
                    tau_ff = 0.0
                
                if i < len(self.joint_indices['R_Arm']):
                    q_idx = self.joint_indices['R_Arm'][i]
                    joint_q_idx = q_idx + 7
                    if joint_q_idx < self.model.nq:
                        q = self.data.qpos[joint_q_idx]
                    else:
                        q = 0.0
                    joint_qd_idx = q_idx + 6
                    if joint_qd_idx < self.model.nv:
                        qd = self.data.qvel[joint_qd_idx]
                    else:
                        qd = 0.0
                    
                    tau = kp * (q_des - q) + kd * (qd_des - qd) + tau_ff
                    
                    # 检查并处理 NaN/Inf
                    if not np.isfinite(tau) or abs(tau) > 1000.0:
                        tau = 0.0
                        if not np.isfinite(kp) or not np.isfinite(kd):
                            print(f"[MuJoCoSimulator] Warning: Invalid kp={kp} or kd={kd} at actuator {ctrl_idx}")
                    
                    self.data.ctrl[ctrl_idx] = tau
                ctrl_idx += 1
    
    def _lcm_thread_func(self):
        """LCM 消息处理线程"""
        while self.running:
            if self.lcm:
                try:
                    # Python LCM 库使用 handle_timeout (小写，带下划线)
                    # 参数是超时时间（毫秒）
                    timeout_ms = 10
                    self.lcm.handle_timeout(timeout_ms)
                except AttributeError:
                    # 如果 handle_timeout 不存在，尝试 handle
                    try:
                        self.lcm.handle()
                    except Exception as e:
                        # 忽略超时异常（这是正常的）
                        if "timeout" not in str(e).lower():
                            print(f"[MuJoCoSimulator] LCM handle error: {e}")
                        time.sleep(0.01)
                except Exception as e:
                    # 忽略超时异常（这是正常的）
                    if "timeout" not in str(e).lower():
                        print(f"[MuJoCoSimulator] LCM handle error: {e}")
                    time.sleep(0.01)
            else:
                time.sleep(0.01)
    
    def run(self, headless: bool = False):
        """
        运行仿真
        
        Args:
            headless: 是否无头模式（不显示可视化窗口）
        """
        if not self.model or not self.data:
            raise RuntimeError("Model not loaded")
        
        self.running = True
        
        # 启动 LCM 订阅
        if LCM_AVAILABLE and self.lcm:
            try:
                self.lcm.subscribe(self.joint_command_channel, self._handle_joint_command)
                self.lcm_thread = threading.Thread(target=self._lcm_thread_func, daemon=True)
                self.lcm_thread.start()
                print("[MuJoCoSimulator] LCM subscription started successfully")
                print(f"[MuJoCoSimulator] Listening on channel: {self.joint_command_channel}")
            except Exception as e:
                print(f"[MuJoCoSimulator] Failed to start LCM subscription: {e}")
                import traceback
                traceback.print_exc()
        else:
            if not LCM_AVAILABLE:
                print("[MuJoCoSimulator] WARNING: LCM not available, skipping subscription")
            if not self.lcm:
                print("[MuJoCoSimulator] WARNING: LCM not initialized, skipping subscription")
        
        # 启动仿真循环
        if headless:
            self._run_headless()
        else:
            self._run_with_viewer()
    
    def _run_headless(self):
        """无头模式运行"""
        print("[MuJoCoSimulator] Running in headless mode")
        try:
            while self.running:
                # 应用控制
                self._apply_control()
                
                # 执行一步仿真
                mujoco.mj_step(self.model, self.data)
                
                # 发布状态（只在 LCM 可用时发布）
                if LCM_AVAILABLE and self.lcm:
                    self._publish_joint_state()
                    self._publish_imu_data()
                
                # 等待一个时间步长
                time.sleep(self.simulation_dt)
        except KeyboardInterrupt:
            print("\n[MuJoCoSimulator] Simulation stopped by user")
        finally:
            self.running = False
    
    def _run_with_viewer(self):
        """带可视化窗口运行"""
        print("[MuJoCoSimulator] Running with viewer")
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # 设置相机跟随根身体（默认视角跟随）
            viewer.cam.lookat[:] = [0, 0, 0.5]  # 看向原点
            viewer.cam.distance = 3.0  # 相机距离
            viewer.cam.azimuth = 90.0  # 方位角
            viewer.cam.elevation = -20.0  # 仰角
            viewer.cam.trackbodyid = -1  # -1 表示跟踪根身体（跟随模式）
            
            while viewer.is_running() and self.running:
                step_start = time.time()
                
                # 应用控制
                self._apply_control()
                
                # 执行一步仿真
                mujoco.mj_step(self.model, self.data)
                
                # 发布状态（只在 LCM 可用时发布）
                if LCM_AVAILABLE and self.lcm:
                    self._publish_joint_state()
                    self._publish_imu_data()
                
                # 同步可视化
                viewer.sync()
                
                # 时间同步
                time_until_next_step = self.simulation_dt - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
        
        self.running = False
    
    def stop(self):
        """停止仿真"""
        self.running = False
        if self.lcm_thread and self.lcm_thread.is_alive():
            self.lcm_thread.join(timeout=1.0)
        print("[MuJoCoSimulator] Simulation stopped")


def main():
    """主函数"""
    import argparse
    import yaml
    
    parser = argparse.ArgumentParser(description="MuJoCo 仿真器")
    parser.add_argument("--config", type=str, default="config.yaml",
                       help="配置文件路径（默认: config.yaml）")
    parser.add_argument("--headless", action="store_true",
                       help="无头模式（不显示可视化窗口）")
    args = parser.parse_args()
    
    # 加载配置
    config_path = args.config
    if not os.path.isabs(config_path):
        config_path = os.path.join(project_root, config_path)
    
    if not os.path.exists(config_path):
        print(f"错误: 配置文件不存在: {config_path}")
        return 1
    
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    # 检查是否启用 MuJoCo
    if not config.get('simulation', {}).get('enable_mujoco', False):
        print("MuJoCo 仿真未启用，请在配置文件中设置 simulation.enable_mujoco: true")
        return 0
    
    # 获取 MuJoCo 配置
    mujoco_config = config.get('simulation', {}).get('mujoco', {})
    xml_path = mujoco_config.get('xml_path', '')
    simulation_dt = mujoco_config.get('simulation_dt', 0.002)
    lcm_url = mujoco_config.get('lcm_url', '')
    
    # 读取 LCM 通道名
    lcm_channels = config.get('motor_communication', {}).get('lcm_channels', {})
    joint_state_channel = lcm_channels.get('joint_state', DEFAULT_JOINT_STATE_CHANNEL)
    joint_command_channel = lcm_channels.get('joint_command', DEFAULT_JOINT_COMMAND_CHANNEL)
    imu_data_channel = lcm_channels.get('imu_data', DEFAULT_IMU_DATA_CHANNEL)
    
    if not xml_path:
        print("错误: 配置文件中未指定 MuJoCo XML 文件路径")
        return 1
    
    # 创建并运行仿真器
    try:
        simulator = MuJoCoSimulator(xml_path, lcm_url, simulation_dt,
                                    joint_state_channel, joint_command_channel, imu_data_channel)
        simulator.run(headless=args.headless)
    except KeyboardInterrupt:
        print("\n[MuJoCoSimulator] Interrupted by user")
    except Exception as e:
        print(f"[MuJoCoSimulator] Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

