#!/usr/bin/env python3
"""
MuJoCo 仿真器 - 用于跳舞算法验证
与 run_algorithm.py 通过 LCM 通信
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import sys
import os
import threading

# 添加项目根目录到路径
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(project_root, 'lcm-types', 'python'))

LCM_AVAILABLE = False
try:
    import lcm
    from development_state_t import development_state_t
    from development_command_t import development_command_t
    LCM_AVAILABLE = True
except ImportError as e:
    print(f"警告: LCM 模块导入失败: {e}")
    LCM_AVAILABLE = False

# LCM 通道名称（与 run_algorithm.py 保持一致）
STATE_CHANNEL = "E3_development_state"
COMMAND_CHANNEL = "E3_development_command"


class MuJoCoDanceSimulator:
    """MuJoCo 跳舞算法仿真器类"""
    
    def __init__(self, xml_path: str, lcm_url: str = "", simulation_dt: float = 0.002, robot_id: str = "E3"):
        """
        初始化 MuJoCo 仿真器
        
        Args:
            xml_path: MuJoCo XML 文件路径
            lcm_url: LCM URL（空字符串表示使用默认）
            simulation_dt: 仿真时间步长（秒）
            robot_id: 机器人ID（用于LCM消息）
        """
        self.xml_path = xml_path
        self.simulation_dt = simulation_dt
        self.robot_id = robot_id
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
        self.development_mode_active = False
        
        # 初始化 LCM
        if LCM_AVAILABLE:
            try:
                if lcm_url:
                    self.lcm = lcm.LCM(lcm_url)
                else:
                    self.lcm = lcm.LCM()
                print("[MuJoCoDanceSimulator] LCM initialized successfully")
                print(f"[MuJoCoDanceSimulator] LCM URL: {lcm_url if lcm_url else 'default'}")
                print("[MuJoCoDanceSimulator] Publishing channels:")
                print(f"  - {STATE_CHANNEL}")
                print("[MuJoCoDanceSimulator] Subscribing channels:")
                print(f"  - {COMMAND_CHANNEL}")
            except Exception as e:
                print(f"[MuJoCoDanceSimulator] LCM initialization failed: {e}")
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
            print(f"[MuJoCoDanceSimulator] Model loaded from: {self.xml_path}")
            print(f"[MuJoCoDanceSimulator] Model has {self.model.nq} position DOFs, {self.model.nv} velocity DOFs")
            print(f"[MuJoCoDanceSimulator] Model has {self.model.nu} actuators")
            print(f"[MuJoCoDanceSimulator] Model has {self.model.njnt} joints")
            
            # 打印所有关节名称和索引
            print("[MuJoCoDanceSimulator] 关节索引映射:")
            for i in range(self.model.njnt):
                jnt_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
                if jnt_name:
                    print(f"  关节索引 {i}: {jnt_name}")
            
            # 打印执行器信息
            print("[MuJoCoDanceSimulator] 执行器信息:")
            for i in range(self.model.nu):
                act_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                if act_name:
                    print(f"  执行器索引 {i}: {act_name}")
                else:
                    print(f"  执行器索引 {i}: (unnamed)")
        except Exception as e:
            print(f"[MuJoCoDanceSimulator] Failed to load model: {e}")
            raise
    
    def _handle_development_command(self, channel, data):
        """处理接收到的开发模式控制命令"""
        try:
            msg = development_command_t.decode(data)
            
            # 检查机器人ID是否匹配
            if msg.robot_id != self.robot_id:
                return
            
            with self.command_lock:
                self.current_command = msg
                self.development_mode_active = (msg.enable_development_mode == 1)
        except Exception as e:
            print(f"[MuJoCoDanceSimulator] Failed to decode command: {e}")
            import traceback
            traceback.print_exc()
    
    def _publish_development_state(self):
        """发布开发模式状态到 LCM"""
        if not LCM_AVAILABLE or not self.lcm:
            return
        
        try:
            msg = development_state_t()
            msg.robot_id = self.robot_id
            
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
            
            # IMU数据（从根身体获取）
            # qpos[3:7] 是四元数 (w, x, y, z)
            if self.model.nq >= 7:
                quat = self.data.qpos[3:7]
                # LCM消息格式: quat[0]=x, quat[1]=y, quat[2]=z, quat[3]=w
                msg.quat[0] = float(quat[0])  # x
                msg.quat[1] = float(quat[1])  # y
                msg.quat[2] = float(quat[2])  # z
                msg.quat[3] = float(quat[3])  # w
                
                # 计算 RPY（从四元数）
                rpy = self._quat_to_rpy(quat)
                msg.rpy[0] = float(rpy[0])  # roll
                msg.rpy[1] = float(rpy[1])  # pitch
                msg.rpy[2] = float(rpy[2])  # yaw
            
            # qvel[3:6] 是角速度 (wx, wy, wz)
            if self.model.nv >= 6:
                omega = self.data.qvel[3:6]
                msg.omega[0] = float(omega[0])  # wx
                msg.omega[1] = float(omega[1])  # wy
                msg.omega[2] = float(omega[2])  # wz
            
            # qacc[3:6] 是角加速度，简化处理使用线加速度
            if self.model.nv >= 3:
                acc = self.data.qacc[:3]
                msg.acc[0] = float(acc[0])
                msg.acc[1] = float(acc[1])
                msg.acc[2] = float(acc[2])
            
            # 遥控器指令（默认值）
            msg.v_des[0] = 0.0
            msg.v_des[1] = 0.0
            msg.v_des[2] = 0.0
            msg.omega_des[0] = 0.0
            msg.omega_des[1] = 0.0
            msg.omega_des[2] = 0.0
            msg.mode = 0
            
            try:
                self.lcm.publish(STATE_CHANNEL, msg.encode())
            except Exception as pub_error:
                print(f"[MuJoCoDanceSimulator] Failed to publish state: {pub_error}")
        except Exception as e:
            print(f"[MuJoCoDanceSimulator] Failed to create/publish state: {e}")
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
            if self.current_command is None or not self.development_mode_active:
                return
        
        cmd = self.current_command
        
        # 应用 PD 控制: tau = kp*(qdes - q) + kd*(qddes - qd) + tff
        ctrl_idx = 0
        
        # 左腿控制
        for i in range(6):
            if ctrl_idx < self.model.nu:
                q_des = cmd.L_Leg_q[i]
                qd_des = cmd.L_Leg_qd[i]
                kp = cmd.L_Leg_kp[i]
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
                    
                    self.data.ctrl[ctrl_idx] = tau
                ctrl_idx += 1
        
        # 右腿控制
        for i in range(6):
            if ctrl_idx < self.model.nu:
                q_des = cmd.R_Leg_q[i]
                qd_des = cmd.R_Leg_qd[i]
                kp = cmd.R_Leg_kp[i]
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
                    
                    self.data.ctrl[ctrl_idx] = tau
                ctrl_idx += 1
        
        # 腰部控制
        if 'waist' in self.joint_indices and len(self.joint_indices['waist']) > 0 and ctrl_idx < self.model.nu:
            q_des = cmd.waist_q
            qd_des = cmd.waist_qd
            kp = cmd.waist_kp
            kd = cmd.waist_kd
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
            if not np.isfinite(tau) or abs(tau) > 1000.0:
                tau = 0.0
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
                    
                    self.data.ctrl[ctrl_idx] = tau
                ctrl_idx += 1
    
    def _lcm_thread_func(self):
        """LCM 消息处理线程"""
        while self.running:
            if self.lcm:
                try:
                    timeout_ms = 10
                    self.lcm.handle_timeout(timeout_ms)
                except AttributeError:
                    try:
                        self.lcm.handle()
                    except Exception as e:
                        if "timeout" not in str(e).lower():
                            print(f"[MuJoCoDanceSimulator] LCM handle error: {e}")
                        time.sleep(0.01)
                except Exception as e:
                    if "timeout" not in str(e).lower():
                        print(f"[MuJoCoDanceSimulator] LCM handle error: {e}")
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
                self.lcm.subscribe(COMMAND_CHANNEL, self._handle_development_command)
                self.lcm_thread = threading.Thread(target=self._lcm_thread_func, daemon=True)
                self.lcm_thread.start()
                print("[MuJoCoDanceSimulator] LCM subscription started successfully")
                print(f"[MuJoCoDanceSimulator] Listening on channel: {COMMAND_CHANNEL}")
            except Exception as e:
                print(f"[MuJoCoDanceSimulator] Failed to start LCM subscription: {e}")
                import traceback
                traceback.print_exc()
        else:
            if not LCM_AVAILABLE:
                print("[MuJoCoDanceSimulator] WARNING: LCM not available, skipping subscription")
            if not self.lcm:
                print("[MuJoCoDanceSimulator] WARNING: LCM not initialized, skipping subscription")
        
        # 启动仿真循环
        if headless:
            self._run_headless()
        else:
            self._run_with_viewer()
    
    def _run_headless(self):
        """无头模式运行"""
        print("[MuJoCoDanceSimulator] Running in headless mode")
        try:
            while self.running:
                # 应用控制
                self._apply_control()
                
                # 执行一步仿真
                mujoco.mj_step(self.model, self.data)
                
                # 发布状态（只在 LCM 可用时发布）
                if LCM_AVAILABLE and self.lcm:
                    self._publish_development_state()
                
                # 等待一个时间步长
                time.sleep(self.simulation_dt)
        except KeyboardInterrupt:
            print("\n[MuJoCoDanceSimulator] Simulation stopped by user")
        finally:
            self.running = False
    
    def _run_with_viewer(self):
        """带可视化窗口运行"""
        print("[MuJoCoDanceSimulator] Running with viewer")
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # 设置相机跟随根身体
            viewer.cam.lookat[:] = [0, 0, 0.5]  # 看向原点
            viewer.cam.distance = 3.0  # 相机距离
            viewer.cam.azimuth = 90.0  # 方位角
            viewer.cam.elevation = -20.0  # 仰角
            viewer.cam.trackbodyid = -1  # -1 表示跟踪根身体
            
            while viewer.is_running() and self.running:
                step_start = time.time()
                
                # 应用控制
                self._apply_control()
                
                # 执行一步仿真
                mujoco.mj_step(self.model, self.data)
                
                # 发布状态（只在 LCM 可用时发布）
                if LCM_AVAILABLE and self.lcm:
                    self._publish_development_state()
                
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
        print("[MuJoCoDanceSimulator] Simulation stopped")


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="MuJoCo 跳舞算法仿真器")
    parser.add_argument(
        "--xml", type=str, default="resources/robots/e3/scene.xml",
        help="MuJoCo XML 文件路径（默认: resources/robots/e3/scene.xml）")
    parser.add_argument(
        "--lcm-url", type=str, default="",
        help="LCM URL（默认: 空字符串，使用默认）")
    parser.add_argument(
        "--dt", type=float, default=0.002,
        help="仿真时间步长（秒，默认: 0.002）")
    parser.add_argument(
        "--robot-id", type=str, default="E3",
        help="机器人ID（默认: E3）")
    parser.add_argument(
        "--headless", action="store_true",
        help="无头模式（不显示可视化窗口）")
    args = parser.parse_args()
    
    # 创建并运行仿真器
    try:
        simulator = MuJoCoDanceSimulator(
            xml_path=args.xml,
            lcm_url=args.lcm_url,
            simulation_dt=args.dt,
            robot_id=args.robot_id
        )
        simulator.run(headless=args.headless)
    except KeyboardInterrupt:
        print("\n[MuJoCoDanceSimulator] Interrupted by user")
    except Exception as e:
        print(f"[MuJoCoDanceSimulator] Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

