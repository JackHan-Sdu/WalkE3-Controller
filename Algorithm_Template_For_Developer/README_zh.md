# Yobotics HumanoidE3 算法部署模板

<div align="right">
  <a href="README.md">English</a> | <a href="README_zh.md">中文</a>
</div>

## 视频教程

<video width="800" controls>
  <source src="How to develop E3?.mp4" type="video/mp4">
  您的浏览器不支持视频播放。请下载视频文件查看：<a href="How to develop E3?.mp4">How to develop E3?.mp4</a>
</video>

本仓库提供了用于 Yobotics HumanoidE3 机器人外部算法部署的开发模板框架。开发者可以基于此框架快速实现自定义控制算法，并通过 LCM 与机器人状态机进行通信。

## 目录结构

```
Algorithm_Template_For_Developer/
├── README.md                    # 本文档（英文）
├── README_zh.md                 # 中文文档
├── __init__.py                  # 包初始化文件
├── algorithm_base.py            # 算法基类（AlgorithmBase）
├── lcm_interface.py             # LCM通信接口（LCMInterface）
└── dance_algorithm/             # 跳舞算法示例
    ├── config.yaml              # 算法配置文件
    ├── run_algorithm.py         # 算法执行脚本（DanceAlgorithm实现）
    ├── mujoco_simulator.py      # MuJoCo仿真器（用于算法验证）
    ├── HumanActornet.onnx       # ONNX策略模型
    └── dataset/                 # 数据集目录
        └── E3Getup350.npz       # 动作序列数据集
```

## 核心组件

### 1. AlgorithmBase（算法基类）

所有自定义算法都应继承 `AlgorithmBase` 类。该类提供了以下功能：

- **策略模型加载**：支持 ONNX 和 PyTorch（.pt）模型，自动从 ONNX metadata 读取配置
- **LCM通信管理**：自动处理 LCM 订阅和发布
- **双线程执行架构**：
  - 策略推理线程：按配置频率运行（如 50Hz）
  - LCM发送线程：固定 500Hz 高频发送控制命令
- **开发模式控制**：自动处理开发模式的开始和结束
- **模型预热**：使用真实状态进行预热推理，检测 action 异常
- **线程安全**：所有状态访问都经过锁保护

#### 必须实现的方法

- `compute_observation(state)`: 根据状态计算观测向量
- `process_action(state, action)`: 处理动作并发送控制命令（通过更新 `latest_command`）

#### 可选重写的方法

- `on_development_mode_start()`: 开发模式开始时的回调
- `on_development_mode_end()`: 开发模式结束时的回调

### 2. LCMInterface（LCM通信接口）

封装了所有 LCM 相关的通信功能：

- 订阅机器人状态消息（`development_state_t`）
- 发布控制命令消息（`development_command_t`）
- 状态缓存和线程安全的访问接口
- 支持状态消息回调

### 3. DanceAlgorithm（示例算法）

基于 `deploy_mujoco_1Step.py` 实现的跳舞算法示例，展示了：

- 从数据集加载动作序列
- 使用 ONNX 模型进行动作生成
- 完整的观测计算（包括 anchor orientation、yaw对齐等）
- 关节顺序映射（模型顺序 ↔ XML顺序）

## 快速开始

### 1. 创建新算法文件夹

```bash
cd Algorithm_Template_For_Developer
mkdir your_algorithm_name
cd your_algorithm_name
```

### 2. 创建配置文件和算法脚本

复制示例算法的配置文件模板：

```bash
cp ../dance_algorithm/config.yaml .
cp ../dance_algorithm/run_algorithm.py ./your_algorithm_name.py
```

### 3. 实现你的算法类

编辑 `your_algorithm_name.py`，继承 `AlgorithmBase` 并实现必要的方法：

```python
#!/usr/bin/env python3
import sys
import os
import numpy as np

# 添加父目录到路径
_parent_dir = os.path.join(os.path.dirname(__file__), '..')
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)

from algorithm_base import AlgorithmBase

class YourAlgorithm(AlgorithmBase):
    """你的算法类"""
    
    def compute_observation(self, state):
        """
        计算观测向量
        
        Args:
            state: development_state_t 消息对象
            
        Returns:
            np.ndarray: 观测向量（shape: [num_obs]）
        """
        obs = np.zeros(self.model_params['num_obs'], dtype=np.float32)
        # 你的观测计算逻辑
        # ...
        return obs
    
    def process_action(self, state, action):
        """
        处理动作并更新控制命令
        
        Args:
            state: development_state_t 消息对象
            action: np.ndarray, 模型输出的动作向量（shape: [num_actions]）
        """
        # 计算目标关节位置
        target_pos = action * self.model_params['action_scale'] + \
                     self.model_params['default_joint_pos']
        
        # 组织关节命令
        joint_positions = {
            'L_Leg_q': target_pos[0:6].tolist(),
            'R_Leg_q': target_pos[6:12].tolist(),
            'waist_q': float(target_pos[12]),
            'L_Arm_q': target_pos[13:17].tolist(),
            'R_Arm_q': target_pos[17:21].tolist()
        }
        
        joint_kp = {
            'L_Leg_kp': self.model_params['joint_stiffness'][0:6].tolist(),
            'R_Leg_kp': self.model_params['joint_stiffness'][6:12].tolist(),
            'waist_kp': float(self.model_params['joint_stiffness'][12]),
            'L_Arm_kp': self.model_params['joint_stiffness'][13:17].tolist(),
            'R_Arm_kp': self.model_params['joint_stiffness'][17:21].tolist()
        }
        
        joint_kd = {
            'L_Leg_kd': self.model_params['joint_damping'][0:6].tolist(),
            'R_Leg_kd': self.model_params['joint_damping'][6:12].tolist(),
            'waist_kd': float(self.model_params['joint_damping'][12]),
            'L_Arm_kd': self.model_params['joint_damping'][13:17].tolist(),
            'R_Arm_kd': self.model_params['joint_damping'][17:21].tolist()
        }
        
        # 更新命令缓存（由500Hz发送循环发送）
        with self.latest_command_lock:
            self.latest_command = {
                'enable_development_mode': True,
                'is_rl_mode': self.config['rl_mode']['is_rl_mode'],
                'joint_positions': joint_positions,
                'joint_velocities': {'L_Leg_qd': [0.0]*6, 'R_Leg_qd': [0.0]*6, 
                                     'waist_qd': 0.0, 'L_Arm_qd': [0.0]*4, 'R_Arm_qd': [0.0]*4},
                'joint_kp': joint_kp,
                'joint_kd': joint_kd
            }

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Your Algorithm Runner')
    parser.add_argument('--config', type=str, default='config.yaml',
                       help='Path to configuration file')
    args = parser.parse_args()
    
    algorithm = YourAlgorithm(args.config)
    algorithm.run()

if __name__ == "__main__":
    main()
```

### 4. 修改配置文件

编辑 `config.yaml`，配置策略路径、LCM通道、执行频率等参数（详见"配置文件说明"部分）。

### 5. 运行算法

```bash
python3 your_algorithm_name.py --config config.yaml
```

## 配置文件说明

### 策略配置（`policy`）

```yaml
policy:
  type: "onnx"  # 或 "pt" (PyTorch)
  path: "./HumanActornet.onnx"  # 策略文件路径（相对或绝对路径）
  read_metadata: true  # 是否从ONNX模型的metadata读取配置
  warmup_count: 500  # 模型预热次数（使用真实状态进行推理，检测action异常）
  action_threshold: 200.0  # Action异常检测阈值
```

### LCM配置（`lcm`）

```yaml
lcm:
  url: ""  # LCM URL（空字符串表示使用默认）
  state_channel: "E3_development_state"  # 订阅的状态通道
  command_channel: "E3_development_command"  # 发布的命令通道
  robot_id: "E3"  # 机器人ID（必须与状态机配置一致）
```

### 执行配置（`execution`）

```yaml
execution:
  frequency: 50.0  # 策略推理频率（Hz），建议50Hz（20ms周期）
  lcm_send_frequency: 500.0  # LCM发送频率（Hz），固定500Hz
  auto_start: true  # 是否自动开始（收到状态消息后自动开始开发模式）
  auto_end: true  # 是否自动结束（算法执行完成后自动结束开发模式）
  max_execution_time: 0.0  # 最大执行时间（秒），0表示无限制
```

### 强化学习模式配置（`rl_mode`）

```yaml
rl_mode:
  is_rl_mode: true  # true: 状态机对踝关节进行扭矩计算，kp/kd清零
                    # false: 直接使用外部传过来的kp/kd/期望角度/期望速度/期望扭矩
```

### 模型参数（`model_params`）

如果 `read_metadata=false`，需要手动配置：

```yaml
model_params:
  num_actions: 21  # 动作维度
  num_obs: 114  # 观测维度
  default_joint_pos: [0.0, ...]  # 默认关节位置（21维）
  joint_stiffness: [100.0, ...]  # 关节刚度Kp（21维）
  joint_damping: [10.0, ...]  # 关节阻尼Kd（21维）
  action_scale: [0.25, ...]  # 动作缩放因子（21维）
```

### 调试配置（`debug`）

```yaml
debug:
  print_config: false  # 是否打印配置文件内容
  print_metadata: false  # 是否打印ONNX模型metadata内容
```

## 关节顺序

标准关节顺序（21维）：
- **左腿（6维）**: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
- **右腿（6维）**: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
- **腰部（1维）**: waist
- **左臂（4维）**: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow
- **右臂（4维）**: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow

如果模型使用不同的关节顺序，需要在 `compute_observation` 和 `process_action` 中进行映射。

## ONNX模型Metadata

框架支持从 ONNX 模型的 metadata 中自动读取配置信息，支持的 metadata 键包括：

- `joint_names`: 关节名称列表（逗号分隔）
- `default_joint_pos`: 默认关节位置（逗号分隔的浮点数）
- `joint_stiffness`: 关节刚度Kp（逗号分隔的浮点数）
- `joint_damping`: 关节阻尼Kd（逗号分隔的浮点数）
- `action_scale`: 动作缩放因子（逗号分隔的浮点数）
- `num_actions`: 动作维度
- `num_obs`: 观测维度

## MuJoCo仿真器

`dance_algorithm/mujoco_simulator.py` 提供了一个 MuJoCo 仿真器，可用于算法验证和调试：

```bash
cd dance_algorithm
python3 mujoco_simulator.py --xml path/to/scene.xml [--headless]
```

仿真器通过 LCM 与算法通信，模拟真实机器人的状态发布和命令接收。

## 依赖要求

- Python 3.6+
- numpy
- pyyaml
- lcm (Python bindings)
- onnxruntime (如果使用ONNX模型)
- torch (如果使用PyTorch模型)
- onnx (如果从metadata读取配置)
- mujoco (如果使用MuJoCo仿真器)

## 注意事项

1. **机器人ID匹配**：确保配置文件中的 `robot_id` 与状态机配置一致
2. **LCM通道**：确保状态通道和命令通道与状态机配置一致
3. **关节顺序**：确保模型输出的关节顺序与机器人关节顺序匹配
4. **执行频率**：建议策略推理频率设置为 50Hz（20ms周期），与状态机控制频率一致
5. **LCM发送频率**：固定为 500Hz，由基类内部管理
6. **线程安全**：`process_action` 中更新 `latest_command` 时需要使用 `self.latest_command_lock`
7. **动作缓冲区**：访问 `self.action_buffer` 时需要使用 `self.action_buffer_lock`
8. **安全**：算法执行前确保机器人处于安全状态，建议先进行仿真测试

## 示例算法：DanceAlgorithm

跳舞算法实现展示了如何使用框架：

- 从数据集（`.npz` 文件）加载动作序列
- 使用 ONNX 模型（`HumanActornet.onnx`）进行动作生成
- 支持 yaw 对齐（motion初始yaw与robot初始yaw对齐）
- 完整的观测计算包括 anchor orientation

使用方法：

```bash
cd dance_algorithm
python3 run_algorithm.py --config config.yaml
```

## LCM接口使用（直接使用）

如果需要直接使用 LCM 接口（不继承基类），可以这样使用：

```python
from lcm_interface import LCMInterface

lcm = LCMInterface(config)
lcm.register_state_callback(your_callback_function)
lcm.start()

# 在主循环中处理消息
while True:
    lcm.handle(10)  # 100ms超时
    state = lcm.get_latest_state()
    # 处理状态...
    
    # 发送命令
    lcm.send_command(
        enable_development_mode=True,
        is_rl_mode=True,
        joint_positions={...},
        joint_kp={...},
        joint_kd={...}
    )
```

## 架构说明

本框架采用双线程架构：

1. **主线程**：处理 LCM 消息接收
2. **策略推理线程**：按配置频率（如50Hz）运行策略推理
3. **LCM发送线程**：固定500Hz频率发送控制命令

这种架构确保了：
- 策略推理的实时性（50Hz）
- 控制命令的高频发送（500Hz）
- 线程安全的状态访问

## 许可证

请参考项目根目录的许可证文件。

