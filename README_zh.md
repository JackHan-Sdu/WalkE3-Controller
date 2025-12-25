# 人形机器人控制器

> 人形机器人强化学习控制与部署框架，支持仿真与硬件双模式实时RL策略部署

**Language / 语言**: [English](README.md) | [中文](README_zh.md)

---

## 项目简介

本代码仓为人形机器人提供完整的控制系统，支持强化学习策略从仿真到硬件的无缝部署。主要特性包括：

- **双模式运行**：通过配置文件在 MuJoCo 仿真和硬件部署之间切换
- **实时 RL 部署**：部署 ONNX 格式的 RL 策略，并进行优化的推理
- **基于 FSM 的控制**：模块化状态机架构，易于扩展
- **安全优先**：多层安全检查系统，保护硬件安全

---

## 环境配置

### 系统要求

**必须使用 Ubuntu 20.04 系统**

### 配置环境

使用提供的脚本一键配置 conda 环境：

```bash
./scripts/setup_conda_env.sh
```

脚本将自动：
- 创建 conda 环境（默认名称：`humanoid_controller`）
- 安装必需的 Python 包（mujoco, numpy, pyyaml, lcm 等）
- 配置库路径和环境变量

### 删除环境

如需删除已配置的环境，运行：

```bash
./scripts/remove_conda_env.sh
```

---

## 遥控器校准

激活 conda 环境后，需要先校准遥控器：

```bash
# 激活环境
conda activate humanoid_controller

# 运行校准脚本
python scripts/calibrate_gamepad.py
```

校准脚本将：
1. 检测所有可用的游戏手柄设备
2. 引导您完成按钮校准（LB, RB, Back, Start, A, B, X, Y）
3. 引导您完成摇杆校准（左/右摇杆 X/Y）
4. 生成配置文件

**请将生成的映射配置复制到 `config.yaml` 的 `gamepad` 部分**。

**注意**：确保您的 conda 环境路径在 `config.yaml` 的 `algorithm_launcher.conda_search_paths` 配置中。如果您的 conda 环境路径不在列表中，请添加您的实际路径，例如：
```yaml
algorithm_launcher:
  conda_search_paths:
    - "/data/ubuntu20_user/conda_env/humanoid_controller"
    - "$HOME/miniconda3/envs/humanoid_controller"
    - "$HOME/anaconda3/envs/humanoid_controller"
    - "/opt/conda/envs/humanoid_controller"
    - "您的实际conda环境路径"  # 添加您的路径
```

---

## 使用说明

### 启动仿真环境

```bash
# 确保已激活 conda 环境
conda activate humanoid_controller

# 启动 MuJoCo 仿真环境
python mujoco_sim/mujoco_simulator.py
```

### 启动算法

在另一个终端中：

```bash
# 确保已激活 conda 环境
conda activate humanoid_controller

# 启动控制器算法（推荐：指定配置文件）
bash scripts/run_controller.sh ./config.yaml

# 或直接指定可执行文件路径
bash scripts/run_controller.sh ./human_ctrl
# 或开发环境：
bash scripts/run_controller.sh ./build/user/MIT_Controller/human_ctrl
```

**注意**：`run_controller.sh` 脚本可以接受配置文件路径（`config.yaml`）或可执行文件路径作为参数。如果指定配置文件，脚本会自动查找对应的可执行文件。

---

## 控制模式

框架支持以下四种控制模式：

### 模式切换

使用遥控器进行模式切换：
- **START** 按钮：切换到下一个模式
- **BACK** 按钮：切换到上一个模式

![Logitech F310 手柄](F310.png)

*Logitech F310 游戏手柄按钮示意图*

### 可用模式

| 模式 | 描述 |
|------|------|
| `RL_HYBRID` | 混合模式（过渡 → 舞蹈 → 行走） |
| `DEVELOPMENT` | 通过 LCM 的外部算法控制 |
| `RL_WALK` | 基于 RL 的行走，支持摇杆控制 |
| `PASSIVE` | 被动模式，无主动控制 |

### 使用说明

#### 模式顺序

系统支持的模式切换顺序为：`OFF` → `RL_HYBRID` → `RL_WALK` → `DEVELOPMENT`，通过 **START** 和 **BACK** 按钮可以循环切换。

#### 基本操作流程

1. **启动系统**
   - 等待算法启动完成后，按下 **START** 按钮进入 `RL_HYBRID` 模式

2. **机器人站立**
   - 进入 `RL_HYBRID` 模式后，等待机器人手臂张开
   - 将**左摇杆 Y 轴往上推**，机器人即可自动站起来
   - 机器人会自动进入行走模式

3. **行走控制**
   - 在行走模式下，移动**左右摇杆**可以控制机器人来回行走

4. **机器人爬下**
   - 当不想继续控制时，在 `RL_HYBRID` 模式下将**左摇杆 Y 轴往下推**
   - 机器人会自主爬下
   - 按下 **BACK** 按钮回到 `PASSIVE` 模式

5. **模式切换**
   - 在 `RL_HYBRID` 模式下机器人站立后，可以继续通过 **START** 按钮切换到 `RL_WALK` 或 `DEVELOPMENT` 模式
   - 使用 **START** 和 **BACK** 按钮可以在所有模式之间循环切换

#### 开发者模式

在 `DEVELOPMENT` 模式下，可以通过实现 `Algorithm_Template_For_Developer` 目录中的算法基元进行二次开发。具体使用方法请参考该目录中的 README 文档。

---

## 联系方式

如有问题或需要支持：

- **作者**：Han Jiang
- **邮箱**：jh18954242606@163.com
