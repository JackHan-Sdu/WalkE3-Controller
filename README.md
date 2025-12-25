# Humanoid Robot Controller

> A universal deployment framework for humanoid robots, supporting real-time Reinforcement Learning (RL) policy deployment with dual-mode operation (Simulation/Hardware).

**Language / 语言**: [English](README.md) | [中文](README_zh.md)

---

## Overview

This repository provides a comprehensive control system for humanoid robots, enabling seamless deployment of reinforcement learning policies from simulation to hardware. Key features include:

- **Dual-Mode Operation**: Switch between MuJoCo simulation and hardware deployment via configuration
- **Real-Time RL Deployment**: Deploy ONNX-format RL policies with optimized inference
- **FSM-Based Control**: Modular state machine architecture for easy extension
- **Safety-First**: Multi-layer safety checking system for hardware protection

---

## Environment Setup

### System Requirements

**Ubuntu 20.04 is required**

### Setup Environment

Use the provided script to configure the conda environment:

```bash
./scripts/setup_conda_env.sh
```

The script will automatically:
- Create a conda environment (default name: `humanoid_controller`)
- Install required Python packages (mujoco, numpy, pyyaml, lcm, etc.)
- Configure library paths and environment variables

### Remove Environment

To remove the configured environment:

```bash
./scripts/remove_conda_env.sh
```

---

## Gamepad Calibration

After activating the conda environment, calibrate your gamepad:

```bash
# Activate environment
conda activate humanoid_controller

# Run calibration script
python scripts/calibrate_gamepad.py
```

The calibration script will:
1. Detect all available gamepad devices
2. Guide you through button calibration (LB, RB, Back, Start, A, B, X, Y)
3. Guide you through joystick calibration (left/right stick X/Y)
4. Generate a configuration file

**Copy the generated mappings to the `gamepad` section in `config.yaml`**.

**Note**: Ensure your conda environment path is included in the `algorithm_launcher.conda_search_paths` configuration in `config.yaml`. If your conda environment path is not in the list, please add your actual path, for example:
```yaml
algorithm_launcher:
  conda_search_paths:
    - "/data/ubuntu20_user/conda_env/humanoid_controller"
    - "$HOME/miniconda3/envs/humanoid_controller"
    - "$HOME/anaconda3/envs/humanoid_controller"
    - "/opt/conda/envs/humanoid_controller"
    - "your_actual_conda_path"  # Add your path
```

---

## Usage

### Start Simulation

```bash
# Make sure conda environment is activated
conda activate humanoid_controller

# Start MuJoCo simulation
python mujoco_sim/mujoco_simulator.py
```

### Start Controller

In another terminal:

```bash
# Make sure conda environment is activated
conda activate humanoid_controller

# Start controller algorithm (recommended: specify config file)
bash scripts/run_controller.sh ./config.yaml

# Or directly specify executable file path
bash scripts/run_controller.sh ./human_ctrl
# Or for development environment:
bash scripts/run_controller.sh ./build/user/MIT_Controller/human_ctrl
```

**Note**: The `run_controller.sh` script can accept either a config file path (`config.yaml`) or an executable file path as an argument. If a config file is specified, the script will automatically locate the corresponding executable file.

---

## Control Modes

The framework supports the following four control modes:

### Mode Switching

Use the gamepad to switch modes:
- **START** button: Switch to next mode
- **BACK** button: Switch to previous mode

![Logitech F310 Gamepad](F310.png)

*Logitech F310 gamepad button diagram*

### Available Modes

| Mode | Description |
|------|-------------|
| `RL_HYBRID` | Hybrid mode (transition → dance → walk) |
| `DEVELOPMENT` | External algorithm control via LCM |
| `RL_WALK` | RL-based walking with joystick control |
| `PASSIVE` | Passive mode, no active control |

### Usage Instructions

#### Mode Sequence

The system supports mode switching in the following order: `OFF` → `RL_HYBRID` → `RL_WALK` → `DEVELOPMENT`. Use the **START** and **BACK** buttons to cycle through modes.

#### Basic Operation Flow

1. **Start System**
   - After the algorithm starts, press the **START** button to enter `RL_HYBRID` mode

2. **Robot Stand Up**
   - After entering `RL_HYBRID` mode, wait for the robot's arms to open
   - Push the **left joystick Y-axis up**, and the robot will automatically stand up
   - The robot will automatically enter walking mode

3. **Walking Control**
   - In walking mode, move the **left and right joysticks** to control the robot's walking

4. **Robot Lie Down**
   - When you want to stop controlling, push the **left joystick Y-axis down** in `RL_HYBRID` mode
   - The robot will autonomously lie down
   - Press the **BACK** button to return to `PASSIVE` mode

5. **Mode Switching**
   - After the robot stands up in `RL_HYBRID` mode, you can continue to switch to `RL_WALK` or `DEVELOPMENT` mode using the **START** button
   - Use **START** and **BACK** buttons to cycle through all modes

#### Development Mode

In `DEVELOPMENT` mode, you can perform secondary development by implementing algorithm primitives in the `Algorithm_Template_For_Developer` directory. For detailed usage, please refer to the README in that directory.

---

## Contact

For questions or support:

- **Author**: Han Jiang
- **Email**: jh18954242606@163.com
