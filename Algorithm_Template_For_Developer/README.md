# Yobotics HumanoidE3 Algorithm Deployment Template

<div align="right">
  <a href="README.md">English</a> | <a href="README_zh.md">中文</a>
</div>

## Video Tutorial

<video width="800" controls>
  <source src="How to develop E3?.mp4" type="video/mp4">
  Your browser does not support video playback. Please download the video file to view: <a href="How to develop E3?.mp4">How to develop E3?.mp4</a>
</video>

This repository provides a development template framework for deploying external algorithms on the Yobotics HumanoidE3 robot. Developers can quickly implement custom control algorithms based on this framework and communicate with the robot's state machine via LCM.

## Directory Structure

```
Algorithm_Template_For_Developer/
├── README.md                    # This document
├── __init__.py                  # Package initialization file
├── algorithm_base.py            # Algorithm base class (AlgorithmBase)
├── lcm_interface.py             # LCM communication interface (LCMInterface)
└── dance_algorithm/             # Dancing algorithm example
    ├── config.yaml              # Algorithm configuration file
    ├── run_algorithm.py         # Algorithm execution script (DanceAlgorithm implementation)
    ├── mujoco_simulator.py      # MuJoCo simulator (for algorithm verification)
    ├── HumanActornet.onnx       # ONNX policy model
    └── dataset/                 # Dataset directory
        └── E3Getup350.npz       # Motion sequence dataset
```

## Core Components

### 1. AlgorithmBase (Algorithm Base Class)

All custom algorithms should inherit from the `AlgorithmBase` class. This class provides the following features:

- **Policy Model Loading**: Supports ONNX and PyTorch (.pt) models, automatically reads configuration from ONNX metadata
- **LCM Communication Management**: Automatically handles LCM subscription and publishing
- **Dual-Thread Execution Architecture**:
  - Policy inference thread: Runs at configured frequency (e.g., 50Hz)
  - LCM send thread: Fixed 500Hz high-frequency control command sending
- **Development Mode Control**: Automatically handles development mode start and end
- **Model Warmup**: Uses real state for warmup inference, detects action anomalies
- **Thread Safety**: All state access is protected by locks

#### Required Methods to Implement

- `compute_observation(state)`: Compute observation vector based on state
- `process_action(state, action)`: Process action and send control command (by updating `latest_command`)

#### Optional Methods to Override

- `on_development_mode_start()`: Callback when development mode starts
- `on_development_mode_end()`: Callback when development mode ends

### 2. LCMInterface (LCM Communication Interface)

Encapsulates all LCM-related communication functions:

- Subscribe to robot state messages (`development_state_t`)
- Publish control command messages (`development_command_t`)
- State caching and thread-safe access interface
- Supports state message callbacks

### 3. DanceAlgorithm (Example Algorithm)

A dancing algorithm example implemented based on `deploy_mujoco_1Step.py`, demonstrating:

- Loading motion sequences from datasets
- Using ONNX models for action generation
- Complete observation computation (including anchor orientation, yaw alignment, etc.)
- Joint order mapping (model order ↔ XML order)

## Quick Start

### 1. Create New Algorithm Folder

```bash
cd Algorithm_Template_For_Developer
mkdir your_algorithm_name
cd your_algorithm_name
```

### 2. Create Configuration File and Algorithm Script

Copy the configuration file template from the example algorithm:

```bash
cp ../dance_algorithm/config.yaml .
cp ../dance_algorithm/run_algorithm.py ./your_algorithm_name.py
```

### 3. Implement Your Algorithm Class

Edit `your_algorithm_name.py`, inherit from `AlgorithmBase` and implement the necessary methods:

```python
#!/usr/bin/env python3
import sys
import os
import numpy as np

# Add parent directory to path
_parent_dir = os.path.join(os.path.dirname(__file__), '..')
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)

from algorithm_base import AlgorithmBase

class YourAlgorithm(AlgorithmBase):
    """Your algorithm class"""
    
    def compute_observation(self, state):
        """
        Compute observation vector
        
        Args:
            state: development_state_t message object
            
        Returns:
            np.ndarray: Observation vector (shape: [num_obs])
        """
        obs = np.zeros(self.model_params['num_obs'], dtype=np.float32)
        # Your observation computation logic
        # ...
        return obs
    
    def process_action(self, state, action):
        """
        Process action and update control command
        
        Args:
            state: development_state_t message object
            action: np.ndarray, action vector output by model (shape: [num_actions])
        """
        # Calculate target joint positions
        target_pos = action * self.model_params['action_scale'] + \
                     self.model_params['default_joint_pos']
        
        # Organize joint commands
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
        
        # Update command cache (sent by 500Hz send loop)
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

### 4. Modify Configuration File

Edit `config.yaml` to configure policy path, LCM channels, execution frequency, and other parameters (see "Configuration File Reference" section).

### 5. Run Algorithm

```bash
python3 your_algorithm_name.py --config config.yaml
```

## Configuration File Reference

### Policy Configuration (`policy`)

```yaml
policy:
  type: "onnx"  # or "pt" (PyTorch)
  path: "./HumanActornet.onnx"  # Policy file path (relative or absolute)
  read_metadata: true  # Whether to read configuration from ONNX model metadata
  warmup_count: 500  # Model warmup count (using real state for inference, detecting action anomalies)
  action_threshold: 200.0  # Action anomaly detection threshold
```

### LCM Configuration (`lcm`)

```yaml
lcm:
  url: ""  # LCM URL (empty string means use default)
  state_channel: "E3_development_state"  # State subscription channel
  command_channel: "E3_development_command"  # Command publishing channel
  robot_id: "E3"  # Robot ID (must match state machine configuration)
```

### Execution Configuration (`execution`)

```yaml
execution:
  frequency: 50.0  # Policy inference frequency (Hz), recommended 50Hz (20ms period)
  lcm_send_frequency: 500.0  # LCM send frequency (Hz), fixed at 500Hz
  auto_start: true  # Auto start (automatically start development mode after receiving state message)
  auto_end: true  # Auto end (automatically end development mode after algorithm execution completes)
  max_execution_time: 0.0  # Maximum execution time (seconds), 0 means unlimited
```

### Reinforcement Learning Mode Configuration (`rl_mode`)

```yaml
rl_mode:
  is_rl_mode: true  # true: State machine performs torque calculation for ankle joints, kp/kd cleared
                    # false: Directly use kp/kd/desired angle/desired velocity/desired torque from external source
```

### Model Parameters (`model_params`)

If `read_metadata=false`, manual configuration is required:

```yaml
model_params:
  num_actions: 21  # Action dimension
  num_obs: 114  # Observation dimension
  default_joint_pos: [0.0, ...]  # Default joint positions (21 dimensions)
  joint_stiffness: [100.0, ...]  # Joint stiffness Kp (21 dimensions)
  joint_damping: [10.0, ...]  # Joint damping Kd (21 dimensions)
  action_scale: [0.25, ...]  # Action scale factors (21 dimensions)
```

### Debug Configuration (`debug`)

```yaml
debug:
  print_config: false  # Whether to print configuration file contents
  print_metadata: false  # Whether to print ONNX model metadata contents
```

## Joint Order

Standard joint order (21 dimensions):
- **Left Leg (6 dims)**: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
- **Right Leg (6 dims)**: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
- **Waist (1 dim)**: waist
- **Left Arm (4 dims)**: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow
- **Right Arm (4 dims)**: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow

If the model uses a different joint order, mapping is required in `compute_observation` and `process_action`.

## ONNX Model Metadata

The framework supports automatically reading configuration information from ONNX model metadata. Supported metadata keys include:

- `joint_names`: List of joint names (comma-separated)
- `default_joint_pos`: Default joint positions (comma-separated floating point numbers)
- `joint_stiffness`: Joint stiffness Kp (comma-separated floating point numbers)
- `joint_damping`: Joint damping Kd (comma-separated floating point numbers)
- `action_scale`: Action scale factors (comma-separated floating point numbers)
- `num_actions`: Action dimension
- `num_obs`: Observation dimension

## MuJoCo Simulator

`dance_algorithm/mujoco_simulator.py` provides a MuJoCo simulator that can be used for algorithm verification and debugging:

```bash
cd dance_algorithm
python3 mujoco_simulator.py --xml path/to/scene.xml [--headless]
```

The simulator communicates with the algorithm via LCM, simulating the real robot's state publishing and command reception.

## Requirements

- Python 3.6+
- numpy
- pyyaml
- lcm (Python bindings)
- onnxruntime (if using ONNX models)
- torch (if using PyTorch models)
- onnx (if reading configuration from metadata)
- mujoco (if using MuJoCo simulator)

## Notes

1. **Robot ID Matching**: Ensure the `robot_id` in the configuration file matches the state machine configuration
2. **LCM Channels**: Ensure state and command channels match the state machine configuration
3. **Joint Order**: Ensure the joint order output by the model matches the robot's joint order
4. **Execution Frequency**: Recommended policy inference frequency is 50Hz (20ms period), matching the state machine control frequency
5. **LCM Send Frequency**: Fixed at 500Hz, managed internally by the base class
6. **Thread Safety**: Use `self.latest_command_lock` when updating `latest_command` in `process_action`
7. **Action Buffer**: Use `self.action_buffer_lock` when accessing `self.action_buffer`
8. **Safety**: Ensure the robot is in a safe state before algorithm execution, recommend simulation testing first

## Example Algorithm: DanceAlgorithm

The dancing algorithm implementation demonstrates how to use the framework:

- Load motion sequences from dataset (`.npz` files)
- Use ONNX model (`HumanActornet.onnx`) for action generation
- Support yaw alignment (align motion initial yaw with robot initial yaw)
- Complete observation computation including anchor orientation

Usage:

```bash
cd dance_algorithm
python3 run_algorithm.py --config config.yaml
```

## LCM Interface Usage (Direct Usage)

If you need to use the LCM interface directly (without inheriting the base class), you can use it like this:

```python
from lcm_interface import LCMInterface

lcm = LCMInterface(config)
lcm.register_state_callback(your_callback_function)
lcm.start()

# Process messages in main loop
while True:
    lcm.handle(10)  # 100ms timeout
    state = lcm.get_latest_state()
    # Process state...
    
    # Send command
    lcm.send_command(
        enable_development_mode=True,
        is_rl_mode=True,
        joint_positions={...},
        joint_kp={...},
        joint_kd={...}
    )
```

## Architecture

This framework uses a dual-thread architecture:

1. **Main Thread**: Handles LCM message reception
2. **Policy Inference Thread**: Runs policy inference at configured frequency (e.g., 50Hz)
3. **LCM Send Thread**: Sends control commands at fixed 500Hz frequency

This architecture ensures:
- Real-time policy inference (50Hz)
- High-frequency control command sending (500Hz)
- Thread-safe state access

## License

Please refer to the license file in the project root directory.
