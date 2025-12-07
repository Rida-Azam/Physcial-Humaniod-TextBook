# Capstone Project: Autonomous Humanoid from Spoken Command

This repository contains the complete implementation for the capstone project: an autonomous humanoid robot system that responds to spoken commands. The system integrates vision, language, and action to perform tasks like "pick up the red cup" from a single spoken command.

## Table of Contents
- [Overview](#overview)
- [Architecture](#architecture)
- [Installation](#installation)
- [Simulation](#simulation)
- [Real Robot Deployment](#real-robot-deployment)
- [Usage](#usage)
- [Evaluation](#evaluation)
- [Troubleshooting](#troubleshooting)

## Overview

The system consists of multiple integrated components:

1. **Voice Processing**: Converts speech to text using Whisper
2. **Natural Language Understanding**: Parses commands and extracts entities
3. **Perception**: Detects objects and localizes them in 3D space
4. **Planning**: Generates action sequences to achieve the requested task
5. **Execution**: Controls the humanoid robot to execute planned actions
6. **Safety**: Multiple layers of safety checks and emergency procedures

## Architecture

```
[Voice Input] → [Speech Recognition] → [NLU] → [Task Planner] → [Action Executor] → [Robot Control]
      ↑              ↓                    ↓         ↓              ↓                ↓
[Environment] ← [Perception] ← [VSLAM] ← [Fusion] ← [Scheduler] ← [Safety Layer] ← [Robot]
```

## Installation

### Prerequisites

- Ubuntu 22.04
- ROS 2 Jazzy
- NVIDIA GPU with CUDA 12.4+
- Python 3.10+

### Setup

1. Clone the repository:
```bash
git clone https://github.com/your-username/physical-ai-textbook-capstone.git
cd physical-ai-textbook-capstone
```

2. Install Python dependencies:
```bash
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu124
pip3 install transformers openai-whisper
pip3 install opencv-python numpy scipy matplotlib
pip3 install pyquaternion transforms3d
```

3. Install ROS 2 dependencies:
```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-vision-opencv ros-jazzy-cv-bridge
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-rosbridge-suite
```

4. Build the ROS packages:
```bash
colcon build --packages-select perception_pipeline nav2_humanoid isaac_gym_rl
source install/setup.bash
```

## Simulation

### Gazebo Simulation

To run the system in simulation:

1. Launch Gazebo with the humanoid robot:
```bash
ros2 launch capstone simulation.launch.py
```

2. In another terminal, launch the perception pipeline:
```bash
ros2 launch perception_pipeline perception_pipeline_launch.py
```

3. Launch the voice processing and planning nodes:
```bash
ros2 launch capstone voice_control.launch.py
```

4. Send voice commands via the provided interface or directly via ROS topics.

### Isaac Sim (Optional)

For more realistic simulation with Isaac Sim:

1. Install Isaac Sim following NVIDIA's documentation
2. Launch Isaac Sim with the humanoid environment
3. Run the Isaac Sim bridge:
```bash
ros2 launch capstone isaac_sim_bridge.launch.py
```

## Real Robot Deployment

### Hardware Requirements

- Humanoid robot platform (e.g., Unitree G1/H1)
- RGB-D camera
- Microphone array
- NVIDIA Jetson Orin (NX/Nano) or equivalent edge computer
- 2.4GHz/5GHz WiFi

### Setup

1. Install ROS 2 Jazzy on the robot's edge computer
2. Configure network settings for communication with external systems
3. Calibrate sensors (camera, IMU, etc.)
4. Test individual components before full system integration

### Deployment Instructions

1. Upload the code to the robot:
```bash
# Copy the entire workspace to the robot
scp -r ~/ros2_ws user@robot-ip:/home/user/
```

2. Build on the robot:
```bash
ssh user@robot-ip
cd ~/ros2_ws
colcon build --packages-select capstone perception_pipeline nav2_humanoid
source install/setup.bash
```

3. Launch the system:
```bash
# On the robot
ros2 launch capstone real_robot.launch.py

# On the control station
ros2 launch capstone voice_control.launch.py
```

## Usage

### Basic Command

To start the complete system:
```bash
ros2 launch capstone complete_system.launch.py
```

### Voice Commands

The system understands various commands:

- "Pick up the red cup"
- "Go to the kitchen"
- "Grasp the blue box"
- "Move to the table"
- "Navigate to the chair"

### Testing Commands

For testing without voice:
```bash
# Send a text command
ros2 topic pub /spoken_command std_msgs/String "data: 'pick up the red cup'"

# Check system status
ros2 topic echo /system_status
```

## Evaluation

### Performance Metrics

The system is evaluated based on:

1. **Task Success Rate**: Percentage of successfully completed tasks
2. **Command Accuracy**: Correct interpretation of spoken commands
3. **Execution Time**: Time from command to task completion
4. **Safety Compliance**: Number of safety violations or emergency stops
5. **Robustness**: Performance under varying conditions

### Running Evaluation

```bash
# Run the evaluation script
python3 evaluation/run_evaluation.py --task "pick_up_red_cup" --trials 10

# View results
python3 evaluation/analyze_results.py
```

## Troubleshooting

### Common Issues

**Problem**: Speech recognition not working
**Solution**: Check audio input device and permissions
```bash
# Test audio input
arecord -D hw:0,0 -f cd test.wav
```

**Problem**: Robot not moving
**Solution**: Check joint controllers and safety limits
```bash
# Check controller status
ros2 control list_controllers
```

**Problem**: Object detection failing
**Solution**: Verify camera calibration and lighting conditions

### Debugging

Enable debug output:
```bash
# Launch with debug logging
ros2 launch capstone complete_system.launch.py log_level:=debug
```

Check individual components:
```bash
# Monitor perception
ros2 topic echo /object_detections

# Monitor navigation
ros2 topic echo /vslam/odom

# Monitor voice commands
ros2 topic echo /voice_command
```

## Safety Considerations

⚠️ **IMPORTANT SAFETY INFORMATION** ⚠️

- Always maintain visual supervision of the robot
- Ensure emergency stop procedures are accessible
- Test in controlled environments before complex tasks
- Implement multiple safety layers and fallback procedures
- Follow all robot manufacturer safety guidelines

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for details on how to contribute to this project.

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Based on the Physical AI & Humanoid Robotics textbook
- Inspired by OpenVLA, RT-2, and other VLA research
- Built with ROS 2, NVIDIA Isaac, and PyTorch