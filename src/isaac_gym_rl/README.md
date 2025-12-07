# Isaac Gym RL Walking Policies

This package provides reinforcement learning-based walking policies for humanoid robots, trained in Isaac Gym and deployable on real robots.

## Overview

The package includes:

- Pre-trained RL policy for humanoid walking
- Training script for Isaac Gym
- Configuration files for different walking behaviors
- ROS 2 interface for deployment on real robots

## Installation

1. Install Isaac Gym (requires NVIDIA GPU and appropriate drivers)

2. Clone this repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository-url>
```

3. Build the workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select isaac_gym_rl
source install/setup.bash
```

## Training in Isaac Gym

To train a new walking policy:

1. Launch Isaac Gym with your humanoid robot model
2. Run the training script:
```bash
ros2 run isaac_gym_rl train_walking_policy
```

The training process uses Proximal Policy Optimization (PPO) to learn stable walking gaits that maintain balance while achieving desired velocities.

## Deployment on Real Robots

To deploy a pre-trained policy on a real humanoid robot:

1. Ensure the robot is properly calibrated
2. Launch the walking policy node:
```bash
ros2 run isaac_gym_rl rl_walking_policy
```

3. Send velocity commands via the `/cmd_vel` topic

## Configuration

The behavior of the RL policy can be adjusted through the configuration file:

- `state_dim`: Dimension of the input state vector
- `action_dim`: Dimension of the output action vector
- `hidden_dim`: Size of hidden layers in the neural network
- `reward_weights`: Weights for different components of the reward function
- `safety_limits`: Balance and fall prevention thresholds

## Simulation

The package includes simulation capabilities for testing policies before real robot deployment:

- Gazebo simulation with ROS 2 bridge
- Isaac Sim integration for high-fidelity physics
- RViz2 visualization of robot state and planned trajectories

## Performance Considerations

- RL policies require significant computational resources
- Ensure sufficient GPU memory for neural network inference
- Monitor robot balance and safety limits during deployment
- Adjust control frequency based on robot capabilities

## Troubleshooting

- If the robot falls frequently, check the reward weights and safety limits
- If walking is unstable, consider retraining with different parameters
- Monitor CPU/GPU usage during deployment
- Verify joint limits and safety constraints are properly configured

## References

This implementation is based on state-of-the-art RL methods for humanoid locomotion, including:

- Proximal Policy Optimization (PPO)
- Deep Reinforcement Learning for Robust Robot Control
- Sim-to-Real Transfer Techniques for Humanoid Locomotion