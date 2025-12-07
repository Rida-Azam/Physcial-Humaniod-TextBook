# Nav2 Humanoid Package

This package provides a customized Navigation2 stack configuration specifically designed for humanoid robots.

## Overview

The Nav2 Humanoid package includes:

- Custom launch files for humanoid navigation
- Parameter configurations optimized for bipedal locomotion
- Behavior trees tailored for humanoid movement patterns
- Integration with humanoid-specific controllers

## Features

- **Humanoid-Specific Parameters**: Configured for humanoid robot kinematics and dynamics
- **Bipedal-Friendly Controllers**: Uses rotation-shim controller for smoother turning
- **SMAC Planner Integration**: Uses SMAC (Sparse Marcov Chain) planner for 2.5D planning
- **Custom Behavior Trees**: Optimized for humanoid navigation patterns
- **Safety Considerations**: Includes appropriate costmap inflation and recovery behaviors

## Installation

1. Clone this repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url>
   ```

2. Install dependencies:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace:
   ```bash
   colcon build --packages-select nav2_humanoid
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

To launch the humanoid navigation stack:

```bash
ros2 launch nav2_humanoid humanoid_navigation.launch.py
```

## Configuration

The main configuration file is located at `config/nav2_params.yaml`. Key humanoid-specific parameters include:

- `robot_radius`: Set to appropriate value for humanoid footprint
- `controller_server`: Uses rotation-shim controller for smoother turning
- `planner_server`: Configured to use SMAC planner for 2.5D planning
- `costmap` parameters: Adjusted for humanoid sensor placement and stability

## Customization

To adapt this configuration for your specific humanoid robot:

1. Update the `robot_radius` in the costmap configuration
2. Adjust controller parameters for your robot's dynamics
3. Modify the `base_frame_id` and `odom_frame_id` to match your robot's TF tree
4. Update sensor topics to match your robot's configuration

## Integration with Isaac ROS

This package can be integrated with Isaac ROS for hardware-accelerated perception:

1. Ensure Isaac ROS packages are installed
2. Launch perception nodes alongside this navigation stack
3. Use Isaac ROS Nav2 Bridge for optimized perception-navigation integration

## Troubleshooting

- If navigation fails frequently, check that costmap inflation parameters are appropriate for your robot's size
- For unstable behavior, verify that odometry is accurate and TF tree is properly configured
- If planning fails, ensure that the map is properly loaded and the global planner is configured correctly

## See Also

- [Navigation2 Documentation](https://navigation.ros.org/)
- [Isaac ROS Navigation Integration](https://nvidia-isaac-ros.github.io/repositories_and_packages/navigation/index.html)
- Textbook Chapter 10: Isaac ROS – Hardware-Accelerated Perception & Navigation