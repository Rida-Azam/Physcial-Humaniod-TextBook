# Isaac Sim ROS Bridge

The Isaac Sim ROS Bridge provides bidirectional communication between Isaac Sim and ROS 2 for humanoid robotics simulation and control.

## Overview

This package enables:
- Real-time simulation of humanoid robots in Isaac Sim
- Bidirectional communication with ROS 2 ecosystem
- Integration with standard ROS 2 tools and frameworks
- Support for complex humanoid control and perception tasks

## Features

- **Joint State Publishing**: Publishes real-time joint states from Isaac Sim to ROS 2
- **Command Interface**: Receives joint position, velocity, and effort commands from ROS 2
- **Sensor Simulation**: Simulates IMU, cameras, and other sensors
- **Odometry**: Provides robot odometry in simulated environment
- **TF Broadcasting**: Publishes coordinate transforms for robot frames
- **Physics Integration**: Accurate physics simulation with collision detection

## Installation

### Prerequisites

- ROS 2 Jazzy
- Isaac Sim 2023.2 or later
- NVIDIA GPU with CUDA support
- Python 3.10+

### Build Instructions

```bash
# Clone the repository
git clone <repository_url>
cd <repository_name>

# Build the package
colcon build --packages-select isaac_sim_ros_bridge

# Source the workspace
source install/setup.bash
```

## Usage

### Launch the Bridge

```bash
# Launch Isaac Sim with ROS 2 bridge
ros2 launch isaac_sim_ros_bridge isaac_sim_bridge.launch.py
```

### Parameters

- `namespace`: Namespace for all nodes (default: "")
- `use_sim_time`: Use simulation time (default: true)
- `params_file`: Path to parameters file (default: config/bridge_params.yaml)

## Configuration

The bridge behavior can be configured through the `config/bridge_params.yaml` file:

- **Connection settings**: Timeout and reconnection parameters
- **Publishing rates**: Frequencies for different sensor data streams
- **Simulation settings**: Physics and rendering timesteps
- **Robot parameters**: Joint names and URDF path
- **Sensor settings**: IMU and camera parameters
- **Controller settings**: Joint controller configuration
- **Safety settings**: Limits and collision checking
- **Performance settings**: Threading and resource usage

## Topics Published

- `/joint_states`: Joint positions, velocities, and efforts
- `/odom`: Robot odometry
- `/imu/data`: IMU sensor data
- `/camera/rgb/image_raw`: RGB camera images
- `/camera/rgb/camera_info`: Camera calibration info
- `/tf` and `/tf_static`: Transform information

## Topics Subscribed

- `/joint_commands`: Joint position commands
- `/cmd_vel`: Velocity commands
- `/joint_trajectory`: Trajectory commands

## Integration with Humanoid Control

The bridge is designed to work seamlessly with humanoid control systems:

1. **Perception**: Camera and IMU data for state estimation
2. **Planning**: Access to robot state for motion planning
3. **Control**: Send commands to robot joints
4. **Simulation**: Validate control policies in simulation before real-robot deployment

## Troubleshooting

### Common Issues

1. **Connection Problems**:
   - Verify Isaac Sim is running and accessible
   - Check network configuration if running remotely
   - Ensure Isaac Sim Python API is available

2. **Performance Issues**:
   - Adjust physics and rendering timesteps in config
   - Verify GPU is properly utilized
   - Check CPU and memory usage

3. **Joint State Issues**:
   - Verify joint names match URDF definition
   - Check joint limits and ranges
   - Ensure controllers are properly configured

### Debugging

Enable debug output:
```bash
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG
ros2 launch isaac_sim_ros_bridge isaac_sim_bridge.launch.py
```

Monitor topics:
```bash
# Monitor joint states
ros2 topic echo /joint_states

# Monitor IMU data
ros2 topic echo /imu/data

# Monitor camera data
ros2 run image_view image_view __ns:=/camera
```

## Performance Optimization

For optimal performance:

- Use appropriate physics timesteps (typically 1ms for humanoid robots)
- Adjust rendering quality based on performance needs
- Configure appropriate controller update rates
- Enable GPU acceleration where possible
- Use efficient collision meshes

## Safety Considerations

- Always test control policies in simulation first
- Implement safety limits and constraints
- Monitor robot state during operation
- Have emergency stop procedures in place

## ROS 2 Compatibility

This package is designed for ROS 2 Jazzy and may require modifications for other distributions.

## Contributing

See the CONTRIBUTING.md file for details on contributing to this package.

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.