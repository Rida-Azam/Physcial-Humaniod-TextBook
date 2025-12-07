# Gazebo Environment for ROS 2

This directory contains the Docker configuration for running Gazebo Harmonic with ROS 2 Jazzy integration.

## Overview

This Docker environment provides:
- ROS 2 Jazzy (Desktop)
- Gazebo Harmonic
- ROS 2 Gazebo packages and plugins
- Basic tools for simulation and robot control

## Building the Docker Image

To build the Gazebo-ROS 2 Docker image:

```bash
cd docker/gazebo_env
docker build -t gazebo-ros2 .
```

## Running the Container

To run the container with GUI support (requires X11 forwarding on Linux or XQuartz on macOS):

```bash
# On Linux
xhost +local:docker
docker run -it --rm \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --network=host \
  gazebo-ros2

# On macOS with XQuartz
docker run -it --rm \
  --env="DISPLAY=host.docker.internal:0" \
  gazebo-ros2
```

For headless operation (no GUI), simply run:

```bash
docker run -it --rm gazebo-ros2
```

## Usage Examples

Once inside the container, you can:

1. **Launch Gazebo with ROS 2 bridge**:
   ```bash
   source /ros_ws/install/setup.bash
   ros2 launch gazebo_ros empty_world.launch.py
   ```

2. **Spawn a robot model**:
   ```bash
   ros2 run gazebo_ros spawn_entity.py -entity my_robot -file /path/to/robot.urdf -x 0 -y 0 -z 1
   ```

3. **Run ROS 2 nodes that interact with Gazebo**:
   ```bash
   ros2 run your_package your_gazebo_controller
   ```

## Integration with Textbook Examples

This environment is designed to work with the URDF models and ROS 2 workspaces in the textbook:

- URDF models from `src/urdf_models/` can be loaded into Gazebo
- ROS 2 workspaces from `src/ros2_workspaces/` are built into the image
- Simulation examples from the textbook chapters can be run directly

For more details, refer to the textbook chapters on simulation and robot control.