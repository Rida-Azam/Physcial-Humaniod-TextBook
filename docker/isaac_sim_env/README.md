# Isaac Sim Environment for ROS 2

This directory contains the Docker configuration for running NVIDIA Isaac Sim with ROS 2 Jazzy integration.

## Overview

This Docker environment provides:
- NVIDIA Isaac Sim base environment (placeholder)
- ROS 2 Jazzy (Desktop) with Isaac ROS packages
- ROS-Gazebo bridge packages
- Basic tools for simulation and robot control

## Important Note

NVIDIA Isaac Sim is a proprietary simulation environment that requires:
1. An NVIDIA Developer account
2. Access to NVIDIA's container registry
3. Appropriate licensing

The Dockerfile in this directory provides a template and documentation for setting up Isaac Sim with ROS 2, but the actual Isaac Sim installation requires NVIDIA-specific credentials and images.

## Building the Docker Image

To build the base ROS 2 environment with Isaac ROS packages:

```bash
cd docker/isaac_sim_env
docker build -t isaac-ros2-base .
```

## Actual Isaac Sim Setup

To properly set up Isaac Sim with ROS 2:

1. **Sign up for NVIDIA Developer account** at https://developer.nvidia.com/
2. **Access Isaac Sim** through NVIDIA Omniverse
3. **Pull the Isaac Sim Docker image** (requires NVIDIA credentials):
   ```bash
   # Login to NVIDIA Container Registry
   docker login nvcr.io
   # Pull Isaac Sim image
   docker pull nvcr.io/nvidia/isaac-sim:4.0.0
   ```

4. **Run Isaac Sim with ROS 2 bridge**:
   ```bash
   docker run --gpus all -it --rm \
     --network=host \
     --env "NVIDIA_VISIBLE_DEVICES=all" \
     --env "NVIDIA_DRIVER_CAPABILITIES=all" \
     --volume $HOME/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache/Kit:rw \
     --volume $HOME/docker/isaac-sim/cache/ov:/isaac-sim/kit/cache/ov:rw \
     --volume $HOME/docker/isaac-sim/cache/computecache:/home/ubuntu/.nvidia-omniverse/computecache:rw \
     --volume $HOME/docker/isaac-sim/logs:/isaac-sim/logs:rw \
     --volume $HOME/docker/isaac-sim/data:/isaac-sim/data:rw \
     --volume $HOME/docker/isaac-sim/extensions:/isaac-sim/kit/exts:rw \
     --volume $HOME/docker/isaac-sim/config:/isaac-sim/kit/config:rw \
     --volume $HOME/docker/isaac-sim/content:/isaac-sim/content:rw \
     --env "OMNIVERSE_HEADLESS=0" \
     nvcr.io/nvidia/isaac-sim:4.0.0
   ```

## ROS 2 Integration

Isaac Sim has built-in ROS 2 bridge capabilities through the Isaac ROS GEMs. To enable ROS 2 communication:

1. **Inside Isaac Sim**, enable the ROS 2 Bridge extension
2. **Configure ROS 2 endpoints** in Isaac Sim
3. **Map Isaac Sim topics** to ROS 2 topics

## Isaac ROS Packages

This environment includes Isaac ROS packages for:
- Perception (Isaac ROS Apriltag, Isaac ROS DNN Image Encoding, etc.)
- Navigation (Isaac ROS Nav2 Bridge)
- Manipulation (Isaac ROS Manipulation)
- Sensor processing (Isaac ROS Stereo DNN, Isaac ROS Optical Flow, etc.)

## Integration with Textbook Examples

This environment is designed to work with the URDF models and ROS 2 workspaces in the textbook:

- URDF models from `src/urdf_models/` can be imported into Isaac Sim
- ROS 2 workspaces from `src/ros2_workspaces/` can communicate with Isaac Sim
- Simulation examples from the textbook chapters can be run with Isaac Sim

For more details, refer to the textbook chapters on simulation and robot control with Isaac Sim.