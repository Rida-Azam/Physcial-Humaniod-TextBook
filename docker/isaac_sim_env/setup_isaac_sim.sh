#!/bin/bash

# Script to help set up Isaac Sim environment with ROS 2

set -e  # Exit on any error

echo "Isaac Sim Environment Setup Script"
echo "=================================="

# Check if running on Ubuntu 22.04
if [[ ! "$(lsb_release -rs)" == "22.04" ]]; then
    echo "Warning: This script is designed for Ubuntu 22.04. You are running $(lsb_release -rs)"
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check for NVIDIA GPU
if ! command -v nvidia-smi &> /dev/null; then
    echo "Error: NVIDIA drivers not found. Please install NVIDIA drivers first."
    exit 1
else
    echo "NVIDIA GPU detected:"
    nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader,nounits
fi

# Check CUDA version
if command -v nvcc &> /dev/null; then
    CUDA_VERSION=$(nvcc --version | grep "release" | cut -d " " -f 6 | cut -c 2-)
    echo "CUDA version: $CUDA_VERSION"
    if (( $(echo "$CUDA_VERSION < 12.4" | bc -l) )); then
        echo "Warning: CUDA 12.4 or higher is recommended. You have $CUDA_VERSION"
    fi
else
    echo "CUDA toolkit not found. Please install CUDA 12.4 or higher."
    exit 1
fi

# Install NVIDIA Container Toolkit
echo "Installing NVIDIA Container Toolkit..."
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

apt-get update
apt-get install -y nvidia-container-toolkit

# Configure Docker
echo "Configuring Docker for NVIDIA runtime..."
nvidia-ctk runtime configure --runtime=docker
systemctl restart docker

echo "NVIDIA Container Toolkit installed and Docker configured."

# Check if Isaac Sim is installed
ISAAC_SIM_PATH="/opt/isaac-sim"
if [ -d "$ISAAC_SIM_PATH" ]; then
    echo "Isaac Sim detected at: $ISAAC_SIM_PATH"
else
    echo "Isaac Sim not found at $ISAAC_SIM_PATH"
    echo "Please install Isaac Sim from the NVIDIA Developer portal:"
    echo "https://developer.nvidia.com/isaac-sim"
    echo
    echo "After installation, run this script again."
    exit 1
fi

# Check ROS 2 installation
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "ROS 2 Jazzy detected"
else
    echo "ROS 2 Jazzy not found. Please install ROS 2 Jazzy first."
    exit 1
fi

# Install Isaac Sim ROS 2 Bridge dependencies
echo "Installing ROS 2 packages for Isaac Sim bridge..."
apt-get update
apt-get install -y \
    ros-jazzy-rosbridge-suite \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup

echo "Setup complete! You can now build the Isaac Sim Docker image:"
echo "cd docker/isaac_sim_env"
echo "docker build -t ros2-isaac-sim:jazzy ."

echo
echo "To run Isaac Sim with the container:"
echo "docker run -it --gpus all --env=\"DISPLAY\" --volume=\"/tmp/.X11-unix:/tmp/.X11-unix:rw\" --volume=\"$ISAAC_SIM_PATH:/isaac-sim\" --net=host --privileged ros2-isaac-sim:jazzy"