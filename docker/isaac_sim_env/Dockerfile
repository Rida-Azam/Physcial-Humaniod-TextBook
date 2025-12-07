# Use Isaac Sim base image (using a placeholder image as the actual Isaac Sim image requires NVIDIA credentials)
# For demonstration purposes, we'll use an NVIDIA base image and document the actual setup
FROM nvidia/cudagl:12.1-devel-ubuntu22.04

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Set locale
RUN apt update && apt install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# Install basic dependencies
RUN apt update && apt install -y \
    wget \
    curl \
    git \
    python3 \
    python3-pip \
    python3-dev \
    build-essential \
    software-properties-common \
    lsb-release \
    gnupg \
    curl \
    sudo \
    locales \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Jazzy dependencies
RUN apt update && apt install -y \
    software-properties-common \
    && add-apt-repository universe \
    && apt update \
    && apt install -y \
    curl \
    gnupg \
    lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt update \
    && apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-sim-demos \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --upgrade pip && \
    pip3 install \
    numpy \
    transforms3d \
    matplotlib \
    opencv-python \
    && rm -rf ~/.cache/pip

# Set up ROS 2 environment
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/jazzy/setup.bash && \
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Create workspace directory
WORKDIR /ros_ws

# Copy ROS workspace if provided (optional)
COPY . /ros_ws/src/

# Build the workspace
RUN source /opt/ros/jazzy/setup.bash && \
    apt update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/* && \
    colcon build --symlink-install

# Source the workspace
RUN echo "source /ros_ws/install/setup.bash" >> ~/.bashrc

# Install Isaac Sim dependencies (placeholder - actual installation requires NVIDIA credentials)
# In a real scenario, you would:
# 1. Have NVIDIA credentials to access Isaac Sim Docker images
# 2. Pull the Isaac Sim image from NVIDIA's registry
# 3. Set up the proper environment for Isaac Sim

# For demonstration, we'll just document the actual process:

# ACTUAL ISAAC SIM SETUP (to be done separately):
# 1. Sign up for NVIDIA Developer account
# 2. Access Isaac Sim through NVIDIA Omniverse
# 3. Pull Isaac Sim Docker image:
#    docker pull nvcr.io/nvidia/isaac-sim:4.0.0
#
# 4. Run Isaac Sim with ROS 2 bridge:
#    docker run --gpus all -it --rm \
#      --network=host \
#      --env "NVIDIA_VISIBLE_DEVICES=all" \
#      --env "NVIDIA_DRIVER_CAPABILITIES=all" \
#      --volume $HOME/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache/Kit:rw \
#      --volume $HOME/docker/isaac-sim/cache/ov:/isaac-sim/kit/cache/ov:rw \
#      --volume $HOME/docker/isaac-sim/cache/computecache:/home/ubuntu/.nvidia-omniverse/computecache:rw \
#      --volume $HOME/docker/isaac-sim/logs:/isaac-sim/logs:rw \
#      --volume $HOME/docker/isaac-sim/data:/isaac-sim/data:rw \
#      --volume $HOME/docker/isaac-sim/extensions:/isaac-sim/kit/exts:rw \
#      --volume $HOME/docker/isaac-sim/config:/isaac-sim/kit/config:rw \
#      --volume $HOME/docker/isaac-sim/content:/isaac-sim/content:rw \
#      --env "OMNIVERSE_HEADLESS=0" \
#      nvcr.io/nvidia/isaac-sim:4.0.0

# ROS 2 Bridge setup (to be done inside Isaac Sim)
# Isaac Sim has built-in ROS 2 bridge capabilities through the Isaac ROS GEMs
# You would typically:
# 1. Enable the ROS 2 Bridge extension in Isaac Sim
# 2. Configure ROS 2 endpoints
# 3. Map Isaac Sim topics to ROS 2 topics

# Environment variables for Isaac Sim integration
ENV ISAAC_ROS_WS=/ros_ws
ENV ROS_DOMAIN_ID=1

# Expose ports for Isaac Sim (when running with networking)
EXPOSE 55555
EXPOSE 55556
EXPOSE 33501

# Default command
CMD ["bash"]