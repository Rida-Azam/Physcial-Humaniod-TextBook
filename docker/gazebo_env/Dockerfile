# Use ROS 2 Jazzy with Gazebo Harmonic
FROM osrf/ros:jazzy-desktop

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Set locale
RUN apt update && apt install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# Install Gazebo Harmonic
RUN apt update && \
    apt install -y software-properties-common && \
    add-apt-repository -y universe && \
    curl -sSL http://get.gazebosim.org | sh && \
    apt install -y gazebo-harmonic

# Install ROS 2 Gazebo packages
RUN apt update && \
    apt install -y \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-gazebo-dev \
    ros-jazzy-gazebo-msgs \
    ros-jazzy-gazebo-plugins \
    ros-jazzy-gazebo-ros \
    ros-jazzy-gazebo-ros2-control \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-sim-demos

# Install additional tools
RUN apt update && \
    apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    wget \
    curl \
    gnupg \
    lsb-release \
    build-essential \
    cmake

# Install Python packages
RUN pip3 install --upgrade pip && \
    pip3 install \
    numpy \
    transforms3d \
    matplotlib

# Create workspace directory
WORKDIR /ros_ws

# Copy ROS workspace if provided (optional)
COPY . /ros_ws/src/

# Source ROS 2 environment
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/jazzy/setup.bash && \
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Build the workspace
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install

# Source the workspace
RUN echo "source /ros_ws/install/setup.bash" >> ~/.bashrc

# Set up environment variables for Gazebo
ENV GAZEBO_MODEL_PATH=/ros_ws/src/models:$GAZEBO_MODEL_PATH
ENV GAZEBO_RESOURCE_PATH=/ros_ws/src/worlds:$GAZEBO_RESOURCE_PATH

# Expose Gazebo ports (optional, for GUI forwarding)
EXPOSE 11345
EXPOSE 11346

# Default command
CMD ["bash"]