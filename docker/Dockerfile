FROM ubuntu:22.04

# Install common dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    curl \
    wget \
    python3 \
    python3-pip \
    sudo \
    locales

# Set locale
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# Install ROS 2 Jazzy (Ubuntu 22.04)
RUN apt update && apt install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8
RUN apt install -y software-properties-common
RUN add-apt-repository -y universe
RUN apt update && apt install -y curl
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update
RUN apt upgrade -y
RUN apt install -y ros-jazzy-desktop

# Placeholder for Isaac Sim headless installation (highly complex, usually requires NVIDIA base image and specific setup)
# RUN echo "Isaac Sim Headless installation commands would go here"

# Install Node.js for Docusaurus
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
RUN apt-get install -y nodejs

# Install Docusaurus dependencies globally to speed up builds (optional)
# RUN npm install -g yarn # If using yarn
# RUN npm install -g @docusaurus/cli

# Create a non-root user
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL='(ALL) NOPASSWD:ALL' > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME/workspace

# Copy project files into the container
COPY --chown=$USERNAME:$USERNAME . .

# Install project dependencies (assuming package.json exists)
RUN npm install

# Expose Docusaurus default port
EXPOSE 3000

CMD ["npm", "start"]
