#!/bin/bash

# Setup script for Isaac Sim ROS Bridge environment

set -e  # Exit immediately if a command exits with a non-zero status

echo "Isaac Sim ROS Bridge Environment Setup Script"
echo "============================================="

# Check if running on the correct platform
if [[ "$OSTYPE" != "linux-gnu"* ]]; then
    echo "Error: This script is designed for Linux systems only."
    exit 1
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

    # Check if CUDA version is sufficient (12.4+ recommended for Isaac Sim 2023.2+)
    if (( $(echo "$CUDA_VERSION < 12.4" | bc -l) )); then
        echo "Warning: CUDA 12.4 or higher is recommended. You have $CUDA_VERSION"
        echo "Consider upgrading CUDA for optimal Isaac Sim performance."
    fi
else
    echo "CUDA toolkit not found. Please install CUDA 12.4 or higher."
    exit 1
fi

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

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "ROS 2 not sourced. Please source your ROS 2 installation:"
    echo "source /opt/ros/jazzy/setup.bash"
    exit 1
else
    echo "ROS 2 $ROS_DISTRO detected"
fi

# Check for required Python packages
REQUIRED_PYTHON_PKGS=(
    "rclpy"
    "sensor_msgs"
    "geometry_msgs"
    "std_msgs"
    "cv_bridge"
    "tf2_ros"
    "tf2_geometry_msgs"
)

MISSING_PYTHON_PKGS=()

for pkg in "${REQUIRED_PYTHON_PKGS[@]}"; do
    if python3 -c "import $pkg" 2>/dev/null; then
        echo "✓ $pkg found"
    else
        echo "✗ $pkg not found"
        MISSING_PYTHON_PKGS+=("$pkg")
    fi
done

if [ ${#MISSING_PYTHON_PKGS[@]} -ne 0 ]; then
    echo
    echo "Installing missing Python packages..."
    for pkg in "${MISSING_PYTHON_PKGS[@]}"; do
        # Try to install via pip first, then via apt
        if pip3 install "$pkg" 2>/dev/null; then
            echo "✓ Installed $pkg via pip"
        else
            # Convert ROS package name to apt package name
            APT_PKG=$(echo "$pkg" | sed 's/_/-/g')
            if sudo apt-get install -y "python3-$APT_PKG" 2>/dev/null; then
                echo "✓ Installed $pkg via apt"
            else
                echo "⚠ Could not install $pkg"
            fi
        fi
    done
fi

# Check for Isaac Sim Python API
if python3 -c "import omni" 2>/dev/null; then
    echo "✓ Isaac Sim Python API available"
else
    echo "✗ Isaac Sim Python API not available"
    echo "Make sure Isaac Sim is properly installed and environment is set up"
    echo "Try: source /opt/isaac-sim/python.sh"
    exit 1
fi

# Create necessary directories
echo
echo "Creating workspace directories..."
mkdir -p ~/isaac_sim_ws/src
cd ~/isaac_sim_ws

# If this is the first run, copy the package
if [ ! -d "src/isaac_sim_ros_bridge" ]; then
    echo "Copying Isaac Sim ROS Bridge package to workspace..."
    cp -r /path/to/isaac_sim_ros_bridge src/  # This would need to be adjusted based on actual location
fi

# Build the workspace
echo
echo "Building workspace..."
colcon build --packages-select isaac_sim_ros_bridge

# Source the workspace
source install/setup.bash

# Create an alias for easier access
echo
echo "Creating convenience aliases..."

# Add to .bashrc if not already present
if ! grep -q "isaac_sim_ros_bridge" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# Isaac Sim ROS Bridge aliases" >> ~/.bashrc
    echo "alias isaac_sim_build='cd ~/isaac_sim_ws && colcon build --packages-select isaac_sim_ros_bridge'" >> ~/.bashrc
    echo "alias isaac_sim_launch='cd ~/isaac_sim_ws && source install/setup.bash && ros2 launch isaac_sim_ros_bridge isaac_sim_bridge.launch.py'" >> ~/.bashrc
    echo "alias isaac_sim_source='cd ~/isaac_sim_ws && source install/setup.bash'" >> ~/.bashrc
    echo "alias isaac_sim_reset='pkill -f IsaacSim; pkill -f ros2'" >> ~/.bashrc

    echo "Added convenience aliases to ~/.bashrc"
    echo "Restart your shell or run 'source ~/.bashrc' to use the aliases"
fi

# Environment validation
echo
echo "Validating environment setup..."

# Test Isaac Sim connection
echo "Testing Isaac Sim connection..."
if python3 -c "
import carb
import omni
try:
    omni.kit.acquire_app()
    print('✓ Isaac Sim connection successful')
except Exception as e:
    print(f'✗ Isaac Sim connection failed: {e}')
    exit 1
" > /dev/null 2>&1; then
    echo "✓ Isaac Sim connection test passed"
else
    echo "✗ Isaac Sim connection test failed"
    exit 1
fi

# Test ROS 2 connection
echo "Testing ROS 2 connection..."
if ros2 topic list > /dev/null 2>&1; then
    echo "✓ ROS 2 connection test passed"
else
    echo "✗ ROS 2 connection test failed"
    exit 1
fi

echo
echo "Environment setup completed successfully!"
echo
echo "Next steps:"
echo "1. Restart your shell or run 'source ~/.bashrc'"
echo "2. Launch Isaac Sim: '/opt/isaac-sim/isaac-sim.sh'"
echo "3. In a new terminal: 'ros2 launch isaac_sim_ros_bridge isaac_sim_bridge.launch.py'"
echo
echo "Useful aliases:"
echo "  isaac_sim_build   - Build the Isaac Sim ROS Bridge package"
echo "  isaac_sim_launch  - Launch the Isaac Sim ROS Bridge"
echo "  isaac_sim_source  - Source the workspace"
echo "  isaac_sim_reset   - Kill Isaac Sim and ROS 2 processes"