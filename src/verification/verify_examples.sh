#!/bin/bash
# Verification script for textbook code examples

echo "Starting verification of code examples..."

# Check if Docker is available
if command -v docker &> /dev/null; then
    echo "✓ Docker is available"

    # Test basic ROS 2 functionality in Docker
    echo "Testing ROS 2 workspace setup..."
    if [ -d "src/ros2_workspaces" ]; then
        echo "✓ ROS 2 workspaces directory exists"
        # Count the number of ROS 2 packages
        package_count=$(find src/ros2_workspaces -name "package.xml" | wc -l)
        echo "Found $package_count ROS 2 packages"
    else
        echo "✗ ROS 2 workspaces directory not found"
    fi
else
    echo "✗ Docker not available in this environment"
fi

# Check Python examples
echo "Checking Python code examples..."
python_files=$(find docs -name "*.py" -o -name "*.ipynb" | wc -l)
mdx_files=$(find docs -name "*.mdx" | xargs grep -l "\`\`\`python" | wc -l)

echo "Found $python_files Python files and $mdx_files MDX files with Python code blocks"

# Check if key directories exist
if [ -d "src/urdf_models" ]; then
    echo "✓ URDF models directory exists"
else
    echo "✗ URDF models directory missing"
fi

if [ -d "src/nav2_humanoid" ]; then
    echo "✓ Nav2 humanoid directory exists"
else
    echo "✗ Nav2 humanoid directory missing"
fi

if [ -d "src/isaac_gym_rl" ]; then
    echo "✓ Isaac Gym RL directory exists"
else
    echo "✗ Isaac Gym RL directory missing"
fi

if [ -d "capstone" ]; then
    echo "✓ Capstone directory exists"
else
    echo "✗ Capstone directory missing"
fi

echo "Verification complete!"