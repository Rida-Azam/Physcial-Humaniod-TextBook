# URDF Models

This directory contains URDF (Unified Robot Description Format) models for simple humanoid robots used in the Physical AI & Humanoid Robotics textbook.

## Available Models

1.  **simple_humanoid.urdf**: A basic humanoid model with essential links and joints for educational purposes.
2.  **advanced_humanoid.urdf**: A more complex humanoid model with additional joints and refined physical properties.
3.  **simulation_humanoid.urdf**: A humanoid model specifically designed for simulation with Gazebo, including ROS control plugins.

## Usage

These URDF files can be used with ROS 2 and simulation environments like Gazebo:

1.  **Visualize the robot in RViz**:
    ```bash
    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat simple_humanoid.urdf)'
    ```

2.  **Load into Gazebo**:
    ```bash
    ros2 run gazebo_ros spawn_entity.py -entity simple_humanoid -file simple_humanoid.urdf
    ```

3.  **Use with robot_state_publisher**:
    ```bash
    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(xacro simple_humanoid.urdf)'
    ```

For more details on URDF and robot description, refer to the textbook chapters on robot description and simulation.
