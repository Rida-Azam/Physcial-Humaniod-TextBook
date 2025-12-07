# ROS 2 Workspaces

This directory contains various ROS 2 workspaces and example packages for the Physical AI & Humanoid Robotics textbook.

## Available Packages

1.  **demo_nodes_py**: Basic ROS 2 examples in Python including:
    *   `talker`/`listener`: Publisher/subscriber example
    *   `add_two_ints_server`/`add_two_ints_client`: Service server/client example
    *   `fibonacci_action_server`/`fibonacci_action_client`: Action server/client example

2.  **simple_robot_control**: Simple robot control and simulation examples including:
    *   `robot_controller`: A controller that moves a robot and avoids obstacles
    *   `robot_sensor_sim`: A simple laser scan simulator

## Usage

To use these packages:

1.  Source your ROS 2 environment:
    ```bash
    source /opt/ros/jazzy/setup.bash  # Or your ROS 2 installation path
    ```

2.  Navigate to the workspace containing the package:
    ```bash
    cd src/ros2_workspaces
    ```

3.  Build the workspace:
    ```bash
    colcon build --packages-select demo_nodes_py simple_robot_control
    ```

4.  Source the local setup:
    ```bash
    source install/setup.bash
    ```

5.  Run a node from one of the packages:
    ```bash
    ros2 run demo_nodes_py talker
    ```

For more details on each example, see the source code and documentation within each package.
