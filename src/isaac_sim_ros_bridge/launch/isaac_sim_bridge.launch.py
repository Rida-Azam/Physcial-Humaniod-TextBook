from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace all nodes will be placed under')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('isaac_sim_ros_bridge'),
            'config',
            'bridge_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # Isaac Sim ROS Bridge node
    isaac_sim_bridge = Node(
        package='isaac_sim_ros_bridge',
        executable='isaac_sim_ros_bridge',
        name='isaac_sim_bridge',
        namespace=namespace,
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Include other necessary launch files
    # This would typically include robot state publisher, controllers, etc.

    # Robot State Publisher for URDF
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_state_publisher'),
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'publish_frequency': '50.0'
        }.items()
    )

    # Controller manager (for humanoid robot controllers)
    controller_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('controller_manager'),
                'launch',
                'ros2_control_node.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)

    # Add nodes
    ld.add_action(isaac_sim_bridge)
    ld.add_action(robot_state_publisher)
    ld.add_action(controller_manager)

    return ld