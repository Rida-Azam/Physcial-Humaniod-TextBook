from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_voice = LaunchConfiguration('enable_voice')
    robot_model = LaunchConfiguration('robot_model')

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace all nodes will be placed under')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('capstone'),
            'config',
            'capstone_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_enable_rviz_cmd = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Launch RViz2 for visualization')

    declare_enable_voice_cmd = DeclareLaunchArgument(
        'enable_voice',
        default_value='true',
        description='Enable voice processing nodes')

    declare_robot_model_cmd = DeclareLaunchArgument(
        'robot_model',
        default_value='unitree_g1',
        description='Robot model to use (unitree_g1, unitree_h1, etc.)')

    # Include perception pipeline launch
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('perception_pipeline'),
                'launch',
                'perception_pipeline_launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    # Include Nav2 launch for humanoid
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_humanoid'),
                'launch',
                'humanoid_navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    # Voice processing nodes
    voice_processing_group = GroupAction(
        condition=IfCondition(enable_voice),
        actions=[
            Node(
                package='capstone',
                executable='voice_processor',
                name='voice_processor',
                namespace=namespace,
                parameters=[params_file],
                output='screen'
            ),
            Node(
                package='capstone',
                executable='nlu_processor',
                name='nlu_processor',
                namespace=namespace,
                parameters=[params_file],
                output='screen'
            )
        ]
    )

    # Planning and execution nodes
    planning_node = Node(
        package='capstone',
        executable='task_planner',
        name='task_planner',
        namespace=namespace,
        parameters=[params_file],
        output='screen'
    )

    execution_node = Node(
        package='capstone',
        executable='action_executor',
        name='action_executor',
        namespace=namespace,
        parameters=[params_file],
        output='screen'
    )

    # Safety monitor node
    safety_monitor = Node(
        package='capstone',
        executable='safety_monitor',
        name='safety_monitor',
        namespace=namespace,
        parameters=[params_file],
        output='screen'
    )

    # RViz2 for visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('capstone'),
        'rviz',
        'capstone_system.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(enable_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_enable_rviz_cmd)
    ld.add_action(declare_enable_voice_cmd)
    ld.add_action(declare_robot_model_cmd)

    # Add launch files and nodes
    ld.add_action(perception_launch)
    ld.add_action(nav2_launch)
    ld.add_action(voice_processing_group)
    ld.add_action(planning_node)
    ld.add_action(execution_node)
    ld.add_action(safety_monitor)
    ld.add_action(rviz_node)

    return ld