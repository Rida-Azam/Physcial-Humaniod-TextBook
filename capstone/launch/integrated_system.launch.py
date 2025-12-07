from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    enable_rviz = LaunchConfiguration('enable_rviz')

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

    # Group all voice processing nodes
    voice_processing_group = GroupAction(
        actions=[
            # Audio input node
            Node(
                package='audio_capture',
                executable='audio_capture_node',
                name='audio_input',
                namespace=namespace,
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'device': 'default',
                    'sample_rate': 16000,
                    'channels': 1
                }],
                output='screen'
            ),

            # Voice processor node (Whisper)
            Node(
                package='capstone',
                executable='voice_processor',
                name='voice_processor',
                namespace=namespace,
                parameters=[params_file],
                output='screen'
            ),

            # Natural Language Understanding processor
            Node(
                package='capstone',
                executable='nlu_processor',
                name='nlu_processor',
                namespace=namespace,
                parameters=[params_file],
                output='screen'
            ),
        ]
    )

    # Group planning nodes
    planning_group = GroupAction(
        actions=[
            # LLM Planner
            Node(
                package='capstone',
                executable='llm_planner',
                name='llm_planner',
                namespace=namespace,
                parameters=[params_file],
                output='screen'
            ),
        ]
    )

    # Group execution nodes
    execution_group = GroupAction(
        actions=[
            # ROS Executor
            Node(
                package='capstone',
                executable='ros_executor',
                name='ros_executor',
                namespace=namespace,
                parameters=[params_file],
                output='screen'
            ),

            # Integration node
            Node(
                package='capstone',
                executable='integration_node',
                name='integration_node',
                namespace=namespace,
                parameters=[params_file],
                output='screen'
            ),
        ]
    )

    # Include perception pipeline
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

    # Include Nav2 for humanoid
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
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_rviz)
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_enable_rviz_cmd)

    # Add node groups and launches
    ld.add_action(voice_processing_group)
    ld.add_action(planning_group)
    ld.add_action(execution_group)
    ld.add_action(perception_launch)
    ld.add_action(nav2_launch)
    ld.add_action(rviz_node)

    return ld