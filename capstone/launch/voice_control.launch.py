from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # Declare launch arguments
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

    # Audio input node
    audio_input = Node(
        package='audio_capture',
        executable='audio_capture_node',
        name='audio_input',
        parameters=[{
            'use_sim_time': use_sim_time,
            'device': 'default',
            'sample_rate': 16000,
            'channels': 1
        }],
        output='screen'
    )

    # Voice processor node
    voice_processor = Node(
        package='capstone',
        executable='voice_processor',
        name='voice_processor',
        parameters=[params_file],
        remappings=[
            ('/audio_input', '/audio_captured'),
        ],
        output='screen'
    )

    # Natural language understanding node
    nlu_processor = Node(
        package='capstone',
        executable='nlu_processor',
        name='nlu_processor',
        parameters=[params_file],
        output='screen'
    )

    # Voice command executor
    voice_executor = Node(
        package='capstone',
        executable='voice_command_executor',
        name='voice_executor',
        parameters=[params_file],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)

    # Add nodes
    ld.add_action(audio_input)
    ld.add_action(voice_processor)
    ld.add_action(nlu_processor)
    ld.add_action(voice_executor)

    return ld