from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    robot_model = LaunchConfiguration('robot_model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_ip = LaunchConfiguration('robot_ip')

    # Declare launch arguments
    declare_robot_model_cmd = DeclareLaunchArgument(
        'robot_model',
        default_value='unitree_g1',
        description='Robot model to use (unitree_g1, unitree_h1, etc.)')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_robot_ip_cmd = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.123.10',
        description='IP address of the robot')

    # Robot hardware interface node
    robot_interface = Node(
        package='capstone',
        executable='robot_interface',
        name='robot_interface',
        parameters=[{
            'robot_model': robot_model,
            'robot_ip': robot_ip,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Camera driver node
    camera_driver = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera_driver',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'color_width': 640,
            'color_height': 480,
            'depth_width': 640,
            'depth_height': 480,
            'enable_infra1': False,
            'enable_infra2': False,
            'publish_tf': True,
            'tf_publish_rate': 30.0
        }],
        output='screen'
    )

    # IMU driver node
    imu_driver = Node(
        package='imu_driver',  # Replace with actual IMU driver package
        executable='imu_node',
        name='imu_driver',
        parameters=[{
            'frame_id': 'imu_link',
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Launch the complete capstone system
    capstone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('capstone'),
                'launch',
                'complete_system.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_robot_model_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_ip_cmd)

    # Add nodes
    ld.add_action(robot_interface)
    ld.add_action(camera_driver)
    ld.add_action(imu_driver)
    ld.add_action(capstone_launch)

    return ld