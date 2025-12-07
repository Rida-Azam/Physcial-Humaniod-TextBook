from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # VSLAM node
    vslam_node = Node(
        package='perception_pipeline',
        executable='vslam_node',
        name='vslam_node',
        namespace=namespace,
        parameters=[params_file],
        remappings=[
            ('/camera/rgb/image_raw', '/camera/rgb/image_raw'),
            ('/camera/rgb/camera_info', '/camera/rgb/camera_info'),
        ],
        output='screen'
    )

    # Object detection node
    object_detection_node = Node(
        package='perception_pipeline',
        executable='object_detection_node',
        name='object_detection_node',
        namespace=namespace,
        parameters=[params_file],
        remappings=[
            ('/camera/rgb/image_raw', '/camera/rgb/image_raw'),
            ('/camera/rgb/camera_info', '/camera/rgb/camera_info'),
        ],
        output='screen'
    )

    # Perception fusion node
    perception_fusion_node = Node(
        package='perception_pipeline',
        executable='perception_fusion_node',
        name='perception_fusion_node',
        namespace=namespace,
        parameters=[params_file],
        output='screen'
    )

    # RViz2 for visualization (optional)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('perception_pipeline'),
        'rviz',
        'perception_pipeline.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=LaunchConfiguration('enable_rviz')
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace all nodes will be placed under'))

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))

    ld.add_action(DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('perception_pipeline'),
            'config',
            'perception_pipeline_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for all launched nodes'))

    ld.add_action(DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'))

    # Add nodes
    ld.add_action(vslam_node)
    ld.add_action(object_detection_node)
    ld.add_action(perception_fusion_node)
    ld.add_action(rviz_node)

    return ld