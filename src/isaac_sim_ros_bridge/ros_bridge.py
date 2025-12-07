#!/usr/bin/env python3

"""
Isaac Sim ROS 2 Bridge for Humanoid Robotics

This module provides the interface between Isaac Sim and ROS 2,
enabling bidirectional communication for humanoid robot simulation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import JointState, Image, CameraInfo, Imu
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time

import numpy as np
import time
from threading import Thread
import queue


class IsaacSimROSBridge(Node):
    """
    Bridge node connecting Isaac Sim and ROS 2 for humanoid robotics
    """

    def __init__(self):
        super().__init__('isaac_sim_ros_bridge')

        # Initialize Isaac Sim interface (using dummy implementation for this example)
        self.isaac_sim_connected = False
        self.simulation_running = False
        self.robot_state_queue = queue.Queue()

        # ROS publishers for simulation data
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', qos_profile)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile)
        self.camera_image_pub = self.create_publisher(Image, '/camera/rgb/image_raw', qos_profile)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/rgb/camera_info', qos_profile)

        # ROS subscribers for robot commands
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, qos_profile)
        self.joint_cmd_sub = self.create_subscription(Float32, '/joint_commands', self.joint_cmd_callback, qos_profile)

        # Status publisher
        self.status_pub = self.create_publisher(String, '/sim_status', 10)

        # Timer for periodic publishing
        self.pub_timer = self.create_timer(0.05, self.publish_simulation_data)  # 20 Hz

        # Initialize connection to Isaac Sim
        self.connect_to_isaac_sim()

        self.get_logger().info('Isaac Sim ROS Bridge initialized')

    def connect_to_isaac_sim(self):
        """
        Connect to Isaac Sim environment
        """
        try:
            # In a real implementation, this would connect to Isaac Sim
            # For this example, we'll simulate the connection
            self.get_logger().info('Connecting to Isaac Sim environment...')

            # Simulate connection process
            time.sleep(1.0)

            self.isaac_sim_connected = True
            self.simulation_running = True

            self.get_logger().info('Connected to Isaac Sim successfully')

            # Publish connection status
            status_msg = String()
            status_msg.data = 'CONNECTED'
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to connect to Isaac Sim: {e}')
            self.isaac_sim_connected = False

    def cmd_vel_callback(self, msg):
        """
        Handle velocity commands from ROS
        """
        if not self.isaac_sim_connected:
            return

        self.get_logger().debug(f'Received velocity command: linear={msg.linear}, angular={msg.angular}')

        # In a real implementation, this would send the command to Isaac Sim
        # For simulation, we'll just log the command
        command = {
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'linear_z': msg.linear.z,
            'angular_x': msg.angular.x,
            'angular_y': msg.angular.y,
            'angular_z': msg.angular.z
        }

        # Add command to queue for Isaac Sim processing
        self.robot_state_queue.put(('cmd_vel', command))

    def joint_cmd_callback(self, msg):
        """
        Handle joint position commands from ROS
        """
        if not self.isaac_sim_connected:
            return

        self.get_logger().debug(f'Received joint command: {msg.data}')

        # In a real implementation, this would send joint commands to Isaac Sim
        command = {
            'joint_positions': [msg.data]  # In real implementation, this would be an array
        }

        # Add command to queue for Isaac Sim processing
        self.robot_state_queue.put(('joint_cmd', command))

    def publish_simulation_data(self):
        """
        Publish simulation data to ROS topics
        """
        if not self.isaac_sim_connected or not self.simulation_running:
            return

        try:
            # Simulate getting data from Isaac Sim
            sim_data = self.get_simulation_data()

            # Publish joint states
            self.publish_joint_states(sim_data)

            # Publish odometry
            self.publish_odometry(sim_data)

            # Publish IMU data
            self.publish_imu_data(sim_data)

            # Publish camera data
            self.publish_camera_data(sim_data)

        except Exception as e:
            self.get_logger().error(f'Error publishing simulation data: {e}')

    def get_simulation_data(self):
        """
        Get simulation data from Isaac Sim
        """
        # In a real implementation, this would interface with Isaac Sim directly
        # For this example, we'll generate simulated data

        current_time = self.get_clock().now()

        sim_data = {
            'timestamp': current_time,
            'joint_positions': np.random.uniform(-1.5, 1.5, size=28).tolist(),  # 28 DoF humanoid
            'joint_velocities': np.random.uniform(-2.0, 2.0, size=28).tolist(),
            'joint_efforts': np.random.uniform(-50.0, 50.0, size=28).tolist(),
            'robot_pose': {
                'position': [np.random.uniform(-5.0, 5.0), np.random.uniform(-5.0, 5.0), 0.0],
                'orientation': [0.0, 0.0, np.random.uniform(-0.1, 0.1), 1.0]  # quaternion
            },
            'robot_twist': {
                'linear': [np.random.uniform(-0.5, 0.5), np.random.uniform(-0.5, 0.5), 0.0],
                'angular': [0.0, 0.0, np.random.uniform(-0.5, 0.5)]
            },
            'imu_data': {
                'linear_acceleration': [np.random.uniform(-0.1, 0.1), np.random.uniform(-0.1, 0.1), -9.81],
                'angular_velocity': [np.random.uniform(-0.01, 0.01), np.random.uniform(-0.01, 0.01), np.random.uniform(-0.1, 0.1)],
                'orientation': [0.0, 0.0, np.random.uniform(-0.01, 0.01), 1.0]
            },
            'camera_image': np.random.randint(0, 255, size=(480, 640, 3), dtype=np.uint8),  # Simulated image
            'camera_info': {
                'width': 640,
                'height': 480,
                'distortion_model': 'plumb_bob',
                'd': [0.0, 0.0, 0.0, 0.0, 0.0],  # Distortion coefficients
                'k': [300.0, 0.0, 320.0, 0.0, 300.0, 240.0, 0.0, 0.0, 1.0],  # Intrinsic matrix (as row-major)
                'r': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],  # Rectification matrix
                'p': [300.0, 0.0, 320.0, 0.0, 0.0, 300.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]  # Projection matrix
            }
        }

        return sim_data

    def publish_joint_states(self, sim_data):
        """
        Publish joint state data to ROS
        """
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = sim_data['timestamp'].to_msg()
        joint_state_msg.header.frame_id = 'base_link'

        # Define joint names for a typical humanoid (28 DoF)
        joint_names = [
            # Left leg
            'left_hip_roll', 'left_hip_yaw', 'left_hip_pitch',
            'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
            # Right leg
            'right_hip_roll', 'right_hip_yaw', 'right_hip_pitch',
            'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
            # Torso
            'torso_pitch', 'torso_roll', 'torso_yaw',
            # Left arm
            'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
            'left_elbow_pitch', 'left_forearm_yaw', 'left_wrist_pitch', 'left_wrist_yaw',
            # Right arm
            'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
            'right_elbow_pitch', 'right_forearm_yaw', 'right_wrist_pitch', 'right_wrist_yaw'
        ]

        joint_state_msg.name = joint_names
        joint_state_msg.position = sim_data['joint_positions']
        joint_state_msg.velocity = sim_data['joint_velocities']
        joint_state_msg.effort = sim_data['joint_efforts']

        self.joint_state_pub.publish(joint_state_msg)

    def publish_odometry(self, sim_data):
        """
        Publish odometry data to ROS
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = sim_data['timestamp'].to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set pose
        odom_msg.pose.pose.position = Point(
            x=sim_data['robot_pose']['position'][0],
            y=sim_data['robot_pose']['position'][1],
            z=sim_data['robot_pose']['position'][2]
        )
        odom_msg.pose.pose.orientation = Quaternion(
            x=sim_data['robot_pose']['orientation'][0],
            y=sim_data['robot_pose']['orientation'][1],
            z=sim_data['robot_pose']['orientation'][2],
            w=sim_data['robot_pose']['orientation'][3]
        )

        # Set twist
        odom_msg.twist.twist.linear = Point(
            x=sim_data['robot_twist']['linear'][0],
            y=sim_data['robot_twist']['linear'][1],
            z=sim_data['robot_twist']['linear'][2]
        )
        odom_msg.twist.twist.angular = Point(
            x=sim_data['robot_twist']['angular'][0],
            y=sim_data['robot_twist']['angular'][1],
            z=sim_data['robot_twist']['angular'][2]
        )

        self.odom_pub.publish(odom_msg)

    def publish_imu_data(self, sim_data):
        """
        Publish IMU data to ROS
        """
        imu_msg = Imu()
        imu_msg.header.stamp = sim_data['timestamp'].to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Set orientation (if available from simulation)
        imu_msg.orientation = Quaternion(
            x=sim_data['imu_data']['orientation'][0],
            y=sim_data['imu_data']['orientation'][1],
            z=sim_data['imu_data']['orientation'][2],
            w=sim_data['imu_data']['orientation'][3]
        )
        imu_msg.orientation_covariance = [0.0] * 9  # Set covariance to 0 for perfect simulation

        # Set angular velocity
        imu_msg.angular_velocity = Point(
            x=sim_data['imu_data']['angular_velocity'][0],
            y=sim_data['imu_data']['angular_velocity'][1],
            z=sim_data['imu_data']['angular_velocity'][2]
        )
        imu_msg.angular_velocity_covariance = [0.0] * 9

        # Set linear acceleration
        imu_msg.linear_acceleration = Point(
            x=sim_data['imu_data']['linear_acceleration'][0],
            y=sim_data['imu_data']['linear_acceleration'][1],
            z=sim_data['imu_data']['linear_acceleration'][2]
        )
        imu_msg.linear_acceleration_covariance = [0.0] * 9

        self.imu_pub.publish(imu_msg)

    def publish_camera_data(self, sim_data):
        """
        Publish camera image and camera info to ROS
        """
        # Publish camera info
        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = sim_data['timestamp'].to_msg()
        camera_info_msg.header.frame_id = 'camera_rgb_optical_frame'

        camera_info_msg.width = sim_data['camera_info']['width']
        camera_info_msg.height = sim_data['camera_info']['height']
        camera_info_msg.distortion_model = sim_data['camera_info']['distortion_model']
        camera_info_msg.d = sim_data['camera_info']['d']
        camera_info_msg.k = sim_data['camera_info']['k']
        camera_info_msg.r = sim_data['camera_info']['r']
        camera_info_msg.p = sim_data['camera_info']['p']

        self.camera_info_pub.publish(camera_info_msg)

        # For image publishing, we would normally convert the numpy array to a ROS Image message
        # In a real implementation, this would involve encoding the image data
        image_msg = Image()
        image_msg.header.stamp = sim_data['timestamp'].to_msg()
        image_msg.header.frame_id = 'camera_rgb_optical_frame'
        image_msg.height = 480
        image_msg.width = 640
        image_msg.encoding = 'rgb8'
        image_msg.is_bigendian = False
        image_msg.step = 640 * 3  # Width * bytes per pixel
        # In a real implementation, we would encode the actual image data here
        image_msg.data = []  # Placeholder for image data

        self.camera_image_pub.publish(image_msg)

    def destroy_node(self):
        """
        Cleanup when node is destroyed
        """
        self.simulation_running = False
        self.get_logger().info('Isaac Sim ROS Bridge shutting down...')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    bridge_node = IsaacSimROSBridge()

    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        bridge_node.get_logger().info('Interrupted by user')
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()