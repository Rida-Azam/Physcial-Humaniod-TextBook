#!/usr/bin/env python3

"""
Main entry point for Isaac Sim Integration Package

This module provides the main interface for the Isaac Sim integration system
with physics, rendering, and sensor simulation for humanoid robotics.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import JointState, Image, CameraInfo, Imu
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry

import sys
import os
import time
from datetime import datetime
import numpy as np

# Import Isaac Sim integration components
from isaac_sim_integration.isaac_sim_integration import IsaacSimIntegratedSystem


class IsaacSimIntegrationNode(Node):
    """
    ROS 2 node for Isaac Sim integration with physics, rendering, and sensor simulation
    """

    def __init__(self):
        super().__init__('isaac_sim_integration_node')

        # Initialize Isaac Sim integrated system
        self.get_logger().info('Initializing Isaac Sim integration system...')

        # Use headless mode if requested (for training)
        use_headless = self.declare_parameter('use_headless', False).get_parameter_value().bool_value
        physics_dt = self.declare_parameter('physics_dt', 0.001).get_parameter_value().double_value
        rendering_dt = self.declare_parameter('rendering_dt', 0.016667).get_parameter_value().double_value

        self.isaac_sim_system = IsaacSimIntegratedSystem(
            headless=use_headless,
            physics_dt=physics_dt,
            rendering_dt=rendering_dt
        )

        self.get_logger().info('Isaac Sim integration system initialized')

        # Publishers for simulation data
        qos_profile = QoSProfile(depth=10)

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', qos_profile)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile)
        self.sim_status_pub = self.create_publisher(String, '/sim_status', qos_profile)

        # Subscribers for commands
        self.joint_cmd_sub = self.create_subscription(
            Float32, '/joint_commands', self.joint_cmd_callback, qos_profile
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos_profile
        )

        # Timer for publishing simulation data
        self.pub_timer = self.create_timer(0.05, self.publish_simulation_data)  # 20 Hz

        # Timer for simulation stepping
        self.sim_timer = self.create_timer(physics_dt, self.step_simulation)

        self.get_logger().info('Isaac Sim Integration node initialized')

    def joint_cmd_callback(self, msg):
        """
        Handle joint command messages
        """
        self.get_logger().debug(f'Received joint command: {msg.data}')

        # Process joint command and send to Isaac Sim
        # In a real implementation, this would process the command and send to the robot
        pass

    def cmd_vel_callback(self, msg):
        """
        Handle velocity command messages
        """
        self.get_logger().debug(f'Received velocity command: linear=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), angular=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})')

        # Process velocity command and send to Isaac Sim
        # In a real implementation, this would process the command and send to the robot
        pass

    def publish_simulation_data(self):
        """
        Publish simulation data to ROS topics
        """
        try:
            # Get robot state from Isaac Sim
            robot_state = self.isaac_sim_system.get_robot_state()

            if robot_state:
                # Publish joint states
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.header.frame_id = 'base_link'

                # In a real implementation, we would have actual joint names
                joint_state_msg.name = [f'joint_{i}' for i in range(len(robot_state.get('joint_positions', [])))]
                joint_state_msg.position = robot_state.get('joint_positions', [])
                joint_state_msg.velocity = robot_state.get('joint_velocities', [])
                joint_state_msg.effort = robot_state.get('joint_efforts', [])

                self.joint_state_pub.publish(joint_state_msg)

                # Publish IMU data
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = 'imu_link'

                # Set dummy IMU data (in real implementation, this would come from simulated IMU)
                imu_msg.linear_acceleration.x = 0.0
                imu_msg.linear_acceleration.y = 0.0
                imu_msg.linear_acceleration.z = -9.81  # Gravity
                imu_msg.angular_velocity.x = 0.0
                imu_msg.angular_velocity.y = 0.0
                imu_msg.angular_velocity.z = 0.0
                imu_msg.orientation.w = 1.0  # No rotation initially

                self.imu_pub.publish(imu_msg)

                # Publish odometry (if available)
                if 'base_position' in robot_state and 'base_orientation' in robot_state:
                    odom_msg = Odometry()
                    odom_msg.header.stamp = self.get_clock().now().to_msg()
                    odom_msg.header.frame_id = 'odom'
                    odom_msg.child_frame_id = 'base_link'

                    pos = robot_state['base_position']
                    quat = robot_state['base_orientation']

                    odom_msg.pose.pose.position.x = pos[0]
                    odom_msg.pose.pose.position.y = pos[1]
                    odom_msg.pose.pose.position.z = pos[2]

                    odom_msg.pose.pose.orientation.x = quat[0]
                    odom_msg.pose.pose.orientation.y = quat[1]
                    odom_msg.pose.pose.orientation.z = quat[2]
                    odom_msg.pose.pose.orientation.w = quat[3]

                    # Set velocities if available
                    if 'base_linear_velocity' in robot_state:
                        lin_vel = robot_state['base_linear_velocity']
                        odom_msg.twist.twist.linear.x = lin_vel[0]
                        odom_msg.twist.twist.linear.y = lin_vel[1]
                        odom_msg.twist.twist.linear.z = lin_vel[2]

                    if 'base_angular_velocity' in robot_state:
                        ang_vel = robot_state['base_angular_velocity']
                        odom_msg.twist.twist.angular.x = ang_vel[0]
                        odom_msg.twist.twist.angular.y = ang_vel[1]
                        odom_msg.twist.twist.angular.z = ang_vel[2]

                    self.odom_pub.publish(odom_msg)

            # Publish simulation status
            status_msg = String()
            status_msg.data = f"Running - Sim Time: {self.isaac_sim_system.simulation_time:.2f}s"
            self.sim_status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing simulation data: {e}')

    def step_simulation(self):
        """
        Step the Isaac Sim physics simulation
        """
        try:
            # In a real implementation, this would step the Isaac Sim world
            # For this demonstration, we'll just update the simulation time
            self.isaac_sim_system.simulation_time += self.isaac_sim_system.physics_dt

            # In actual implementation:
            # self.isaac_sim_system.world.step(render=True)

        except Exception as e:
            self.get_logger().error(f'Error stepping simulation: {e}')

    def destroy_node(self):
        """
        Cleanup when node is destroyed
        """
        self.get_logger().info('Shutting down Isaac Sim integration node...')

        # Close Isaac Sim application
        if hasattr(self, 'isaac_sim_system'):
            self.isaac_sim_system.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    integration_node = IsaacSimIntegrationNode()

    try:
        rclpy.spin(integration_node)
    except KeyboardInterrupt:
        integration_node.get_logger().info('Interrupted by user')
    finally:
        integration_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()