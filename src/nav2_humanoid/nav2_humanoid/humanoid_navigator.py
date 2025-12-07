#!/usr/bin/env python3

"""
Humanoid Navigator Node

This node implements navigation capabilities specifically tailored for humanoid robots,
including special considerations for bipedal locomotion and balance.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import Imu
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import math


class HumanoidNavigator(Node):
    """
    A navigation node specifically designed for humanoid robots.
    It interfaces with the Nav2 stack while considering humanoid-specific constraints.
    """

    def __init__(self):
        super().__init__('humanoid_navigator')

        # Create action client for navigation
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Create TF buffer and listener for transform operations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # IMU subscriber for balance feedback
        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )

        # Current pose tracking
        self.current_pose = None
        self.is_balanced = True

        # Navigation parameters specific to humanoid robots
        self.step_size = 0.3  # Maximum step size for humanoid
        self.turn_threshold = 0.2  # Threshold for turning in place
        self.balance_threshold = 0.1  # Acceptable deviation from upright

        self.get_logger().info('Humanoid Navigator initialized')

    def imu_callback(self, msg):
        """
        Callback for IMU data to monitor robot balance
        """
        # Extract roll and pitch from quaternion
        w, x, y, z = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z

        # Calculate roll and pitch
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - z * x))

        # Check if robot is within balance threshold
        self.is_balanced = abs(roll) < self.balance_threshold and abs(pitch) < self.balance_threshold

        if not self.is_balanced:
            self.get_logger().warn(f'Humanoid is out of balance! Roll: {roll:.3f}, Pitch: {pitch:.3f}')

    def get_current_pose(self):
        """
        Get current robot pose from TF tree
        """
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                'base_link',  # Source frame
                now,
                rclpy.duration.Duration(seconds=1.0)
            )

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation

            return pose
        except TransformException as ex:
            self.get_logger().error(f'Could not transform map to base_link: {ex}')
            return None

    def navigate_to_pose(self, x, y, z, quat_x, quat_y, quat_z, quat_w):
        """
        Navigate the humanoid robot to a specified pose
        """
        # Wait for action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position = Point(x=x, y=y, z=z)
        goal_msg.pose.pose.orientation = Quaternion(x=quat_x, y=quat_y, z=quat_z, w=quat_w)

        # Send goal
        self.get_logger().info(f'Sending navigation goal to ({x}, {y})')
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Add result callback
        send_goal_future.add_done_callback(self.goal_response_callback)

        return True

    def feedback_callback(self, feedback_msg):
        """
        Handle feedback from the navigation action
        """
        feedback = feedback_msg.feedback
        # In a real implementation, we would monitor the humanoid's specific feedback
        # such as step timing, balance adjustments, etc.
        self.get_logger().info(f'Navigation feedback: {feedback.current_pose.pose.position.x:.2f}, {feedback.current_pose.pose.position.y:.2f}')

    def goal_response_callback(self, future):
        """
        Handle response when goal is accepted or rejected
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Handle the final result of the navigation action
        """
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')

        # Check if navigation was successful
        if result:
            self.get_logger().info('Successfully reached the goal pose')
        else:
            self.get_logger().info('Navigation failed')


def main(args=None):
    rclpy.init(args=args)

    navigator = HumanoidNavigator()

    # Example: Navigate to a pose (this would typically be triggered by a service call or other input)
    # For demonstration, we'll navigate to a simple pose
    success = navigator.navigate_to_pose(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    if success:
        rclpy.spin(navigator)
    else:
        navigator.get_logger().error('Failed to send navigation goal')

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()