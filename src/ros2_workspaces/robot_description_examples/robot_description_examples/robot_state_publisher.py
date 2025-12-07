import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import numpy as np


class RobotStatePublisher(Node):
    """
    A simple robot state publisher that publishes joint states and transforms.
    """

    def __init__(self):
        super().__init__('robot_state_publisher')

        # Create publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Create transform broadcaster for TF
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Joint names and initial positions
        self.joint_names = ['joint1', 'joint2', 'joint3']
        self.joint_positions = [0.0, 0.0, 0.0]
        self.joint_velocities = [0.0, 0.0, 0.0]
        self.joint_efforts = [0.0, 0.0, 0.0]

        self.get_logger().info('Robot state publisher initialized')

    def publish_joint_states(self):
        """
        Publish joint states and transforms.
        """
        # Create joint state message
        msg = JointState()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = self.joint_velocities
        msg.effort = self.joint_efforts
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Update joint positions with a simple oscillating pattern
        t = self.get_clock().now().nanoseconds / 1e9  # Time in seconds
        self.joint_positions[0] = math.sin(t) * 0.5
        self.joint_positions[1] = math.sin(t * 1.5) * 0.3
        self.joint_positions[2] = math.sin(t * 2.0) * 0.2

        # Publish joint states
        self.joint_state_pub.publish(msg)

        # Publish transforms for each joint
        self.publish_transforms(t)

    def publish_transforms(self, t):
        """
        Publish transforms for the robot links.
        """
        # Base link to link1
        t_base_link1 = TransformStamped()
        t_base_link1.header.stamp = self.get_clock().now().to_msg()
        t_base_link1.header.frame_id = 'base_link'
        t_base_link1.child_frame_id = 'link1'
        t_base_link1.transform.translation.x = 0.5
        t_base_link1.transform.translation.y = 0.0
        t_base_link1.transform.translation.z = 0.0
        t_base_link1.transform.rotation.x = 0.0
        t_base_link1.transform.rotation.y = 0.0
        t_base_link1.transform.rotation.z = 0.0
        t_base_link1.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_base_link1)

        # Link1 to link2 (rotating joint)
        t_link1_link2 = TransformStamped()
        t_link1_link2.header.stamp = self.get_clock().now().to_msg()
        t_link1_link2.header.frame_id = 'link1'
        t_link1_link2.child_frame_id = 'link2'
        t_link1_link2.transform.translation.x = 0.3 * math.cos(self.joint_positions[0])
        t_link1_link2.transform.translation.y = 0.3 * math.sin(self.joint_positions[0])
        t_link1_link2.transform.translation.z = 0.0
        # Rotate around z-axis based on joint position
        t_link1_link2.transform.rotation.x = 0.0
        t_link1_link2.transform.rotation.y = 0.0
        t_link1_link2.transform.rotation.z = math.sin(self.joint_positions[0] / 2)
        t_link1_link2.transform.rotation.w = math.cos(self.joint_positions[0] / 2)
        self.tf_broadcaster.sendTransform(t_link1_link2)

        # Link2 to link3 (rotating joint)
        t_link2_link3 = TransformStamped()
        t_link2_link3.header.stamp = self.get_clock().now().to_msg()
        t_link2_link3.header.frame_id = 'link2'
        t_link2_link3.child_frame_id = 'link3'
        t_link2_link3.transform.translation.x = 0.2 * math.cos(self.joint_positions[1])
        t_link2_link3.transform.translation.y = 0.2 * math.sin(self.joint_positions[1])
        t_link2_link3.transform.translation.z = 0.0
        # Rotate around z-axis based on joint position
        t_link2_link3.transform.rotation.x = 0.0
        t_link2_link3.transform.rotation.y = 0.0
        t_link2_link3.transform.rotation.z = math.sin(self.joint_positions[1] / 2)
        t_link2_link3.transform.rotation.w = math.cos(self.joint_positions[1] / 2)
        self.tf_broadcaster.sendTransform(t_link2_link3)


def main(args=None):
    rclpy.init(args=args)
    publisher = RobotStatePublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info('Shutting down robot state publisher...')
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()