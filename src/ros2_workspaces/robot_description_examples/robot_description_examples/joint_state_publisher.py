import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class JointStatePublisher(Node):
    """
    A simple joint state publisher that publishes joint positions, velocities, and efforts.
    """

    def __init__(self):
        super().__init__('joint_state_publisher')

        # Create publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Timer for publishing joint states
        self.timer = self.create_timer(0.02, self.publish_joint_states)  # 50 Hz

        # Joint names and initial values
        self.joint_names = ['hip_joint', 'knee_joint', 'ankle_joint',
                           'shoulder_joint', 'elbow_joint', 'wrist_joint']
        self.joint_positions = [0.0] * len(self.joint_names)
        self.joint_velocities = [0.0] * len(self.joint_names)
        self.joint_efforts = [0.0] * len(self.joint_names)

        self.get_logger().info('Joint state publisher initialized')

    def publish_joint_states(self):
        """
        Publish joint states.
        """
        # Create joint state message
        msg = JointState()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = self.joint_velocities
        msg.effort = self.joint_efforts
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Update joint positions with time-varying values
        t = self.get_clock().now().nanoseconds / 1e9  # Time in seconds

        # Create different oscillation patterns for different joints
        for i, _ in enumerate(self.joint_names):
            # Use different frequencies and phases for each joint
            self.joint_positions[i] = math.sin(t * (i + 1) * 0.5) * (0.5 - i * 0.05)
            self.joint_velocities[i] = math.cos(t * (i + 1) * 0.5) * (i + 1) * 0.5 * (0.5 - i * 0.05)

        # Publish joint states
        self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    publisher = JointStatePublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info('Shutting down joint state publisher...')
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()