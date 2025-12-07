import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import math


class SimpleArmController(Node):
    """
    A simple arm controller that sends joint trajectory commands.
    """

    def __init__(self):
        super().__init__('simple_arm_controller')

        # Create publisher for joint trajectory commands
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            'joint_trajectory',
            10
        )

        # Timer for sending trajectory commands
        self.timer = self.create_timer(2.0, self.send_trajectory_command)

        # Joint names for a simple 3-DOF arm
        self.joint_names = ['shoulder_joint', 'elbow_joint', 'wrist_joint']

        self.get_logger().info('Simple arm controller initialized')

    def send_trajectory_command(self):
        """
        Send a simple trajectory command to move the arm.
        """
        # Create joint trajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # Create trajectory point
        point = JointTrajectoryPoint()

        # Set positions (in radians) - moving to a simple pose
        t = self.get_clock().now().nanoseconds / 1e9  # Time in seconds
        point.positions = [
            math.sin(t * 0.5) * 0.5,      # Shoulder
            math.sin(t * 0.3) * 0.3,      # Elbow
            math.sin(t * 0.7) * 0.2       # Wrist
        ]

        # Set velocities
        point.velocities = [
            math.cos(t * 0.5) * 0.5 * 0.5,    # Shoulder velocity
            math.cos(t * 0.3) * 0.3 * 0.3,    # Elbow velocity
            math.cos(t * 0.7) * 0.2 * 0.7     # Wrist velocity
        ]

        # Set accelerations
        point.accelerations = [
            -math.sin(t * 0.5) * 0.5 * 0.5 * 0.5,  # Shoulder acceleration
            -math.sin(t * 0.3) * 0.3 * 0.3 * 0.3,  # Elbow acceleration
            -math.sin(t * 0.7) * 0.2 * 0.7 * 0.7   # Wrist acceleration
        ]

        # Set time from start (2 seconds to reach the position)
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0

        # Add the point to the trajectory
        trajectory_msg.points = [point]

        # Set header
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.header.frame_id = 'base_link'

        # Publish the trajectory command
        self.joint_trajectory_pub.publish(trajectory_msg)

        self.get_logger().info(f'Sent trajectory command: {point.positions}')


def main(args=None):
    rclpy.init(args=args)
    controller = SimpleArmController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down arm controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()