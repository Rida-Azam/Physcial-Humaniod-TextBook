import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class RobotController(Node):
    """
    A simple robot controller that moves the robot avoiding obstacles.
    """

    def __init__(self):
        super().__init__('robot_controller')

        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Robot state
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')

        self.get_logger().info('Robot controller initialized')

    def scan_callback(self, msg):
        """
        Callback function to process laser scan data.
        """
        # Check for obstacles in front of the robot (within 45 degrees)
        min_index = len(msg.ranges) // 2 - 10  # Approx 45 degrees to the left
        max_index = len(msg.ranges) // 2 + 10  # Approx 45 degrees to the right

        # Find the minimum distance in the front sector
        front_distances = msg.ranges[min_index:max_index]
        self.obstacle_distance = min(front_distances) if front_distances else float('inf')

        # Check if obstacle is within threshold (e.g., 1 meter)
        self.obstacle_detected = self.obstacle_distance < 1.0

        self.get_logger().info(f'Obstacle distance: {self.obstacle_distance:.2f}m, Detected: {self.obstacle_detected}')

    def control_loop(self):
        """
        Main control loop to send velocity commands.
        """
        cmd_vel = Twist()

        if self.obstacle_detected:
            # Stop and turn if obstacle is detected
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Turn right
        else:
            # Move forward if no obstacle
            cmd_vel.linear.x = 0.5
            cmd_vel.angular.z = 0.0

        # Publish the command
        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down robot controller...')
    finally:
        # Stop the robot before shutting down
        cmd_vel = Twist()
        controller.cmd_vel_pub.publish(cmd_vel)
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()