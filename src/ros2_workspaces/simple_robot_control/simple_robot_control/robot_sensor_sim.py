import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import random


class RobotSensorSimulator(Node):
    """
    A simple laser scan simulator for the robot.
    """

    def __init__(self):
        super().__init__('robot_sensor_simulator')

        # Create publisher for laser scan data
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # Create subscriber for velocity commands to track robot movement
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer for publishing simulated laser scan
        self.timer = self.create_timer(0.1, self.publish_scan)

        # Robot state for simulation
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation = 0.0  # in radians

        self.get_logger().info('Robot sensor simulator initialized')

    def cmd_vel_callback(self, msg):
        """
        Callback to receive velocity commands and update robot state.
        """
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

        # Update position based on velocity (simplified simulation)
        dt = 0.1  # Time step from timer
        self.position_x += self.linear_velocity * dt * math.cos(self.orientation)
        self.position_y += self.linear_velocity * dt * math.sin(self.orientation)
        self.orientation += self.angular_velocity * dt

    def publish_scan(self):
        """
        Publish simulated laser scan data.
        """
        scan_msg = LaserScan()

        # Fill in the required fields for LaserScan message
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'

        # Laser scanner parameters
        scan_msg.angle_min = -math.pi / 2  # -90 degrees
        scan_msg.angle_max = math.pi / 2   # 90 degrees
        scan_msg.angle_increment = math.pi / 180  # 1 degree increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0

        # Generate simulated ranges
        num_ranges = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
        ranges = []

        # Simulate some obstacles around the robot
        for i in range(num_ranges):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment

            # Simulate distance to "wall" or "obstacle"
            # Add some randomness to make it more realistic
            distance = 3.0 + random.uniform(-0.5, 0.5)  # Base distance with noise

            # Simulate an obstacle in front of the robot
            if -0.5 < angle < 0.5:  # Front-facing sensors
                if abs(self.position_x) < 2.0 and abs(self.position_y) < 2.0:
                    # If robot is near center, add an obstacle
                    distance = 1.0 + random.uniform(-0.2, 0.2)

            ranges.append(distance)

        scan_msg.ranges = ranges
        scan_msg.intensities = [100.0] * len(ranges)  # Constant intensity

        # Publish the simulated scan
        self.scan_pub.publish(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    simulator = RobotSensorSimulator()

    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        simulator.get_logger().info('Shutting down robot sensor simulator...')
    finally:
        simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()