import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math


class SimpleNavigator(Node):
    """
    A simple navigator that moves the robot to a goal position.
    """

    def __init__(self):
        super().__init__('simple_navigator')

        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # Timer for navigation loop
        self.timer = self.create_timer(0.1, self.navigation_loop)

        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.goal_x = 5.0  # Goal position x
        self.goal_y = 5.0  # Goal position y
        self.reached_goal = False

        # Control parameters
        self.linear_speed = 0.5
        self.angular_speed = 0.5
        self.distance_tolerance = 0.2  # Tolerance for reaching goal

        self.get_logger().info(f'Navigator initialized. Goal: ({self.goal_x}, {self.goal_y})')

    def odom_callback(self, msg):
        """
        Callback function to process odometry data.
        """
        # Extract position and orientation from odometry message
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to euler for theta
        quat = msg.pose.pose.orientation
        self.current_theta = math.atan2(
            2 * (quat.w * quat.z + quat.x * quat.y),
            1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        )

    def navigation_loop(self):
        """
        Main navigation loop to move robot to goal.
        """
        if self.reached_goal:
            # Stop the robot when goal is reached
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            return

        # Calculate distance to goal
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance_to_goal = math.sqrt(dx * dx + dy * dy)

        # Check if goal is reached
        if distance_to_goal < self.distance_tolerance:
            self.get_logger().info('Goal reached!')
            self.reached_goal = True
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            return

        # Calculate angle to goal
        angle_to_goal = math.atan2(dy, dx)

        # Calculate angle difference
        angle_diff = angle_to_goal - self.current_theta

        # Normalize angle difference to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        cmd_vel = Twist()

        # If angle difference is large, rotate first
        if abs(angle_diff) > 0.1:  # 0.1 rad = ~5.7 degrees
            cmd_vel.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
        else:
            # Move forward towards goal
            cmd_vel.linear.x = min(self.linear_speed, distance_to_goal)
            cmd_vel.angular.z = self.angular_speed * angle_diff  # Small correction

        # Publish the command
        self.cmd_vel_pub.publish(cmd_vel)

        self.get_logger().info(f'Pos: ({self.current_x:.2f}, {self.current_y:.2f}), Goal: ({self.goal_x}, {self.goal_y}), Dist: {distance_to_goal:.2f}')


def main(args=None):
    rclpy.init(args=args)
    navigator = SimpleNavigator()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Shutting down navigator...')
    finally:
        # Stop the robot before shutting down
        cmd_vel = Twist()
        navigator.cmd_vel_pub.publish(cmd_vel)
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()