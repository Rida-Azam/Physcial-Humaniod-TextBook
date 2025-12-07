import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class SimpleNavigation(Node):

    def __init__(self):
        super().__init__('simple_navigation')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.target_x = 1.0
        self.target_y = 1.0
        self.current_x = 0.0
        self.current_y = 0.0

        self.timer = self.create_timer(0.1, self.navigate_callback)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def navigate_callback(self):
        cmd_vel = Twist()

        # Calculate distance to target
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)

        # Simple proportional controller
        if distance > 0.1:  # tolerance
            cmd_vel.linear.x = min(0.5, distance)  # Move forward
            cmd_vel.angular.z = math.atan2(dy, dx)  # Turn toward target
        else:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    simple_navigation = SimpleNavigation()
    rclpy.spin(simple_navigation)
    simple_navigation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()