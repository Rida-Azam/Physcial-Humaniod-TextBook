import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Define a simple path of waypoints
        self.waypoints = [
            Point(x=1.0, y=0.0, z=0.0),
            Point(x=1.0, y=1.0, z=0.0),
            Point(x=0.0, y=1.0, z=0.0),
            Point(x=0.0, y=0.0, z=0.0)
        ]

        self.current_waypoint_index = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.tolerance = 0.2

        self.timer = self.create_timer(0.1, self.navigate_callback)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def navigate_callback(self):
        if self.current_waypoint_index >= len(self.waypoints):
            # Reached the end of the path
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            return

        target = self.waypoints[self.current_waypoint_index]

        # Calculate distance to current waypoint
        dx = target.x - self.current_x
        dy = target.y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)

        cmd_vel = Twist()

        if distance < self.tolerance:
            # Reached current waypoint, move to next
            self.current_waypoint_index += 1
        else:
            # Move toward current waypoint
            cmd_vel.linear.x = min(0.5, distance)
            cmd_vel.angular.z = math.atan2(dy, dx)

        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)
    waypoint_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()