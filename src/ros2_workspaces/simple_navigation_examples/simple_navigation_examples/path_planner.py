import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import OccupancyGrid
import math
import numpy as np


class SimplePathPlanner(Node):
    """
    A simple path planner that generates a path from start to goal.
    This is a very basic implementation without obstacle avoidance.
    """

    def __init__(self):
        super().__init__('simple_path_planner')

        # Create publisher for path
        self.path_pub = self.create_publisher(PoseArray, 'path', 10)

        # Create subscriber for map (for future enhancement with real obstacles)
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )

        # Timer for path planning
        self.timer = self.create_timer(1.0, self.plan_path)

        # Start and goal positions
        self.start_x = 0.0
        self.start_y = 0.0
        self.goal_x = 10.0
        self.goal_y = 10.0

        # Path parameters
        self.path_resolution = 0.5  # Distance between path points
        self.path = []

        self.get_logger().info('Simple path planner initialized')

    def map_callback(self, msg):
        """
        Callback function to process map data.
        """
        self.get_logger().info('Map received')

    def plan_path(self):
        """
        Plan a simple straight-line path from start to goal.
        """
        # Calculate the straight-line path
        dx = self.goal_x - self.start_x
        dy = self.goal_y - self.start_y
        distance = math.sqrt(dx * dx + dy * dy)

        # Calculate number of points based on resolution
        num_points = int(distance / self.path_resolution) + 1

        # Generate path points
        self.path = []
        for i in range(num_points):
            t = i / max(1, num_points - 1)  # Normalize to [0, 1]
            x = self.start_x + t * dx
            y = self.start_y + t * dy

            # Create pose for this point
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            # Set orientation to point towards next point
            if i < num_points - 1:
                next_dx = self.goal_x - x
                next_dy = self.goal_y - y
                yaw = math.atan2(next_dy, next_dx)
                # Convert yaw to quaternion
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                # For the last point, keep the same orientation as the previous
                if self.path:
                    pose.pose.orientation = self.path[-1].pose.orientation

            self.path.append(pose)

        # Publish the path
        self.publish_path()

        self.get_logger().info(f'Path planned with {len(self.path)} points')

    def publish_path(self):
        """
        Publish the planned path.
        """
        path_msg = PoseArray()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        path_msg.poses = [pose.pose for pose in self.path]

        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    planner = SimplePathPlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info('Shutting down path planner...')
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()