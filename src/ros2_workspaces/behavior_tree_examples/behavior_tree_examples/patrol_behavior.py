import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
import time


class PatrolBehavior(Node):

    def __init__(self):
        super().__init__('patrol_behavior')
        self.publisher = self.create_publisher(String, '/patrol_status', 10)

        # Define patrol waypoints
        self.waypoints = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=2.0, y=0.0, z=0.0),
            Point(x=2.0, y=1.0, z=0.0),
            Point(x=0.0, y=1.0, z=0.0),
        ]

        self.current_waypoint_index = 0
        self.state = 'MOVING'  # MOVING, AT_WAYPOINT
        self.timer = self.create_timer(1.0, self.patrol_tick)
        self.time_at_waypoint = 0

    def patrol_tick(self):
        status_msg = String()

        if self.state == 'MOVING':
            target = self.waypoints[self.current_waypoint_index]
            status_msg.data = f'Moving to waypoint {self.current_waypoint_index}: ({target.x}, {target.y})'

            # Simulate reaching waypoint after some time
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
            self.state = 'AT_WAYPOINT'
            self.time_at_waypoint = 0

        elif self.state == 'AT_WAYPOINT':
            self.time_at_waypoint += 1
            if self.time_at_waypoint >= 3:  # Stay at waypoint for 3 ticks
                self.state = 'MOVING'
                status_msg.data = f'Finished at waypoint, moving to next'
            else:
                status_msg.data = f'At waypoint {self.current_waypoint_index}, waiting... ({self.time_at_waypoint}/3)'

        self.publisher.publish(status_msg)
        self.get_logger().info(f'Patrol: {status_msg.data}')


def main(args=None):
    rclpy.init(args=args)
    patrol_behavior = PatrolBehavior()
    rclpy.spin(patrol_behavior)
    patrol_behavior.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()