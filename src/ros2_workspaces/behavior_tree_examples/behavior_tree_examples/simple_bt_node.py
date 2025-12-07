import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class SimpleBTNode(Node):

    def __init__(self):
        super().__init__('simple_bt_node')
        self.publisher = self.create_publisher(String, '/bt_status', 10)
        self.timer = self.create_timer(1.0, self.bt_tick)

        # Simple behavior state
        self.state = 'SEARCHING'
        self.search_count = 0

    def bt_tick(self):
        status_msg = String()

        if self.state == 'SEARCHING':
            self.search_count += 1
            if self.search_count > 3:  # After 3 search cycles
                self.state = 'MOVING_TO_TARGET'
                self.search_count = 0
                status_msg.data = 'Transitioning to MOVING_TO_TARGET'
            else:
                status_msg.data = f'Searching... (cycle {self.search_count})'
        elif self.state == 'MOVING_TO_TARGET':
            # Simulate movement
            self.search_count += 1
            if self.search_count > 2:  # After 2 movement cycles
                self.state = 'GRASPING'
                self.search_count = 0
                status_msg.data = 'Transitioning to GRASPING'
            else:
                status_msg.data = f'Moving to target... (step {self.search_count})'
        elif self.state == 'GRASPING':
            # Simulate grasping
            self.search_count += 1
            if self.search_count > 1:  # After 1 grasp cycle
                self.state = 'RETURNING_HOME'
                self.search_count = 0
                status_msg.data = 'Transitioning to RETURNING_HOME'
            else:
                status_msg.data = 'Grasping object...'
        elif self.state == 'RETURNING_HOME':
            # Simulate return
            self.search_count += 1
            if self.search_count > 2:  # After 2 return cycles
                self.state = 'SEARCHING'
                self.search_count = 0
                status_msg.data = 'Transitioning to SEARCHING (completed cycle)'
            else:
                status_msg.data = f'Returning home... (step {self.search_count})'

        self.publisher.publish(status_msg)
        self.get_logger().info(f'BT State: {status_msg.data}')


def main(args=None):
    rclpy.init(args=args)
    simple_bt_node = SimpleBTNode()
    rclpy.spin(simple_bt_node)
    simple_bt_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()