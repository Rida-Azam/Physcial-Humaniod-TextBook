import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear_speed = 0.5
        self.angular_speed = 1.0

        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info('Keyboard Teleop Node Initialized')
        self.get_logger().info('Controls:')
        self.get_logger().info('w/s: Move forward/backward')
        self.get_logger().info('a/d: Turn left/right')
        self.get_logger().info('q/e: Rotate left/right in place')
        self.get_logger().info('x: Stop')
        self.get_logger().info('z: Quit')

        # Timer to check for keyboard input
        self.timer = self.create_timer(0.1, self.check_keyboard)

    def check_keyboard(self):
        key = self.get_key()
        if key:
            twist = Twist()

            if key == 'w':  # Forward
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
            elif key == 's':  # Backward
                twist.linear.x = -self.linear_speed
                twist.angular.z = 0.0
            elif key == 'a':  # Left
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
            elif key == 'd':  # Right
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_speed
            elif key == 'q':  # Rotate left in place
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
            elif key == 'e':  # Rotate right in place
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_speed
            elif key == 'x':  # Stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif key == 'z':  # Quit
                self.get_logger().info('Shutting down...')
                rclpy.shutdown()
                return
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.publisher.publish(twist)
            if key != 'x':
                self.get_logger().info(f'Published command: linear.x={twist.linear.x}, angular.z={twist.angular.z}')

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select_list = [sys.stdin.fileno()]
        available = select.select(select_list, [], [], 0.0)[0]

        if available:
            key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key
        else:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return None


def main(args=None):
    rclpy.init(args=args)
    keyboard_teleop = KeyboardTeleop()

    try:
        rclpy.spin(keyboard_teleop)
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()