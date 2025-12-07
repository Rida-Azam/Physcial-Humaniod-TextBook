import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoystickTeleop(Node):

    def __init__(self):
        super().__init__('joystick_teleop')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parameters for mapping joystick to robot movement
        self.linear_axis = 1  # Left stick vertical (forward/backward)
        self.angular_axis = 0  # Left stick horizontal (left/right)
        self.linear_scale = 1.0
        self.angular_scale = 1.0

        self.get_logger().info('Joystick Teleop Node Initialized')

    def joy_callback(self, msg):
        twist = Twist()

        # Map joystick axes to linear and angular velocities
        if len(msg.axes) > max(self.linear_axis, self.angular_axis):
            twist.linear.x = msg.axes[self.linear_axis] * self.linear_scale
            twist.angular.z = msg.axes[self.angular_axis] * self.angular_scale

        # Publish the command
        self.publisher.publish(twist)
        self.get_logger().info(f'Joystick command: linear.x={twist.linear.x}, angular.z={twist.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    joystick_teleop = JoystickTeleop()
    rclpy.spin(joystick_teleop)
    joystick_teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()