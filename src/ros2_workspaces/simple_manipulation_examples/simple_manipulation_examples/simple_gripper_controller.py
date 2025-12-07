import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math


class SimpleGripperController(Node):
    """
    A simple gripper controller that opens and closes the gripper.
    """

    def __init__(self):
        super().__init__('simple_gripper_controller')

        # Create publisher for gripper commands
        self.gripper_command_pub = self.create_publisher(
            Float64MultiArray,
            'gripper_command',
            10
        )

        # Create subscriber for joint states (to monitor gripper position)
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for sending gripper commands
        self.timer = self.create_timer(3.0, self.send_gripper_command)

        # Gripper state
        self.gripper_position = 0.0  # 0.0 = closed, 1.0 = open
        self.opening = True  # True if gripper is opening, False if closing

        self.get_logger().info('Simple gripper controller initialized')

    def joint_state_callback(self, msg):
        """
        Callback function to receive joint states.
        """
        # Check if gripper joints are in the message
        for i, name in enumerate(msg.name):
            if 'gripper' in name.lower():
                self.gripper_position = msg.position[i]
                break

    def send_gripper_command(self):
        """
        Send a command to open or close the gripper.
        """
        command_msg = Float64MultiArray()

        # Alternate between opening and closing
        if self.opening:
            # Send command to open gripper
            command_msg.data = [0.8, 0.8]  # Open both fingers
            self.get_logger().info('Sending gripper open command')
        else:
            # Send command to close gripper
            command_msg.data = [0.1, 0.1]  # Close both fingers (not completely to avoid collisions)
            self.get_logger().info('Sending gripper close command')

        # Toggle the opening state for next time
        self.opening = not self.opening

        # Publish the command
        self.gripper_command_pub.publish(command_msg)

        self.get_logger().info(f'Gripper command sent: {command_msg.data}')


def main(args=None):
    rclpy.init(args=args)
    controller = SimpleGripperController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down gripper controller...')
    finally:
        # Send a command to close the gripper before shutting down
        command_msg = Float64MultiArray()
        command_msg.data = [0.0, 0.0]  # Close gripper
        controller.gripper_command_pub.publish(command_msg)

        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()