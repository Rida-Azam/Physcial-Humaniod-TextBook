import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header


class ArmController(Node):

    def __init__(self):
        super().__init__('arm_controller')
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Timer to send joint commands periodically
        self.timer = self.create_timer(2.0, self.send_joint_trajectory)
        self.waypoint_index = 0

        # Define some simple waypoints for the arm
        self.waypoints = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Home position
            [0.5, 0.3, -0.2, 0.1, 0.0, 0.0],  # Position 1
            [-0.5, 0.3, -0.2, 0.1, 0.0, 0.0], # Position 2
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # Back to home
        ]

    def send_joint_trajectory(self):
        if self.waypoint_index >= len(self.waypoints):
            self.waypoint_index = 0  # Reset to beginning

        waypoint = self.waypoints[self.waypoint_index]

        # Create joint trajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.header = Header()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.header.frame_id = 'base_link'

        # Define joint names (for a generic manipulator)
        trajectory_msg.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = waypoint
        point.velocities = [0.0] * len(waypoint)  # Start with zero velocity
        point.accelerations = [0.0] * len(waypoint)  # Start with zero acceleration
        point.time_from_start.sec = 1  # Move to position in 1 second
        point.time_from_start.nanosec = 0

        trajectory_msg.points = [point]

        self.publisher.publish(trajectory_msg)
        self.get_logger().info(f'Published joint trajectory to waypoint {self.waypoint_index}: {waypoint}')

        self.waypoint_index += 1


def main(args=None):
    rclpy.init(args=args)
    arm_controller = ArmController()
    rclpy.spin(arm_controller)
    arm_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()