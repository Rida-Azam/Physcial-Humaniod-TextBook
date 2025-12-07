import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
import math


class GraspPlanner(Node):

    def __init__(self):
        super().__init__('grasp_planner')
        self.subscription = self.create_subscription(
            String,
            '/object_detections',
            self.detection_callback,
            10)
        self.grasp_publisher = self.create_publisher(Pose, '/grasp_pose', 10)

    def detection_callback(self, msg):
        # Simple grasp planner that plans a grasp for detected objects
        detection_data = msg.data
        self.get_logger().info(f'Processing detection: {detection_data}')

        # Parse detection data (simplified - in reality this would be more complex)
        if 'Red object' in detection_data:
            # Extract position from detection (simplified parsing)
            # In a real system, we would have structured detection messages
            pose = Pose()
            pose.position.x = 0.5  # Placeholder position
            pose.position.y = 0.0
            pose.position.z = 0.1  # Height above ground

            # Simple orientation (facing down to grasp)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.707  # 90 degree rotation around y-axis
            pose.orientation.z = 0.0
            pose.orientation.w = 0.707

            self.grasp_publisher.publish(pose)
            self.get_logger().info(f'Published grasp pose: ({pose.position.x}, {pose.position.y}, {pose.position.z})')


def main(args=None):
    rclpy.init(args=args)
    grasp_planner = GraspPlanner()
    rclpy.spin(grasp_planner)
    grasp_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()