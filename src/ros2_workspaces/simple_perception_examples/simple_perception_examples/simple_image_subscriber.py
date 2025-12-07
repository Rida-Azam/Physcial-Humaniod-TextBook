import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np
import cv2
from cv_bridge import CvBridge


class SimpleImageSubscriber(Node):
    """
    A simple image subscriber that receives camera images and performs basic processing.
    """

    def __init__(self):
        super().__init__('simple_image_subscriber')

        # Create subscriber for camera images
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Create publisher for processed images
        self.processed_image_pub = self.create_publisher(Image, 'camera/processed_image', 10)

        # Create CvBridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        self.get_logger().info('Simple image subscriber initialized')

    def image_callback(self, msg):
        """
        Callback function to process incoming camera images.
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {str(e)}')
            return

        # Perform some basic image processing (e.g., edge detection)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Convert edges back to BGR for display purposes
        edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        # Combine original and edge-detected images (side by side)
        combined = np.hstack((cv_image, edges_bgr))

        try:
            # Convert back to ROS Image message
            processed_msg = self.bridge.cv2_to_imgmsg(combined, encoding='bgr8')

            # Update header information
            processed_msg.header = msg.header

            # Publish the processed image
            self.processed_image_pub.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f'Could not convert processed image: {str(e)}')

        self.get_logger().info('Processed image published')


def main(args=None):
    rclpy.init(args=args)
    subscriber = SimpleImageSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        subscriber.get_logger().info('Shutting down image subscriber...')
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()