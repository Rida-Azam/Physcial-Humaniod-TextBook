import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
import numpy as np
import cv2
from cv_bridge import CvBridge


class SimpleObjectDetector(Node):
    """
    A simple object detector that processes camera images and detects colored objects.
    """

    def __init__(self):
        super().__init__('simple_object_detector')

        # Create subscriber for camera images
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Create publisher for detection results
        self.detection_pub = self.create_publisher(String, 'object_detections', 10)

        # Create CvBridge to convert ROS images to OpenCV format
        self.bridge = CvBridge()

        # Detection parameters
        self.min_area = 500  # Minimum area for a detection to be considered valid

        self.get_logger().info('Simple object detector initialized')

    def image_callback(self, msg):
        """
        Callback function to process camera images.
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {str(e)}')
            return

        # Convert BGR to HSV for color detection
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define range for red color in HSV
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        # Create masks for red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Process each contour
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_area:
                # Get bounding box for the contour
                x, y, w, h = cv2.boundingRect(contour)

                # Calculate center of the bounding box
                center_x = x + w // 2
                center_y = y + h // 2

                detection_info = f'Red object detected at ({center_x}, {center_y}) with area {area:.2f}'
                detections.append(detection_info)

                # Draw bounding box on the image
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(cv_image, 'Red Object', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Publish detection results
        if detections:
            result_msg = String()
            result_msg.data = '; '.join(detections)
            self.detection_pub.publish(result_msg)
            self.get_logger().info(result_msg.data)
        else:
            self.get_logger().info('No objects detected')

        # Optionally, you could publish the processed image here
        # For now, we'll just log the detections


def main(args=None):
    rclpy.init(args=args)
    detector = SimpleObjectDetector()

    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        detector.get_logger().info('Shutting down object detector...')
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()