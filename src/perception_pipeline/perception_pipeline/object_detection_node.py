#!/usr/bin/env python3

"""
Object Detection Node for Humanoid Robots

This module implements object detection using deep learning models
to identify and locate objects in the robot's environment.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import message_filters

import cv2
import numpy as np
import torch
import torchvision
from torchvision import transforms
from PIL import Image as PILImage


class ObjectDetectionNode(Node):
    """
    A node that implements object detection for humanoid robots.
    """

    def __init__(self):
        super().__init__('object_detection_node')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Load pre-trained object detection model (YOLOv5 or similar)
        # For this example, we'll use torchvision's pre-trained model
        self.model = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
        self.model.eval()

        # COCO dataset class names (80 classes)
        self.coco_names = [
            '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
            'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
            'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag',
            'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite',
            'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
            'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana',
            'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
            'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table',
            'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
            'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock',
            'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

        # Detection parameters
        self.confidence_threshold = 0.5
        self.nms_threshold = 0.4  # Non-maximum suppression threshold

        # Camera parameters (will be updated from camera_info)
        self.fx = 554.256  # Focal length x
        self.fy = 554.256  # Focal length y
        self.cx = 320.5    # Principal point x
        self.cy = 240.5    # Principal point y

        # Publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.detection_pub = self.create_publisher(Detection2DArray, 'object_detections', 10)
        self.debug_image_pub = self.create_publisher(Image, 'object_detection/debug_image', 10)

        # Subscribers
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/rgb/image_raw', qos_profile=qos_profile)
        self.camera_info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/rgb/camera_info', qos_profile=qos_profile)

        # Approximate time synchronizer for image and camera info
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.camera_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.image_callback)

        self.get_logger().info('Object Detection Node initialized')

    def image_callback(self, image_msg, camera_info_msg):
        """
        Callback for synchronized image and camera info
        """
        try:
            # Update camera parameters if they've changed
            if (abs(self.fx - camera_info_msg.k[0]) > 1e-6 or
                abs(self.fy - camera_info_msg.k[4]) > 1e-6):
                self.fx = camera_info_msg.k[0]
                self.fy = camera_info_msg.k[4]
                self.cx = camera_info_msg.k[2]
                self.cy = camera_info_msg.k[5]

            # Convert ROS image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")

            # Perform object detection
            detections = self.detect_objects(cv_image)

            # Create and publish detection message
            detection_array_msg = self.create_detection_message(detections, image_msg.header)
            self.detection_pub.publish(detection_array_msg)

            # Create and publish debug image with bounding boxes
            debug_image = self.draw_detections(cv_image, detections)
            debug_image_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_image_msg.header = image_msg.header
            self.debug_image_pub.publish(debug_image_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_objects(self, image):
        """
        Perform object detection on the input image
        """
        # Convert OpenCV image to PIL
        pil_image = PILImage.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

        # Preprocess image for the model
        transform = transforms.Compose([
            transforms.ToTensor(),
        ])
        input_tensor = transform(pil_image)
        input_batch = input_tensor.unsqueeze(0)  # Add batch dimension

        # Perform inference
        with torch.no_grad():
            outputs = self.model(input_batch)

        # Process outputs
        detections = []
        for i, (boxes, scores, labels) in enumerate(zip(outputs[0]['boxes'],
                                                        outputs[0]['scores'],
                                                        outputs[0]['labels'])):
            # Filter detections by confidence
            keep_indices = scores > self.confidence_threshold

            filtered_boxes = boxes[keep_indices]
            filtered_scores = scores[keep_indices]
            filtered_labels = labels[keep_indices]

            for box, score, label in zip(filtered_boxes, filtered_scores, filtered_labels):
                x1, y1, x2, y2 = box.tolist()
                class_name = self.coco_names[label]

                detection = {
                    'bbox': [x1, y1, x2, y2],
                    'score': score.item(),
                    'class_id': label.item(),
                    'class_name': class_name
                }
                detections.append(detection)

        return detections

    def create_detection_message(self, detections, header):
        """
        Create a Detection2DArray message from detection results
        """
        detection_array_msg = Detection2DArray()
        detection_array_msg.header = header

        for detection in detections:
            detection_msg = Detection2D()

            # Set ID and hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = str(detection['class_id'])
            hypothesis.score = detection['score']
            detection_msg.results.append(hypothesis)

            # Set bounding box
            x1, y1, x2, y2 = detection['bbox']
            bbox_x = int(x1)
            bbox_y = int(y1)
            bbox_w = int(x2 - x1)
            bbox_h = int(y2 - y1)

            detection_msg.bbox.center.x = bbox_x + bbox_w / 2.0
            detection_msg.bbox.center.y = bbox_y + bbox_h / 2.0
            detection_msg.bbox.size_x = bbox_w
            detection_msg.bbox.size_y = bbox_h

            # Add centroid as a point
            centroid = Point()
            centroid.x = detection_msg.bbox.center.x
            centroid.y = detection_msg.bbox.center.y
            centroid.z = 0.0  # Will be computed from depth in a full implementation
            detection_msg.bbox.center = centroid

            detection_array_msg.detections.append(detection_msg)

        return detection_array_msg

    def draw_detections(self, image, detections):
        """
        Draw bounding boxes and labels on the image
        """
        output_image = image.copy()

        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            score = detection['score']
            class_name = detection['class_name']

            # Draw bounding box
            cv2.rectangle(output_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

            # Draw label and confidence
            label = f"{class_name}: {score:.2f}"
            cv2.putText(output_image, label, (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return output_image


def main(args=None):
    rclpy.init(args=args)

    object_detection_node = ObjectDetectionNode()

    try:
        rclpy.spin(object_detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        object_detection_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()