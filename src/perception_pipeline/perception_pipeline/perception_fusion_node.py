#!/usr/bin/env python3

"""
Perception Fusion Node for Humanoid Robots

This module fuses data from VSLAM and object detection to create
a comprehensive understanding of the environment.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped, Point, Vector3
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray, Detection2D
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point

import numpy as np
from collections import defaultdict, deque
import math


class PerceptionFusionNode(Node):
    """
    A node that fuses VSLAM and object detection data to create
    a comprehensive perception of the environment.
    """

    def __init__(self):
        super().__init__('perception_fusion_node')

        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Data storage
        self.vslam_pose = None
        self.vslam_pose_timestamp = None
        self.object_detections = []
        self.object_detections_timestamp = None

        # Tracking of detected objects with history
        self.tracked_objects = defaultdict(lambda: {
            'positions': deque(maxlen=10),
            'velocities': deque(maxlen=5),
            'last_seen': self.get_clock().now(),
            'class_name': '',
            'confidence': 0.0
        })

        # Parameters
        self.max_tracking_age = 5.0  # seconds
        self.min_detection_confidence = 0.6
        self.fusion_publish_rate = 10.0  # Hz

        # Publishers
        qos_profile = QoSProfile(depth=10)

        self.fused_objects_pub = self.create_publisher(Detection2DArray, 'perception_fused/objects', 10)
        self.fused_map_pub = self.create_publisher(MarkerArray, 'perception_fused/map', 10)
        self.debug_marker_pub = self.create_publisher(MarkerArray, 'perception_fused/debug', 10)

        # Subscribers
        self.vslam_sub = self.create_subscription(
            PoseStamped,
            'vslam/pose',
            self.vslam_callback,
            qos_profile
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            'object_detections',
            self.detection_callback,
            qos_profile
        )

        # Timer for fusion and publishing
        self.fusion_timer = self.create_timer(1.0/self.fusion_publish_rate, self.fusion_callback)

        self.get_logger().info('Perception Fusion Node initialized')

    def vslam_callback(self, msg):
        """
        Callback for VSLAM pose updates
        """
        self.vslam_pose = msg.pose
        self.vslam_pose_timestamp = msg.header.stamp

        # Update tracked objects with robot motion
        self.update_tracked_objects_with_motion()

    def detection_callback(self, msg):
        """
        Callback for object detection updates
        """
        self.object_detections = msg.detections
        self.object_detections_timestamp = msg.header.stamp

        # Fuse new detections with existing tracks
        self.fuse_detections()

    def update_tracked_objects_with_motion(self):
        """
        Update tracked object positions based on robot motion
        """
        if self.vslam_pose is None:
            return

        # This is a simplified approach - in a real implementation,
        # you would transform all tracked object positions based on
        # the change in robot pose since the last update
        current_time = self.get_clock().now().nanoseconds / 1e9
        for obj_id, obj_data in self.tracked_objects.items():
            # Update last seen time
            obj_data['last_seen'] = self.get_clock().now()

    def fuse_detections(self):
        """
        Fuse new detections with existing tracks
        """
        if self.vslam_pose is None:
            return

        # For each detection, try to associate it with an existing track
        for detection in self.object_detections:
            if len(detection.results) == 0:
                continue

            # Get the best hypothesis
            best_hypothesis = max(detection.results, key=lambda x: x.score)
            confidence = best_hypothesis.score

            # Only process detections above confidence threshold
            if confidence < self.min_detection_confidence:
                continue

            # Convert detection center to 3D point in camera frame
            # This is a simplified approach - in reality, you'd use depth information
            # or triangulation to get 3D position
            detection_2d = detection.bbox.center
            detection_3d = self.convert_2d_to_3d(detection_2d)

            if detection_3d is None:
                continue

            # Transform to map frame using current VSLAM pose
            # This is a simplified transformation
            transformed_point = self.transform_point_to_map(detection_3d)

            if transformed_point is None:
                continue

            # Find the closest existing track or create a new one
            obj_id = self.associate_detection_with_track(transformed_point, best_hypothesis.id)

            # Update the track with the new detection
            self.update_track(obj_id, transformed_point, best_hypothesis.id, confidence)

    def convert_2d_to_3d(self, point_2d):
        """
        Convert 2D image coordinates to 3D world coordinates
        This is a simplified approach - in reality, you'd use depth information
        """
        # For this example, we'll assume a fixed depth
        # In a real implementation, you'd use depth from stereo/RGB-D
        depth = 2.0  # meters

        # Simplified pinhole camera model
        x_3d = (point_2d.x - 320.0) * depth / 554.0  # Using approximate focal length
        y_3d = (point_2d.y - 240.0) * depth / 554.0
        z_3d = depth

        point_3d = PointStamped()
        point_3d.point.x = x_3d
        point_3d.point.y = y_3d
        point_3d.point.z = z_3d

        return point_3d

    def transform_point_to_map(self, point_stamped):
        """
        Transform a point from camera frame to map frame using VSLAM pose
        """
        try:
            # This is a simplified transformation based on VSLAM pose
            # In a real implementation, you'd use proper TF transforms
            if self.vslam_pose is None:
                return None

            # Create a transform based on VSLAM pose
            # This is a simplified approach - real implementation would use TF
            transformed_point = Point()
            transformed_point.x = point_stamped.point.x + self.vslam_pose.position.x
            transformed_point.y = point_stamped.point.y + self.vslam_pose.position.y
            transformed_point.z = point_stamped.point.z + self.vslam_pose.position.z

            return transformed_point
        except Exception as e:
            self.get_logger().error(f'Error transforming point: {e}')
            return None

    def associate_detection_with_track(self, detection_point, class_id):
        """
        Associate a new detection with an existing track using nearest neighbor
        """
        if not self.tracked_objects:
            # No existing tracks, create new one
            new_id = f"{class_id}_{len(self.tracked_objects)}"
            return new_id

        # Find the closest existing track
        min_distance = float('inf')
        closest_track_id = None

        for track_id, track_data in self.tracked_objects.items():
            if track_data['positions']:
                last_position = track_data['positions'][-1]
                distance = math.sqrt(
                    (detection_point.x - last_position.x)**2 +
                    (detection_point.y - last_position.y)**2 +
                    (detection_point.z - last_position.z)**2
                )

                if distance < min_distance and distance < 1.0:  # 1 meter threshold
                    min_distance = distance
                    closest_track_id = track_id

        if closest_track_id is not None:
            return closest_track_id
        else:
            # No close track found, create new one
            new_id = f"{class_id}_{len(self.tracked_objects)}"
            return new_id

    def update_track(self, track_id, position, class_name, confidence):
        """
        Update a track with a new position measurement
        """
        track_data = self.tracked_objects[track_id]

        # Add new position
        track_data['positions'].append(position)
        track_data['class_name'] = class_name
        track_data['confidence'] = confidence

        # Calculate velocity if we have enough positions
        if len(track_data['positions']) >= 2:
            pos1 = track_data['positions'][-2]
            pos2 = track_data['positions'][-1]
            dt = 0.1  # Assume 10Hz for now

            velocity = Vector3()
            velocity.x = (pos2.x - pos1.x) / dt
            velocity.y = (pos2.y - pos1.y) / dt
            velocity.z = (pos2.z - pos1.z) / dt

            track_data['velocities'].append(velocity)

    def fusion_callback(self):
        """
        Main fusion callback that runs at a fixed rate
        """
        # Clean up old tracks
        self.cleanup_old_tracks()

        # Create fused detection message
        fused_detections = self.create_fused_detections()

        # Publish fused results
        if fused_detections:
            self.fused_objects_pub.publish(fused_detections)

        # Publish visualization
        self.publish_visualization()

    def cleanup_old_tracks(self):
        """
        Remove tracks that haven't been seen for a while
        """
        current_time = self.get_clock().now()
        tracks_to_remove = []

        for track_id, track_data in self.tracked_objects.items():
            time_since_seen = (current_time - track_data['last_seen']).nanoseconds / 1e9
            if time_since_seen > self.max_tracking_age:
                tracks_to_remove.append(track_id)

        for track_id in tracks_to_remove:
            del self.tracked_objects[track_id]

    def create_fused_detections(self):
        """
        Create a Detection2DArray message with fused information
        """
        detection_array_msg = Detection2DArray()
        detection_array_msg.header.stamp = self.get_clock().now().to_msg()
        detection_array_msg.header.frame_id = 'map'

        for track_id, track_data in self.tracked_objects.items():
            if not track_data['positions']:
                continue

            # Create detection message for this track
            detection_msg = Detection2D()

            # Set ID and hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = track_id
            hypothesis.score = track_data['confidence']
            detection_msg.results.append(hypothesis)

            # Set bounding box based on last position and uncertainty
            last_pos = track_data['positions'][-1]

            # Simplified bounding box - in reality, this would come from object detection
            detection_msg.bbox.center.x = last_pos.x
            detection_msg.bbox.center.y = last_pos.y
            detection_msg.bbox.center.z = last_pos.z
            detection_msg.bbox.size_x = 0.5  # 0.5m x 0.5m x 0.5m box
            detection_msg.bbox.size_y = 0.5
            detection_msg.bbox.size_z = 0.5

            detection_array_msg.detections.append(detection_msg)

        return detection_array_msg

    def publish_visualization(self):
        """
        Publish visualization markers for tracked objects
        """
        marker_array = MarkerArray()

        # Create markers for each tracked object
        for i, (track_id, track_data) in enumerate(self.tracked_objects.items()):
            if not track_data['positions']:
                continue

            last_pos = track_data['positions'][-1]

            # Create marker for the object
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'tracked_objects'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = last_pos.x
            marker.pose.position.y = last_pos.y
            marker.pose.position.z = last_pos.z
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3

            # Color based on object class (simplified)
            marker.color.r = 1.0
            marker.color.g = 0.0 if 'person' in track_data['class_name'] else 1.0
            marker.color.b = 0.0 if 'person' in track_data['class_name'] else 0.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

            # Create text marker with object ID
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'tracked_objects_text'
            text_marker.id = i + 1000  # Different ID space
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = last_pos.x
            text_marker.pose.position.y = last_pos.y
            text_marker.pose.position.z = last_pos.z + 0.5  # Above the object
            text_marker.pose.orientation.w = 1.0

            text_marker.scale.z = 0.2  # Text size
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.text = f"{track_data['class_name']}: {track_data['confidence']:.2f}"

            marker_array.markers.append(text_marker)

        self.debug_marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    perception_fusion_node = PerceptionFusionNode()

    try:
        rclpy.spin(perception_fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        perception_fusion_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()