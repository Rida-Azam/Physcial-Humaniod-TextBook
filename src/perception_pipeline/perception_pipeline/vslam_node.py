#!/usr/bin/env python3

"""
Visual SLAM Node for Humanoid Robots

This module implements a Visual Simultaneous Localization and Mapping (VSLAM)
system that allows humanoid robots to build maps of their environment and
localize themselves within it using visual input.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import message_filters

import cv2
import numpy as np
import math
from collections import deque
from scipy.spatial.transform import Rotation as R


class VSLAMNode(Node):
    """
    A node that implements Visual SLAM for humanoid robots.
    """

    def __init__(self):
        super().__init__('vslam_node')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Camera parameters (these would normally come from camera_info topic)
        self.fx = 554.256  # Focal length x
        self.fy = 554.256  # Focal length y
        self.cx = 320.5    # Principal point x
        self.cy = 240.5    # Principal point y

        # SLAM state variables
        self.current_frame = None
        self.previous_frame = None
        self.current_pose = np.eye(4)  # 4x4 transformation matrix
        self.keyframes = []
        self.map_points = []  # 3D points in the map
        self.tracked_features = []
        self.frame_id = 0

        # Feature detection parameters
        self.feature_params = dict(
            maxCorners=100,
            qualityLevel=0.01,
            minDistance=10,
            blockSize=3
        )

        # ORB detector for more robust features
        self.orb = cv2.ORB_create(nfeatures=500)

        # Publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.odom_pub = self.create_publisher(Odometry, 'vslam/odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'vslam/pose', 10)
        self.map_pub = self.create_publisher(MarkerArray, 'vslam/map', 10)

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Synchronized subscribers for stereo or RGB-D
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/rgb/image_raw', qos_profile=qos_profile)
        self.camera_info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/rgb/camera_info', qos_profile=qos_profile)

        # Approximate time synchronizer for image and camera info
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.camera_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.image_callback)

        # Timer for publishing transforms and visualization
        self.publish_timer = self.create_timer(0.05, self.publish_results)  # 20 Hz

        self.get_logger().info('VSLAM Node initialized')

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

            # Process the frame
            self.process_frame(cv_image)

            # Store current frame for next iteration
            self.previous_frame = self.current_frame.copy() if self.current_frame is not None else None
            self.current_frame = cv_image.copy()

            self.frame_id += 1

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def process_frame(self, frame):
        """
        Process a single frame for VSLAM
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.previous_frame is None:
            # Initialize with first frame
            self.current_frame = gray
            return

        # Detect features in the current frame
        kp_current = self.orb.detect(gray, None)
        kp_current, des_current = self.orb.compute(gray, kp_current)

        # Detect features in the previous frame
        kp_prev = self.orb.detect(self.previous_frame, None)
        kp_prev, des_prev = self.orb.compute(self.previous_frame, kp_prev)

        if des_prev is None or des_current is None:
            return

        # Match features between frames
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(des_prev, des_current, k=2)

        # Apply Lowe's ratio test to filter good matches
        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < 0.75 * n.distance:
                    good_matches.append(m)

        if len(good_matches) < 10:
            self.get_logger().warn(f'Not enough good matches: {len(good_matches)}')
            return

        # Extract matched points
        prev_pts = np.float32([kp_prev[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        curr_pts = np.float32([kp_current[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        # Estimate motion using Essential Matrix
        E, mask = cv2.findEssentialMat(
            curr_pts, prev_pts,
            focal=self.fx, pp=(self.cx, self.cy),
            method=cv2.RANSAC, prob=0.999, threshold=1.0
        )

        if E is not None:
            # Decompose Essential Matrix to get rotation and translation
            _, R, t, _ = cv2.recoverPose(E, curr_pts, prev_pts,
                                        focal=self.fx, pp=(self.cx, self.cy))

            # Create transformation matrix
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t.flatten()

            # Update current pose
            self.current_pose = self.current_pose @ T

            # Add to keyframes if significant movement
            if self.should_add_keyframe():
                self.keyframes.append((self.frame_id, self.current_pose.copy()))

            # Update map points (simplified)
            self.update_map_points(prev_pts, curr_pts, T)

    def should_add_keyframe(self):
        """
        Determine if a new keyframe should be added based on movement threshold
        """
        if not self.keyframes:
            return True

        # Calculate distance from last keyframe
        last_pose = self.keyframes[-1][1]
        translation_diff = np.linalg.norm(
            self.current_pose[:3, 3] - last_pose[:3, 3]
        )

        # Add keyframe if moved more than threshold
        return translation_diff > 0.5  # 0.5m threshold

    def update_map_points(self, prev_pts, curr_pts, T):
        """
        Update 3D map points based on stereo triangulation or motion
        """
        # This is a simplified approach - in a real implementation,
        # you would use triangulation from multiple viewpoints
        for i in range(0, min(len(prev_pts), len(curr_pts)), 10):  # Only process some points
            # Convert image points to normalized coordinates
            prev_norm = np.array([
                (prev_pts[i][0][0] - self.cx) / self.fx,
                (prev_pts[i][0][1] - self.cy) / self.fy
            ])

            curr_norm = np.array([
                (curr_pts[i][0][0] - self.cx) / self.fx,
                (curr_pts[i][0][1] - self.cy) / self.fy
            ])

            # Simple depth estimation based on motion (simplified)
            # In a real implementation, you would use proper triangulation
            depth = 1.0  # Placeholder depth

            # Convert to 3D point in previous frame
            prev_3d = np.array([
                prev_norm[0] * depth,
                prev_norm[1] * depth,
                depth
            ])

            # Transform to current frame
            curr_3d = T @ np.append(prev_3d, 1.0)

            # Add to map points
            if len(self.map_points) < 1000:  # Limit map size
                self.map_points.append(curr_3d[:3])

    def publish_results(self):
        """
        Publish odometry, pose, and map visualization
        """
        if self.current_pose is None:
            return

        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'vslam_frame'

        # Extract position and orientation from transformation matrix
        position = self.current_pose[:3, 3]
        rotation_matrix = self.current_pose[:3, :3]

        # Convert rotation matrix to quaternion
        r = R.from_matrix(rotation_matrix)
        quat = r.as_quat()  # Returns [x, y, z, w]

        odom_msg.pose.pose.position.x = float(position[0])
        odom_msg.pose.pose.position.y = float(position[1])
        odom_msg.pose.pose.position.z = float(position[2])
        odom_msg.pose.pose.orientation.x = float(quat[0])
        odom_msg.pose.pose.orientation.y = float(quat[1])
        odom_msg.pose.pose.orientation.z = float(quat[2])
        odom_msg.pose.pose.orientation.w = float(quat[3])

        # Publish odometry
        self.odom_pub.publish(odom_msg)

        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose = odom_msg.pose.pose
        self.pose_pub.publish(pose_msg)

        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'vslam_frame'
        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = float(position[2])
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(t)

        # Publish map visualization
        self.publish_map_visualization()

    def publish_map_visualization(self):
        """
        Publish map points as visualization markers
        """
        marker_array = MarkerArray()

        for i, point in enumerate(self.map_points[-100:]):  # Only visualize last 100 points
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'vslam_map'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(point[0])
            marker.pose.position.y = float(point[1])
            marker.pose.position.z = float(point[2])
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.map_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    vslam_node = VSLAMNode()

    try:
        rclpy.spin(vslam_node)
    except KeyboardInterrupt:
        pass
    finally:
        vslam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()