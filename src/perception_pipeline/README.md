# Perception Pipeline for Humanoid Robots

This package provides a complete perception pipeline for humanoid robots, combining Visual SLAM (VSLAM) and object detection to create a comprehensive understanding of the environment.

## Components

The perception pipeline consists of three main nodes:

1. **VSLAM Node**: Performs visual SLAM to estimate robot pose and build a map of the environment
2. **Object Detection Node**: Detects and classifies objects in the camera feed
3. **Perception Fusion Node**: Fuses VSLAM and object detection data to create a unified perception of the environment

## Installation

1. Install required dependencies:
```bash
pip3 install torch torchvision opencv-python numpy scipy
```

2. Clone this repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository-url>
```

3. Build the workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select perception_pipeline
source install/setup.bash
```

## Usage

### Launch the full pipeline:

```bash
ros2 launch perception_pipeline perception_pipeline_launch.py
```

### Launch individual components:

```bash
# Launch only VSLAM
ros2 run perception_pipeline vslam_node

# Launch only object detection
ros2 run perception_pipeline object_detection_node

# Launch perception fusion
ros2 run perception_pipeline perception_fusion_node
```

## Topics

The perception pipeline publishes the following topics:

- `/vslam/odom` - Odometry from visual SLAM
- `/vslam/pose` - Pose estimate from visual SLAM
- `/vslam/map` - 3D map points from VSLAM
- `/object_detections` - Object detections from the detection node
- `/object_detection/debug_image` - Debug image with bounding boxes
- `/perception_fused/objects` - Fused object detections in map frame
- `/perception_fused/map` - Fused map with object information
- `/perception_fused/debug` - Debug visualization markers

## Configuration

The pipeline behavior can be adjusted through the configuration file:

- `fx`, `fy`, `cx`, `cy`: Camera intrinsic parameters
- `confidence_threshold`: Minimum confidence for object detection
- `keyframe_translation_threshold`: Distance threshold for adding new keyframes in VSLAM
- `max_tracking_age`: Maximum time to keep tracking an object without detection
- `association_threshold`: Distance threshold for associating detections with tracks

## Integration with Humanoid Robots

The perception pipeline is designed to work with humanoid robots by:

- Providing 3D position estimates for detected objects
- Maintaining a map of the environment that can be used for navigation
- Tracking moving objects over time
- Operating in real-time to support robot control

## Performance Considerations

- VSLAM requires significant computational resources
- Object detection performance depends on available GPU
- Consider reducing camera resolution for real-time performance
- Adjust feature count and tracking parameters based on computational capabilities

## Troubleshooting

- If VSLAM fails to track, ensure adequate lighting and visual features in the environment
- If object detection is slow, consider using a lighter model or reducing input resolution
- Check camera calibration parameters for accurate 3D reconstruction
- Monitor CPU/GPU usage during operation

## References

This implementation combines state-of-the-art techniques in:

- Visual SLAM using feature-based methods
- Deep learning-based object detection
- Multi-object tracking and data association