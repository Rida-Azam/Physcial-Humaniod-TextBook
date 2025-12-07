# Sim-to-Real Transfer Validation Framework

This package provides tools and methodologies for evaluating sim-to-real transfer performance in humanoid robotics applications. It includes metrics, evaluation procedures, and validation frameworks to assess how well policies trained in simulation perform on real robots.

## Overview

The sim-to-real validation framework addresses the fundamental challenge in robotics of transferring policies from simulation to real-world deployment. It provides:

- Quantitative metrics for assessing transfer quality
- Evaluation procedures for different types of robotic tasks
- Visualization tools for understanding transfer performance
- Reporting mechanisms for documenting transfer results

## Key Components

### 1. Trajectory Comparison Module

Compares trajectories generated in simulation with those executed on the real robot:

- Position accuracy metrics
- Orientation accuracy metrics
- Timing synchronization analysis
- Dynamic Time Warping (DTW) for sequence alignment
- Cross-correlation analysis

### 2. Control Fidelity Assessment

Evaluates how closely simulated control commands match real robot behavior:

- Joint position tracking accuracy
- Velocity profile similarity
- Effort/torque correspondence
- Control frequency analysis

### 3. Task Success Evaluation

Measures task completion success across simulation and reality:

- Success rate comparison
- Execution time analysis
- Error type categorization
- Failure mode identification

### 4. Sensor Data Validation

Compares sensor readings between simulation and reality:

- Camera image similarity
- IMU data correlation
- Force/torque sensor comparison
- LIDAR point cloud matching

## Installation

```bash
# Clone the repository
git clone <repository-url>
cd <repository-directory>

# Install dependencies
pip3 install -r requirements.txt

# Build the package
colcon build --packages-select sim_to_real_validation
source install/setup.bash
```

## Usage

### Basic Evaluation

```python
from sim_to_real_validation import SimToRealEvaluator

# Initialize evaluator
evaluator = SimToRealEvaluator(config_path="config/validation_config.yaml")

# Load simulation and real robot data
sim_data = load_simulation_data("sim_trajectory.json")
real_data = load_real_robot_data("real_trajectory.json")

# Evaluate similarity
metrics = evaluator.evaluate_trajectory_similarity(sim_data, real_data)

# Generate comprehensive report
report = evaluator.generate_evaluation_report([sim_data, real_data], "transfer_report.json")

# Create visualizations
evaluator.visualize_evaluation_results(report, "transfer_analysis.png")
```

### Configuration

The evaluation process is configured through `config/validation_config.yaml`:

```yaml
evaluation_metrics:
  trajectory_accuracy:
    position_threshold: 0.05  # 5cm position threshold
    orientation_threshold: 0.1  # 0.1 rad orientation threshold
    timing_threshold: 0.1  # 100ms timing threshold

  control_fidelity:
    joint_position_threshold: 0.02  # 2 degrees
    velocity_threshold: 0.05  # 5% velocity difference
    effort_threshold: 0.1  # 10% effort difference

  behavior_similarity:
    dtw_weight: 0.4
    mse_weight: 0.3
    correlation_weight: 0.3

validation_thresholds:
  minimum_success_rate: 0.7  # 70% success rate
  maximum_divergence: 0.2  # 20% maximum divergence
  minimum_correlation: 0.7  # 70% minimum correlation
```

## Evaluation Metrics

### 1. Position Accuracy
Measures how closely the simulated robot's position matches the real robot:

- **RMSE (Root Mean Square Error)**: Average deviation between trajectories
- **Max Error**: Maximum instantaneous deviation
- **Success Rate**: Percentage of positions within threshold

### 2. Orientation Accuracy
Evaluates orientation tracking performance:

- **Angular Error**: Difference in orientation quaternions
- **Axis-Angle Error**: Rotation error magnitude
- **Alignment Score**: Cosine similarity of orientations

### 3. Temporal Alignment
Assesses timing synchronization:

- **Phase Difference**: Temporal offset between trajectories
- **Frequency Response**: How well frequencies are preserved
- **Latency**: System response delays

### 4. Control Consistency
Measures control signal similarity:

- **Joint Position Error**: Difference in commanded positions
- **Velocity Profile Match**: Similarity in velocity trajectories
- **Effort Correlation**: Torque/force similarity

## Transfer Quality Assessment

The framework classifies transfer quality into categories:

- **Excellent**: >90% similarity across all metrics
- **Good**: 75-90% similarity with minor adjustments needed
- **Fair**: 60-75% similarity requiring moderate retuning
- **Poor**: <60% similarity requiring significant redesign

## Visualization Tools

The package includes visualization capabilities:

- Trajectory overlay plots
- Error distribution histograms
- Correlation scatter plots
- Performance heatmaps
- Time-series comparisons

## Integration with Robotics Workflows

### ROS 2 Integration

```python
import rclpy
from sim_to_real_validation.ros_integration import SimRealComparisonNode

def main(args=None):
    rclpy.init(args=args)
    node = SimRealComparisonNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Isaac Sim Integration

The framework integrates with Isaac Sim for validation:

- Direct API access to simulation data
- Real-time comparison during training
- Automated evaluation of trained policies

## Best Practices

### 1. Domain Randomization Validation
- Test transfer performance across randomized simulation conditions
- Validate that domain randomization parameters are appropriate
- Assess robustness to environmental variations

### 2. Progressive Validation
- Start with simple tasks before complex ones
- Validate individual components before system-level tests
- Use intermediate benchmarks to track progress

### 3. Failure Analysis
- Document failure modes systematically
- Identify common causes of poor transfer
- Create targeted solutions for specific issues

## Common Challenges and Solutions

### Challenge: Visual Domain Gap
**Issue**: Significant differences between simulated and real images
**Solution**: Use domain adaptation techniques, validate on diverse visual conditions

### Challenge: Dynamics Mismatch
**Issue**: Real robot dynamics differ from simulation
**Solution**: System identification, system parameter estimation, adaptive control

### Challenge: Sensor Noise Differences
**Issue**: Simulated sensors cleaner than real ones
**Solution**: Add realistic noise models, validate on sensor quality ranges

### Challenge: Actuator Limitations
**Issue**: Perfect actuators in simulation vs. real-world limitations
**Solution**: Include actuator dynamics in simulation, validate control bandwidth

## Evaluation Protocols

### 1. Single-Policy Transfer
Evaluate one policy across simulation and reality:

1. Train policy in simulation
2. Deploy on real robot without modification
3. Measure performance degradation
4. Analyze causes of differences

### 2. Fine-Tuning Transfer
Evaluate policy after real-world fine-tuning:

1. Train base policy in simulation
2. Deploy on real robot with minimal fine-tuning
3. Measure improvement from fine-tuning
4. Assess efficiency of sim-to-real transfer

### 3. Zero-Shot Transfer
Evaluate without any real-world training:

1. Train entirely in simulation
2. Direct deployment on real robot
3. Measure zero-shot performance
4. Assess generality of simulation

## Reporting

The framework generates comprehensive reports including:

- Quantitative metrics with confidence intervals
- Qualitative assessment of transfer quality
- Identified failure modes and recommendations
- Suggested improvements for better transfer

## Contributing

See the CONTRIBUTING.md file for details on contributing to this package.

## License

This project is licensed under the Apache 2.0 License - see the LICENSE file for details.

## Acknowledgments

- NVIDIA Isaac Sim for providing the simulation platform
- ROS 2 community for the robotics framework
- Open-source robotics researchers for validation methodologies