# Isaac Sim Validation Framework

This framework provides tools for validating Isaac Sim environments against real-world conditions and optimizing simulation performance for humanoid robotics applications.

## Overview

The validation framework includes:

- **Kinematic Validation**: Validates joint position accuracy between simulation and real robot
- **Dynamic Validation**: Validates dynamic behavior and trajectory tracking
- **Balance Validation**: Validates balance and stability characteristics
- **Locomotion Validation**: Validates walking and navigation performance
- **Manipulation Validation**: Validates grasping and manipulation success rates
- **Performance Optimization**: Optimizes simulation settings for different use cases

## Installation

The validation framework is part of the overall project and shares dependencies:

```bash
# Ensure Isaac Sim and ROS 2 are properly installed
# Install Python dependencies
pip install -r requirements.txt

# For visualization and analysis
pip install matplotlib seaborn pandas numpy scipy
```

## Usage

### Running Validation

```python
from validation_framework import IsaacSimValidator, PerformanceOptimizer

# Initialize validator
validator = IsaacSimValidator(config_path="config/validation_config.yaml")

# Prepare simulation and real-world data
sim_data = {...}  # Simulation data dictionary
real_data = {...}  # Real-world data dictionary

# Run comprehensive validation
validation_report = validator.run_comprehensive_validation(sim_data, real_data)

# Generate report
validator.generate_validation_report(validation_report, "validation_results.pdf")
```

### Running Optimization

```python
from validation_framework import PerformanceOptimizer

# Initialize optimizer
optimizer = PerformanceOptimizer(config_path="config/validation_config.yaml")

# Optimize for specific use case
settings, benchmark_results = optimizer.find_optimal_settings(objective="balanced")

print(f"Optimized FPS: {benchmark_results['fps']:.2f}")
print(f"Memory usage: {benchmark_results['memory_usage_percent']:.1f}%")
```

## Configuration

The framework is configured through `config/validation_config.yaml` which includes:

- **Validation thresholds**: Acceptable error bounds for different metrics
- **Performance targets**: Minimum performance requirements
- **Physics parameters**: Settings for physics simulation validation
- **Rendering parameters**: Settings for visual simulation validation
- **Test scenarios**: Different scenarios to validate
- **Data collection**: Settings for collecting validation data
- **Reporting**: Settings for generating validation reports
- **Optimization**: Settings for performance optimization

## Validation Process

The validation process includes several steps:

1. **Data Collection**: Collect synchronized data from simulation and real robot
2. **Metric Calculation**: Compute relevant metrics for each validation category
3. **Threshold Comparison**: Compare metrics against validation thresholds
4. **Report Generation**: Create comprehensive validation report
5. **Optimization**: Suggest performance optimizations based on results

## Performance Optimization

The framework supports optimization for different objectives:

- **Performance**: Maximize simulation speed
- **Quality**: Maximize simulation accuracy
- **Balanced**: Balance performance and quality
- **Training**: Optimize for machine learning training scenarios
- **Visualization**: Optimize for high-quality rendering

## Output Files

The validation framework generates:

- **JSON Reports**: Detailed validation results in JSON format
- **PDF Reports**: Comprehensive reports with visualizations
- **Log Files**: Detailed logs of validation process
- **Plots**: Visualizations of validation results
- **Optimization Suggestions**: Recommended parameter settings

## Integration with Isaac Sim

The framework integrates with Isaac Sim through:

- USD file configuration for simulation environments
- ROS 2 interfaces for data collection
- Isaac Sim API for parameter adjustment
- Custom plugins for specialized validation metrics

## Evaluation Metrics

### Kinematic Metrics
- Joint position accuracy
- End-effector position accuracy
- Inverse kinematics solution accuracy

### Dynamic Metrics
- Trajectory tracking accuracy
- Velocity and acceleration accuracy
- Energy consumption comparison

### Balance Metrics
- Center of Mass (CoM) tracking
- Zero Moment Point (ZMP) tracking
- Stability margins

### Task Performance Metrics
- Success rate for manipulation tasks
- Navigation accuracy
- Execution time

## Troubleshooting

If validation fails:
1. Check that simulation and real data are properly synchronized
2. Verify that units are consistent between simulation and real data
3. Ensure that robot configurations match between sim and real
4. Check that validation thresholds are appropriate for your system

For performance issues:
1. Reduce physics solver iterations
2. Lower rendering quality settings
3. Use level-of-detail (LOD) for complex models
4. Optimize collision meshes

## Extending the Framework

To add new validation tests:
1. Create a new validation method in the IsaacSimValidator class
2. Add appropriate metrics and thresholds to the configuration
3. Update the comprehensive validation method to include your test
4. Add visualization for your new metrics

## License

This framework is released under the Apache 2.0 license.