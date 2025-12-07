# Isaac Sim Integration Package

This package provides comprehensive integration of Isaac Sim with physics, rendering, and sensor simulation for humanoid robotics applications.

## Overview

The Isaac Sim Integration package provides a complete framework for simulating humanoid robots with realistic physics, high-fidelity rendering, and accurate sensor simulation. It serves as the foundation for developing, testing, and validating humanoid robot control algorithms in a safe and efficient simulated environment.

## Features

### Physics Simulation
- Realistic rigid body dynamics with NVIDIA PhysX engine
- Accurate contact modeling for humanoid locomotion
- Configurable physics parameters for different scenarios
- Support for complex multi-body systems with many degrees of freedom
- Realistic friction and restitution properties

### Rendering
- High-fidelity visual rendering with NVIDIA RTX technology
- Photorealistic environments with dynamic lighting
- Support for various camera configurations
- Realistic material properties and textures
- Adjustable rendering quality for performance optimization

### Sensor Simulation
- IMU simulation with configurable noise models
- Camera simulation with realistic distortion
- Force/torque sensor simulation
- LIDAR and depth sensor simulation
- Proprioceptive sensor simulation

### Humanoid-Specific Features
- Support for complex humanoid kinematics
- Balance and locomotion simulation
- Manipulation and grasping simulation
- Multi-modal perception simulation
- Realistic humanoid dynamics

## Installation

### Prerequisites

- Ubuntu 22.04
- NVIDIA GPU with CUDA support (RTX series recommended)
- Isaac Sim 2023.2 or later
- ROS 2 Jazzy

### Setup

1. Install Isaac Sim from NVIDIA Developer portal
2. Set up the Isaac Sim environment:
   ```bash
   cd ~/isaac_sim_ws
   git clone <repository_url>
   cd src
   ln -s ../../src/isaac_sim_integration .
   cd ..
   colcon build --packages-select isaac_sim_integration
   source install/setup.bash
   ```

## Usage

### Basic Usage

```bash
# Launch Isaac Sim with integrated physics and sensor simulation
ros2 launch isaac_sim_integration integrated_simulation.launch.py
```

### Programmatic Usage

```python
from isaac_sim_integration import IsaacSimIntegratedSystem

# Create integrated system
isaac_sim = IsaacSimIntegratedSystem(
    headless=False,
    physics_dt=1.0/60.0,
    rendering_dt=1.0/60.0
)

# Set up environment
isaac_sim.setup_environment("office")

# Load humanoid robot
isaac_sim.load_humanoid_robot(position=[0.0, 0.0, 1.0])

# Set up sensors
isaac_sim.setup_sensors()

# Configure physics properties
isaac_sim.setup_physics_properties()

# Start simulation
isaac_sim.start_simulation()

# Get robot state
robot_state = isaac_sim.get_robot_state()

# Send commands
isaac_sim.send_robot_command([0.1, 0.2, 0.3, ...])  # Joint commands

# Stop simulation
isaac_sim.stop_simulation()
```

## Configuration

### Physics Parameters

Physics parameters can be configured in `config/physics_config.yaml`:

```yaml
physics:
  gravity: [0.0, 0.0, -9.81]
  solver_type: "TGS"  # TGS or PGS
  position_iterations: 8
  velocity_iterations: 4
  max_substeps: 16
  adaptivity_threshold: 0.1
  sleep_threshold: 0.005
  stabilization_threshold: 0.01
```

### Rendering Parameters

Rendering parameters can be configured in `config/rendering_config.yaml`:

```yaml
rendering:
  resolution:
    width: 1280
    height: 720
  quality_level: "high"  # low, medium, high, ultra
  enable_shadows: true
  enable_reflections: false
  enable_global_illumination: false
  anti_aliasing: "fxaa"  # fxaa, taa, or off
  texture_resolution: "high"
```

### Sensor Parameters

Sensor parameters can be configured in `config/sensor_config.yaml`:

```yaml
sensors:
  imu:
    sample_rate: 100  # Hz
    noise_density: 100.0e-6  # g/sqrt(Hz)
    random_walk: 1.0e-6  # g/s^2/sqrt(Hz)
    bias_instability: 5.0e-6  # g

  camera:
    sample_rate: 30  # Hz
    resolution: [640, 480]
    fov: 60  # degrees
    noise_std: 0.01
    distortion:
      k1: -0.1
      k2: 0.02
      p1: 0.0
      p2: 0.0

  force_torque:
    sample_rate: 1000  # Hz
    noise_std: 0.1  # N for force, Nm for torque
    range: [1000.0, 100.0]  # Max force (N), Max torque (Nm)
```

## Environment Types

The package supports various environment types:

- `simple`: Basic environment with ground plane
- `office`: Office environment with desks, chairs, and objects
- `kitchen`: Kitchen environment with counters, appliances, and objects
- `living_room`: Living room environment with furniture
- `laboratory`: Laboratory environment with equipment and test objects
- `custom`: Custom environment loaded from USD file

## Robot Support

Currently supports humanoid robots with up to 30+ degrees of freedom. The system can be extended to support other robot types by modifying the robot loading mechanism.

## Performance Optimization

### Real-Time Performance Tips

1. Adjust rendering quality based on required performance
2. Use appropriate physics timesteps (typically 1ms for humanoid robots)
3. Enable GPU acceleration for physics computation
4. Use simplified collision meshes during development
5. Adjust solver parameters based on stability requirements

### Multi-Environment Training

For reinforcement learning applications, the system supports multi-environment training with instance rendering for improved performance.

## Integration with ROS 2

The package integrates seamlessly with ROS 2 through the Isaac ROS bridge, providing standard ROS 2 message types for:

- Joint states (`sensor_msgs/JointState`)
- IMU data (`sensor_msgs/Imu`)
- Camera images (`sensor_msgs/Image`)
- Robot odometry (`nav_msgs/Odometry`)
- TF transforms (`tf2_msgs/TFMessage`)
- Robot commands (`std_msgs/Float64MultiArray`)

## Examples

### Example 1: Basic Humanoid Simulation

```python
# Example of simulating a basic humanoid walking task
from isaac_sim_integration import IsaacSimIntegratedSystem

def walk_demo():
    # Initialize simulation
    sim = IsaacSimIntegratedSystem()
    sim.setup_environment("simple")
    sim.load_humanoid_robot()
    sim.setup_sensors()
    sim.setup_physics_properties()

    # Execute walking pattern
    for step in range(1000):
        sim.world.step(render=True)

        # Simple walking controller
        if step % 50 == 0:  # Update every 50 steps
            # Send walking commands
            sim.send_robot_command(calculate_walking_pattern(step))

    sim.close()

if __name__ == "__main__":
    walk_demo()
```

### Example 2: Object Manipulation

```python
# Example of simulating object manipulation
from isaac_sim_integration import IsaacSimIntegratedSystem

def manipulation_demo():
    sim = IsaacSimIntegratedSystem()
    sim.setup_environment("office")
    sim.load_humanoid_robot()
    sim.setup_sensors()
    sim.setup_physics_properties()

    # Add object to manipulate
    red_cup = sim.add_object("red_cup", position=[0.5, 0.0, 0.8])

    # Execute manipulation task
    for step in range(2000):
        sim.world.step(render=True)

        # Simple manipulation controller
        if step % 100 == 0:
            # Send manipulation commands
            sim.send_robot_command(calculate_manipulation_pattern(step, red_cup))

    sim.close()

if __name__ == "__main__":
    manipulation_demo()
```

## Troubleshooting

### Common Issues

1. **Low Performance**: Reduce rendering quality or physics accuracy
2. **Instability**: Increase solver iterations or reduce timestep
3. **Sensor Noise**: Check sensor configuration parameters
4. **Collision Issues**: Verify collision mesh quality and physics parameters

### Debugging

Enable debug visualization:
```bash
export GZ_GUI_PLUGIN_VISIBLE_ON_STARTUP="Inspector, World Inspector"
ros2 launch isaac_sim_integration integrated_simulation.launch.py
```

Monitor performance:
```bash
# Monitor FPS and other metrics
ros2 topic echo /isaac_sim/performance_metrics
```

## Extending the System

The system is designed to be extensible:

1. Add new environment types by extending the environment setup functions
2. Add new sensor types by implementing sensor interfaces
3. Add new robot types by extending the robot loading mechanism
4. Add new physics scenarios by configuring physics parameters

## Contributing

See the CONTRIBUTING.md file for details on contributing to this package.

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

## Acknowledgments

- NVIDIA Isaac Sim team for the simulation platform
- ROS 2 community for the robotics framework
- Open robotics community for various tools and libraries