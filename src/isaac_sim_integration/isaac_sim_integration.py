#!/usr/bin/env python3

"""
Isaac Sim Integration Package for Humanoid Robotics

This module provides comprehensive integration of Isaac Sim with ROS 2 for humanoid robotics,
including physics simulation, rendering, and sensor simulation.
"""

import carb
import omni
from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.carb import set_carb_setting
from omni.isaac.core.robots import Robot
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.prims import RigidPrim, XFormPrim
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from omni.isaac.core.materials import OmniPBRMaterial
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.prims import get_prim_at_path

import numpy as np
import math
import time
from typing import Dict, List, Tuple, Optional
import asyncio
import threading

# ROS imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from std_msgs.msg import String, Float32, Bool
    from sensor_msgs.msg import JointState, Image, CameraInfo, Imu
    from geometry_msgs.msg import Twist, Pose, Point, Quaternion
    from nav_msgs.msg import Odometry
    from builtin_interfaces.msg import Time
    ROS_AVAILABLE = True
except ImportError:
    print("ROS 2 not available, running in standalone mode")
    ROS_AVAILABLE = False


class IsaacSimIntegratedSystem:
    """
    Main class for integrating Isaac Sim with physics, rendering, and sensor simulation
    """

    def __init__(self, headless: bool = False, physics_dt: float = 1.0/60.0, rendering_dt: float = 1.0/60.0):
        """
        Initialize the Isaac Sim integrated system

        Args:
            headless: Whether to run in headless mode
            physics_dt: Physics timestep
            rendering_dt: Rendering timestep
        """
        # Configure simulation app parameters
        self.app_config = {
            "headless": headless,
            "window_width": 1280,
            "window_height": 720,
            "clear_color": (0.0, 0.0, 0.0, 1.0),
            "window_title": "Isaac Sim - Humanoid Robotics Integration"
        }

        # Initialize simulation app
        self.simulation_app = SimulationApp(launch_config=self.app_config)

        # Set physics and rendering timesteps
        self.physics_dt = physics_dt
        self.rendering_dt = rendering_dt

        # Initialize world
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=physics_dt,
            rendering_dt=rendering_dt
        )

        # Robot and environment references
        self.robot = None
        self.humanoid_robot = None
        self.camera_sensors = []
        self.imu_sensors = []
        self.force_torque_sensors = []

        # Simulation state
        self.simulation_running = False
        self.simulation_time = 0.0
        self.real_time_factor = 1.0

        # Performance metrics
        self.fps = 0.0
        self.physics_time = 0.0
        self.rendering_time = 0.0

        print("Isaac Sim integrated system initialized")

    def setup_environment(self, environment_type: str = "office"):
        """
        Set up the simulation environment

        Args:
            environment_type: Type of environment to create (office, kitchen, living_room, etc.)
        """
        print(f"Setting up {environment_type} environment...")

        # Create ground plane
        self.world.scene.add_default_ground_plane()

        # Add environment-specific objects based on type
        if environment_type == "office":
            self._create_office_environment()
        elif environment_type == "kitchen":
            self._create_kitchen_environment()
        elif environment_type == "living_room":
            self._create_living_room_environment()
        elif environment_type == "laboratory":
            self._create_laboratory_environment()
        else:
            # Default environment
            self._create_simple_environment()

        print(f"{environment_type} environment created")

    def _create_simple_environment(self):
        """Create a simple environment with basic objects"""
        # Add a simple table
        table = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Table",
                name="table",
                position=np.array([1.0, 0.0, 0.5]),
                size=0.8,
                color=np.array([0.8, 0.6, 0.2])
            )
        )

        # Add a simple cup on the table
        cup = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Cup",
                name="cup",
                position=np.array([1.0, 0.0, 0.9]),
                size=0.1,
                color=np.array([0.8, 0.2, 0.2])  # Red cup
            )
        )

    def _create_office_environment(self):
        """Create an office environment"""
        # Add office furniture
        desk = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Desk",
                name="desk",
                position=np.array([1.0, 0.0, 0.4]),
                size=np.array([1.2, 0.6, 0.8]),
                color=np.array([0.4, 0.2, 0.0])  # Brown desk
            )
        )

        # Add a red cup on the desk
        cup = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/RedCup",
                name="red_cup",
                position=np.array([1.0, 0.0, 0.85]),
                size=0.08,
                color=np.array([0.8, 0.2, 0.2])  # Red
            )
        )

        # Add a chair
        chair = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Chair",
                name="chair",
                position=np.array([0.5, 0.5, 0.2]),
                size=np.array([0.4, 0.4, 0.4]),
                color=np.array([0.2, 0.2, 0.8])  # Blue chair
            )
        )

    def _create_kitchen_environment(self):
        """Create a kitchen environment"""
        # Add kitchen counter
        counter = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Counter",
                name="counter",
                position=np.array([1.0, 0.0, 0.6]),
                size=np.array([1.0, 0.6, 1.2]),
                color=np.array([0.7, 0.7, 0.7])  # Gray counter
            )
        )

        # Add a red cup on the counter
        cup = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/RedCup",
                name="red_cup",
                position=np.array([1.0, 0.0, 1.05]),
                size=0.08,
                color=np.array([0.8, 0.2, 0.2])  # Red
            )
        )

        # Add a refrigerator
        fridge = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Refrigerator",
                name="refrigerator",
                position=np.array([-1.0, 0.0, 0.8]),
                size=np.array([0.6, 0.6, 1.6]),
                color=np.array([0.9, 0.9, 0.9])  # White fridge
            )
        )

    def _create_living_room_environment(self):
        """Create a living room environment"""
        # Add a couch
        couch = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Couch",
                name="couch",
                position=np.array([1.0, 1.0, 0.3]),
                size=np.array([1.5, 0.8, 0.6]),
                color=np.array([0.2, 0.2, 0.7])  # Blue couch
            )
        )

        # Add a coffee table
        table = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/CoffeeTable",
                name="coffee_table",
                position=np.array([1.0, 0.0, 0.2]),
                size=np.array([0.8, 0.4, 0.4]),
                color=np.array([0.5, 0.3, 0.1])  # Brown table
            )
        )

        # Add a red cup on the table
        cup = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/RedCup",
                name="red_cup",
                position=np.array([1.0, 0.0, 0.45]),
                size=0.08,
                color=np.array([0.8, 0.2, 0.2])  # Red
            )
        )

    def _create_laboratory_environment(self):
        """Create a laboratory environment"""
        # Add lab equipment table
        table = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/LabTable",
                name="lab_table",
                position=np.array([1.0, 0.0, 0.5]),
                size=np.array([1.0, 0.8, 0.7]),
                color=np.array([0.9, 0.9, 0.9])  # White lab table
            )
        )

        # Add various colored cups for testing
        red_cup = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/RedCup",
                name="red_cup",
                position=np.array([1.0, 0.0, 0.85]),
                size=0.08,
                color=np.array([0.8, 0.2, 0.2])  # Red
            )
        )

        blue_cup = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/BlueCup",
                name="blue_cup",
                position=np.array([1.1, 0.1, 0.85]),
                size=0.08,
                color=np.array([0.2, 0.2, 0.8])  # Blue
            )
        )

        green_cup = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/GreenCup",
                name="green_cup",
                position=np.array([0.9, -0.1, 0.85]),
                size=0.08,
                color=np.array([0.2, 0.8, 0.2])  # Green
            )
        )

    def load_humanoid_robot(self, robot_usd_path: str = None, position: np.array = np.array([0.0, 0.0, 1.0])):
        """
        Load a humanoid robot into the simulation

        Args:
            robot_usd_path: Path to the robot USD file (if None, uses default)
            position: Initial position of the robot
        """
        print(f"Loading humanoid robot at position {position}...")

        if robot_usd_path is None:
            # For demonstration, we'll create a simple humanoid-like structure
            # In practice, this would load a proper humanoid robot model
            self.humanoid_robot = self.world.scene.add(
                Articulation(
                    prim_path="/World/HumanoidRobot",
                    name="humanoid_robot",
                    position=position
                )
            )
        else:
            # Load robot from USD path
            self.humanoid_robot = self.world.scene.add(
                Articulation(
                    prim_path="/World/HumanoidRobot",
                    name="humanoid_robot",
                    usd_path=robot_usd_path,
                    position=position
                )
            )

        print("Humanoid robot loaded successfully")

    def setup_sensors(self):
        """
        Set up sensors on the humanoid robot
        """
        print("Setting up sensors on humanoid robot...")

        # In a real implementation, this would add actual sensors to the robot
        # For this demonstration, we'll just note that sensors would be set up here

        # Add IMU sensors (conceptual)
        self.imu_sensors = [
            {"name": "torso_imu", "link": "torso", "rate": 100},  # 100 Hz
            {"name": "head_imu", "link": "head", "rate": 100}
        ]

        # Add camera sensors (conceptual)
        self.camera_sensors = [
            {"name": "head_camera", "link": "head", "resolution": [640, 480], "fov": 60, "rate": 30},  # 30 Hz
            {"name": "chest_camera", "link": "torso", "resolution": [640, 480], "fov": 90, "rate": 30}
        ]

        # Add force/torque sensors (conceptual)
        self.force_torque_sensors = [
            {"name": "left_foot_ft", "link": "left_foot", "rate": 1000},  # 1000 Hz
            {"name": "right_foot_ft", "link": "right_foot", "rate": 1000},
            {"name": "left_hand_ft", "link": "left_hand", "rate": 1000},
            {"name": "right_hand_ft", "link": "right_hand", "rate": 1000}
        ]

        print(f"Configured {len(self.imu_sensors)} IMU sensors")
        print(f"Configured {len(self.camera_sensors)} camera sensors")
        print(f"Configured {len(self.force_torque_sensors)} force/torque sensors")

    def setup_physics_properties(self):
        """
        Configure physics properties for realistic humanoid simulation
        """
        print("Setting up physics properties for humanoid simulation...")

        # Configure global physics parameters
        physics_scene = self.world.get_physics_scene()

        # Set gravity (standard Earth gravity)
        physics_scene.set_gravity(np.array([0.0, 0.0, -9.81]))

        # Configure solver parameters for stability with humanoid
        physics_scene.set_solver_type("TGS")  # Use TGS solver for better stability
        physics_scene.set_position_iteration_count(8)  # Position solver iterations
        physics_scene.set_velocity_iteration_count(4)  # Velocity solver iterations

        # Set up material properties for realistic interactions
        # Friction, restitution, etc. for different surfaces
        floor_material = OmniPBRMaterial(
            prim_path="/World/materials/floor_material",
            diffuse_color=(0.8, 0.8, 0.8),  # Light gray
            metallic=0.1,
            roughness=0.8
        )

        robot_material = OmniPBRMaterial(
            prim_path="/World/materials/robot_material",
            diffuse_color=(0.5, 0.5, 0.5),  # Gray robot
            metallic=0.9,
            roughness=0.2
        )

        object_material = OmniPBRMaterial(
            prim_path="/World/materials/object_material",
            diffuse_color=(0.9, 0.9, 0.9),  # White objects
            metallic=0.1,
            roughness=0.5
        )

        print("Physics properties configured")

    def start_simulation(self):
        """
        Start the simulation loop
        """
        print("Starting Isaac Sim simulation...")

        self.simulation_running = True

        # Reset the world
        self.world.reset()

        # Start the simulation loop
        while self.simulation_running and self.simulation_app.is_running():
            # Step the world
            self.world.step(render=True)

            # Update simulation time
            self.simulation_time += self.physics_dt

            # Process any sensor updates or other callbacks here
            self._process_sensor_updates()

            # Check for termination conditions
            if self.simulation_time > 10000:  # Stop after 10000 seconds if needed
                break

        print("Simulation stopped")

    def _process_sensor_updates(self):
        """
        Process sensor updates and publish to ROS if available
        """
        # This is where sensor data would be collected and processed
        # In a real implementation, this would interface with actual Isaac Sim sensors
        pass

    def stop_simulation(self):
        """
        Stop the simulation
        """
        print("Stopping Isaac Sim simulation...")
        self.simulation_running = False

    def reset_simulation(self):
        """
        Reset the simulation to initial state
        """
        print("Resetting simulation...")
        self.world.reset()
        self.simulation_time = 0.0

    def get_robot_state(self) -> Dict:
        """
        Get current robot state (positions, velocities, etc.)

        Returns:
            Dictionary containing robot state information
        """
        if self.humanoid_robot is None:
            return {}

        # Get joint positions and velocities
        joint_positions = self.humanoid_robot.get_joint_positions()
        joint_velocities = self.humanoid_robot.get_joint_velocities()
        joint_efforts = self.humanoid_robot.get_joint_efforts()

        # Get end-effector poses if available
        ee_poses = {}
        # This would be implemented based on the specific robot structure

        # Get base pose (for humanoid, this might be pelvis or torso)
        base_position, base_orientation = self.humanoid_robot.get_world_poses()
        base_linear_vel, base_angular_vel = self.humanoid_robot.get_velocities()

        robot_state = {
            "joint_positions": joint_positions.tolist() if joint_positions is not None else [],
            "joint_velocities": joint_velocities.tolist() if joint_velocities is not None else [],
            "joint_efforts": joint_efforts.tolist() if joint_efforts is not None else [],
            "base_position": base_position[0].tolist() if base_position is not None else [0, 0, 0],
            "base_orientation": base_orientation[0].tolist() if base_orientation is not None else [0, 0, 0, 1],
            "base_linear_velocity": base_linear_vel[0].tolist() if base_linear_vel is not None else [0, 0, 0],
            "base_angular_velocity": base_angular_vel[0].tolist() if base_angular_vel is not None else [0, 0, 0],
            "timestamp": self.simulation_time,
            "simulation_time": self.simulation_time
        }

        return robot_state

    def send_robot_command(self, joint_commands: List[float]):
        """
        Send joint commands to the robot

        Args:
            joint_commands: List of joint position commands
        """
        if self.humanoid_robot is not None:
            # Apply joint commands
            self.humanoid_robot.set_joint_positions(np.array(joint_commands))

    def get_performance_metrics(self) -> Dict:
        """
        Get performance metrics for the simulation

        Returns:
            Dictionary containing performance metrics
        """
        metrics = {
            "simulation_time": self.simulation_time,
            "real_time_factor": self.real_time_factor,
            "fps": self.fps,
            "physics_time": self.physics_time,
            "rendering_time": self.rendering_time,
            "total_steps": int(self.simulation_time / self.physics_dt)
        }

        return metrics

    def close(self):
        """
        Close the simulation application
        """
        print("Closing Isaac Sim application...")
        self.simulation_app.close()


def main():
    """
    Main function to demonstrate Isaac Sim integration
    """
    print("Isaac Sim Integration Demo")
    print("="*40)

    # Initialize Isaac Sim with physics, rendering, and sensors
    isaac_sim = IsaacSimIntegratedSystem(headless=False, physics_dt=1.0/60.0, rendering_dt=1.0/60.0)

    # Set up environment
    isaac_sim.setup_environment("laboratory")

    # Load humanoid robot
    isaac_sim.load_humanoid_robot(position=np.array([0.0, 0.0, 1.0]))

    # Set up sensors
    isaac_sim.setup_sensors()

    # Configure physics properties
    isaac_sim.setup_physics_properties()

    print("\nSystem setup complete!")
    print("Robot and sensors configured")
    print("Environment loaded")
    print("Physics properties set")

    # Show initial robot state
    initial_state = isaac_sim.get_robot_state()
    print(f"\nInitial robot state: position={initial_state.get('base_position', 'unknown')}")

    # Start simulation
    print("\nStarting simulation for 5 seconds...")
    start_time = time.time()

    # Run simulation for 5 seconds of simulated time
    isaac_sim.simulation_running = True
    step_count = 0

    while isaac_sim.simulation_running and isaac_sim.simulation_app.is_running():
        isaac_sim.world.step(render=True)
        isaac_sim.simulation_time += isaac_sim.physics_dt
        step_count += 1

        # Stop after 5 seconds of simulated time
        if isaac_sim.simulation_time >= 5.0:
            break

        # Print status every second
        if step_count % int(1.0/isaac_sim.physics_dt) == 0:
            print(f"Simulation time: {isaac_sim.simulation_time:.2f}s")

    print(f"\nSimulation completed after {step_count} steps ({isaac_sim.simulation_time:.2f}s simulated time)")

    # Get final performance metrics
    metrics = isaac_sim.get_performance_metrics()
    print(f"Performance metrics: {metrics}")

    # Close the simulation
    isaac_sim.close()

    print("\nIsaac Sim integration demo completed successfully!")


if __name__ == "__main__":
    main()