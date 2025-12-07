#!/usr/bin/env python3

"""
Script to generate robot trajectories dataset

This script creates a synthetic dataset of robot trajectories for training
VLA (Vision-Language-Action) models. Each trajectory includes:
- Image sequences from robot's camera
- Language commands
- Action sequences executed by the robot
"""

import numpy as np
import pickle
import json
import os
from datetime import datetime
from dataclasses import dataclass
from typing import List, Dict, Tuple


@dataclass
class Trajectory:
    """Represents a single robot trajectory with vision, language, and action components"""
    trajectory_id: str
    images: List[np.ndarray]  # List of RGB images
    commands: List[str]       # List of natural language commands
    actions: List[np.ndarray] # List of action vectors
    timestamps: List[float]   # Timestamps for each step
    metadata: Dict            # Additional metadata


class TrajectoryGenerator:
    """Generates synthetic robot trajectories for training VLA models"""

    def __init__(self, num_trajectories=500):
        self.num_trajectories = num_trajectories
        self.dataset_path = "datasets/trajectories"

        # Define possible actions for humanoid robot
        self.action_descriptions = [
            "move_forward",
            "turn_left",
            "turn_right",
            "move_backward",
            "reach_forward",
            "reach_left",
            "reach_right",
            "grasp_object",
            "lift_object",
            "place_object"
        ]

        # Define possible commands
        self.commands = [
            "Pick up the red cup",
            "Move to the blue box",
            "Go to the table",
            "Grasp the green object",
            "Navigate to the chair",
            "Move forward",
            "Turn left",
            "Turn right",
            "Pick up the small ball",
            "Place object on table",
            "Go to the left side",
            "Go to the right side",
            "Move toward the window",
            "Approach the door",
            "Grab the yellow item"
        ]

        # Create directory if it doesn't exist
        os.makedirs(self.dataset_path, exist_ok=True)

    def generate_single_trajectory(self, traj_id: str) -> Trajectory:
        """Generate a single trajectory with random actions"""
        # Random length of trajectory (between 10 and 50 steps)
        traj_length = np.random.randint(10, 51)

        images = []
        commands = []
        actions = []
        timestamps = []

        # Generate trajectory steps
        for step in range(traj_length):
            # Generate a random image (simulating camera input)
            # In reality, this would come from a simulation or real robot
            image = np.random.rand(224, 224, 3).astype(np.float32)  # RGB image
            images.append(image)

            # Assign command (usually only one command per trajectory,
            # but for training we might have intermediate commands)
            if step == 0:  # First step gets the main command
                command = np.random.choice(self.commands)
            else:
                command = ""  # Empty for intermediate steps
            commands.append(command)

            # Generate action (7-DoF joint positions or Cartesian poses)
            action = np.random.randn(7).astype(np.float32)  # Example: 7-DoF action
            actions.append(action)

            # Add timestamp
            timestamps.append(datetime.now().timestamp())

        # Create metadata
        metadata = {
            "traj_length": traj_length,
            "start_time": timestamps[0],
            "end_time": timestamps[-1],
            "primary_command": commands[0] if commands[0] else "no_command",
            "action_descriptions": self.action_descriptions,
            "robot_type": "humanoid",
            "environment": "simulated_office"
        }

        return Trajectory(
            trajectory_id=traj_id,
            images=images,
            commands=commands,
            actions=actions,
            timestamps=timestamps,
            metadata=metadata
        )

    def generate_dataset(self):
        """Generate the complete dataset with specified number of trajectories"""
        print(f"Generating {self.num_trajectories} trajectories...")

        trajectories = []

        for i in range(self.num_trajectories):
            traj_id = f"traj_{i:04d}"
            print(f"Generating trajectory {i+1}/{self.num_trajectories}: {traj_id}")

            trajectory = self.generate_single_trajectory(traj_id)
            trajectories.append(trajectory)

            # Save trajectory to disk
            self.save_trajectory(trajectory)

        # Save dataset info
        dataset_info = {
            "total_trajectories": len(trajectories),
            "generation_date": datetime.now().isoformat(),
            "num_trajectories": self.num_trajectories,
            "trajectory_length_range": [10, 50],
            "action_dimension": 7,
            "image_dimensions": [224, 224, 3],
            "commands_used": self.commands,
            "actions_used": self.action_descriptions
        }

        with open(os.path.join(self.dataset_path, "dataset_info.json"), "w") as f:
            json.dump(dataset_info, f, indent=2)

        print(f"Dataset generation completed! Saved to {self.dataset_path}/")
        print(f"Total trajectories: {len(trajectories)}")
        print(f"Dataset info saved to {self.dataset_path}/dataset_info.json")

    def save_trajectory(self, trajectory: Trajectory):
        """Save a single trajectory to disk"""
        traj_path = os.path.join(self.dataset_path, f"{trajectory.trajectory_id}.pkl")

        # Convert numpy arrays to lists for serialization (optional)
        serializable_traj = {
            "trajectory_id": trajectory.trajectory_id,
            "images": [img.tolist() for img in trajectory.images],  # Convert to list for JSON compatibility
            "commands": trajectory.commands,
            "actions": [act.tolist() for act in trajectory.actions],  # Convert to list for JSON compatibility
            "timestamps": trajectory.timestamps,
            "metadata": trajectory.metadata
        }

        with open(traj_path, "wb") as f:
            pickle.dump(serializable_traj, f)

    def load_trajectory(self, traj_id: str) -> Trajectory:
        """Load a trajectory from disk"""
        traj_path = os.path.join(self.dataset_path, f"{traj_id}.pkl")

        with open(traj_path, "rb") as f:
            data = pickle.load(f)

        # Convert lists back to numpy arrays
        trajectory = Trajectory(
            trajectory_id=data["trajectory_id"],
            images=[np.array(img) for img in data["images"]],
            commands=data["commands"],
            actions=[np.array(act) for act in data["actions"]],
            timestamps=data["timestamps"],
            metadata=data["metadata"]
        )

        return trajectory


def main():
    """Main function to generate the dataset"""
    print("Starting trajectory dataset generation...")

    # Create generator with 500+ trajectories
    generator = TrajectoryGenerator(num_trajectories=500)

    # Generate the dataset
    generator.generate_dataset()

    print("Dataset generation complete!")

    # Verify the dataset
    print("\nVerifying dataset...")
    sample_traj = generator.load_trajectory("traj_0000")
    print(f"Sample trajectory ID: {sample_traj.trajectory_id}")
    print(f"Trajectory length: {len(sample_traj.images)} steps")
    print(f"Image shape: {sample_traj.images[0].shape}")
    print(f"Action shape: {sample_traj.actions[0].shape}")
    print(f"Primary command: {sample_traj.metadata['primary_command']}")


if __name__ == "__main__":
    main()