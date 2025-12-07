# Datasets for Humanoid Robotics

This directory contains datasets used for training and evaluating the humanoid robotics system, particularly for Vision-Language-Action (VLA) models.

## Dataset Overview

### Trajectory Dataset

The trajectory dataset contains 500+ robot trajectories for training VLA models. Each trajectory includes:

- **Images**: Sequences of RGB images from the robot's camera (simulated filenames)
- **Commands**: Natural language commands corresponding to the task
- **Actions**: Robot actions executed in response to commands
- **Metadata**: Additional information about the trajectory

#### Data Format

Each trajectory is stored as a JSON file with the following structure:

```json
{
    "trajectory_id": "unique_identifier",
    "images": ["image_001.jpg", "image_002.jpg", ...],
    "commands": ["command1", "command2", ...],
    "actions": [[0.1, 0.2, ...], [0.3, 0.4, ...], ...],
    "timestamps": [timestamp1, timestamp2, ...],
    "metadata": {
        "traj_length": length of trajectory,
        "start_time": start timestamp,
        "end_time": end timestamp,
        "primary_command": main command for the trajectory,
        "robot_type": type of robot,
        "environment": environment type
    }
}
```

#### File Naming Convention

Trajectory files follow the naming convention: `traj_XXXX.json` where XXXX is a zero-padded number.

## Dataset Statistics

- Total trajectories: 500
- Average trajectory length: ~20 steps
- Action dimensions: 7-DoF joint positions
- Environments: Simulated office, kitchen, and living spaces

## Citation

If using this dataset for research, please cite:

```
Author, A.A., et al. (2025). "Dataset for Physical AI & Humanoid Robotics Research".
```

## License

The dataset is released under the Creative Commons Attribution 4.0 International (CC BY 4.0) license. See LICENSE file for details.
