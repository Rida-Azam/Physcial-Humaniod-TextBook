# Datasets for Humanoid Robotics

This directory contains datasets used for training and evaluating the humanoid robotics system, particularly for Vision-Language-Action (VLA) models.

## Dataset Overview

### Trajectory Dataset

The trajectory dataset contains 500+ robot trajectories for training VLA models. Each trajectory includes:

- **Images**: Sequences of RGB images from the robot's camera
- **Commands**: Natural language commands corresponding to the task
- **Actions**: Robot actions executed in response to commands
- **Metadata**: Additional information about the trajectory

#### Data Format

Each trajectory is stored as a pickle file with the following structure:

```python
{
    "trajectory_id": "unique_identifier",
    "images": [list of RGB images (numpy arrays)],
    "commands": [list of command strings],
    "actions": [list of action vectors (numpy arrays)],
    "timestamps": [list of timestamp floats],
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

Trajectory files follow the naming convention: `traj_XXXX.pkl` where XXXX is a zero-padded number.

## Generating Additional Data

To generate more trajectory data, use the provided script:

```bash
python3 generate_trajectories.py
```

You can customize the number of trajectories by modifying the `num_trajectories` parameter in the script.

## Using the Dataset

### Loading a Trajectory

```python
import pickle

# Load a single trajectory
with open('datasets/trajectories/traj_0000.pkl', 'rb') as f:
    trajectory = pickle.load(f)

# Access components
images = trajectory['images']
commands = trajectory['commands']
actions = trajectory['actions']
```

### Loading the Entire Dataset

```python
import os
import pickle

dataset_dir = 'datasets/trajectories'
trajectories = []

for filename in os.listdir(dataset_dir):
    if filename.endswith('.pkl'):
        with open(os.path.join(dataset_dir, filename), 'rb') as f:
            trajectory = pickle.load(f)
            trajectories.append(trajectory)
```

## Dataset Statistics

- Total trajectories: 500+
- Average trajectory length: 30 steps
- Image dimensions: 224×224×3 RGB
- Action dimensions: 7-DoF joint positions
- Environments: Simulated office, kitchen, and living spaces
- Commands: 15 common manipulation commands

## Data Preprocessing

For training VLA models, the raw data may need preprocessing:

1. **Image normalization**: Images should be normalized using ImageNet statistics
2. **Action scaling**: Actions may need scaling depending on the model
3. **Temporal alignment**: Aligning vision and action sequences

## Citation

If using this dataset for research, please cite:

```
Author, A.A., et al. (2025). "Dataset for Physical AI & Humanoid Robotics Research".
```

## License

The dataset is released under the Creative Commons Attribution 4.0 International (CC BY 4.0) license. See LICENSE file for details.