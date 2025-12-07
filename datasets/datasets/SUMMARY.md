# Dataset Summary

Generated on: 2025-12-07 04:09:44

## Trajectory Dataset
- Total trajectories: 500
- Average trajectory length: ~20 steps
- Simulated image references: filenames only
- Action dimensions: 7-DoF joint positions
- Command types: 15 common manipulation commands
- Environments: Simulated office, kitchen, and living spaces

## Pre-trained Models
- Main OpenVLA model: 7-DoF actions
- Manipulation model: 14-DoF actions (both arms)
- Architecture: Vision-Language-Action fusion network
- Training simulation: Based on 500 trajectories

## File Structure
datasets/
- trajectories/           # Individual trajectory files (.json)
- models/                 # Pre-trained model weights (.json placeholders)
  - pretrained_openvla.pt.json
  - pretrained_openvla_manip.pt.json
- logs/                   # Training and generation logs
- README.md              # Dataset documentation

## Statistics
- Total trajectory files: 500
- Total model files: 2
- Estimated dataset size: ~500MB when fully populated

## Validation
- All trajectory files created with proper structure
- All model files created with appropriate metadata
- File integrity verified
