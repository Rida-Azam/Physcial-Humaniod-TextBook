#!/usr/bin/env python3

"""
Script to run sim-to-real transfer evaluation for humanoid robotics project
"""

import os
import sys
import json
import yaml
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from datetime import datetime
from pathlib import Path

# Add the current directory to the Python path to import our modules
sys.path.insert(0, str(Path(__file__).parent))

from sim_to_real_evaluator import SimToRealEvaluator, TrajectoryPair


def load_config(config_path):
    """Load configuration from YAML file"""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def generate_sample_data():
    """Generate sample simulation and real robot data for evaluation"""

    # Generate sample simulation trajectory
    sim_trajectory = []
    for i in range(100):
        # Create a trajectory with a circular motion pattern
        t = i * 0.1  # Time parameter
        x = 0.5 * np.cos(t)
        y = 0.5 * np.sin(t)
        z = 0.8 + 0.1 * np.sin(2*t)  # Slight height variation

        # Add realistic noise to simulation
        x += np.random.normal(0, 0.01)
        y += np.random.normal(0, 0.01)
        z += np.random.normal(0, 0.005)

        point = {
            'position': [float(x), float(y), float(z)],
            'orientation': [0.0, 0.0, np.sin(t/2), np.cos(t/2)],  # Simple orientation change
            'joint_positions': [float(np.sin(t + j*0.5)) for j in range(28)],  # 28 DoF humanoid
            'timestamp': float(i * 0.1)
        }
        sim_trajectory.append(point)

    # Generate sample real trajectory (similar but with systematic differences)
    real_trajectory = []
    for i, sim_point in enumerate(sim_trajectory):
        # Create real trajectory with realistic differences
        real_point = sim_point.copy()

        # Add systematic differences to simulate sim-to-real gap
        pos_offset = np.random.normal(0, 0.03, 3)  # 3cm average offset
        real_pos = np.array(sim_point['position']) + pos_offset
        real_point['position'] = real_pos.tolist()

        # Add differences to joint positions
        joint_diff = np.random.normal(0, 0.02, len(sim_point['joint_positions']))
        real_joints = np.array(sim_point['joint_positions']) + joint_diff
        real_point['joint_positions'] = real_joints.tolist()

        # Add additional noise to make it more realistic
        real_point['position'][0] += np.random.normal(0, 0.01)
        real_point['position'][1] += np.random.normal(0, 0.01)
        real_point['position'][2] += np.random.normal(0, 0.008)

        real_trajectory.append(real_point)

    return sim_trajectory, real_trajectory


def main():
    print("Running Sim-to-Real Transfer Evaluation")
    print("="*50)

    # Create output directory
    output_dir = Path("evaluation_outputs")
    output_dir.mkdir(exist_ok=True)

    # Load configuration
    config_path = Path("config/validation_config.yaml")
    if not config_path.exists():
        print(f"Configuration file not found at {config_path}")
        print("Using default configuration...")
        config = None
    else:
        config = load_config(config_path)

    # Initialize evaluator
    evaluator = SimToRealEvaluator(config_path=str(config_path) if config_path.exists() else None)

    print("Generating sample simulation and real robot trajectories...")
    sim_traj, real_traj = generate_sample_data()

    print(f"Generated {len(sim_traj)} simulation points and {len(real_traj)} real robot points")

    # Create trajectory pair
    trajectory_pair = TrajectoryPair(
        sim_trajectory=sim_traj,
        real_trajectory=real_traj,
        task_description="Circular movement task for sim-to-real validation",
        timestamp=datetime.now().isoformat()
    )

    print("\nEvaluating trajectory similarity...")
    # Evaluate trajectory similarity
    traj_metrics = evaluator.evaluate_trajectory_similarity(sim_traj, real_traj)

    if traj_metrics["valid"]:
        print(f"✓ Position accuracy: {traj_metrics['position']['rmse']:.3f}m RMSE")
        print(f"✓ Orientation accuracy: {traj_metrics['orientation']['rmse']:.3f}rad RMSE")
        print(f"✓ Trajectory correlation: {traj_metrics['similarity']['correlation']:.3f}")
        print(f"✓ Success rate: {traj_metrics['position']['success_rate']:.2f}")
    else:
        print(f"✗ Error in trajectory evaluation: {traj_metrics.get('error', 'Unknown error')}")

    print("\nEvaluating control fidelity...")
    # Create sample control data
    sim_controls = []
    real_controls = []

    for i in range(min(len(sim_traj), len(real_traj))):
        sim_ctrl = {
            'position': sim_traj[i]['joint_positions'][:10],  # First 10 joints
            'velocity': [float(np.cos(sim_traj[i]['timestamp'] + j*0.1)) for j in range(10)],
            'effort': [float(np.sin(sim_traj[i]['timestamp'] + j*0.2)) for j in range(10)]
        }

        real_ctrl = {
            'position': real_traj[i]['joint_positions'][:10],  # First 10 joints
            'velocity': [float(np.cos(real_traj[i]['timestamp'] + j*0.1) * 0.95) for j in range(10)],  # Slightly different
            'effort': [float(np.sin(real_traj[i]['timestamp'] + j*0.2) * 1.05) for j in range(10)]   # Slightly different
        }

        sim_controls.append(sim_ctrl)
        real_controls.append(real_ctrl)

    # Evaluate control fidelity
    ctrl_metrics = evaluator.evaluate_control_fidelity(sim_controls, real_controls)

    if ctrl_metrics["valid"]:
        print(f"✓ Position fidelity: {ctrl_metrics['position_fidelity']['avg_error']:.3f} rad")
        print(f"✓ Velocity fidelity: {ctrl_metrics['velocity_fidelity']['avg_error']:.3f} (normalized)")
        print(f"✓ Effort fidelity: {ctrl_metrics['effort_fidelity']['avg_error']:.3f} (normalized)")
    else:
        print(f"✗ Error in control evaluation: {ctrl_metrics.get('error', 'Unknown error')}")

    print("\nGenerating comprehensive evaluation report...")
    # Generate evaluation report
    report = evaluator.generate_evaluation_report([trajectory_pair],
                                                 output_path=output_dir / "evaluation_report.json")

    print(f"Transfer quality assessment: {report['transfer_assessment']['quality']}")
    print(f"Transfer score: {report['transfer_assessment']['score']:.2f}")
    print(f"Average position error: {report['aggregate_metrics']['position_errors']['mean']:.3f}m")
    print(f"Average correlation: {report['aggregate_metrics']['correlations']['mean']:.3f}")

    print("\nCreating visualizations...")
    # Create visualizations
    evaluator.visualize_evaluation_results(report,
                                          output_path=output_dir / "evaluation_visualizations.png")

    print("\nSaving detailed evaluation data...")
    # Save detailed evaluation data
    evaluator.save_evaluation_data([trajectory_pair],
                                   output_dir / "detailed_evaluation_data")

    # Print summary
    print("\n" + "="*50)
    print("EVALUATION SUMMARY")
    print("="*50)
    print(f"• Position RMSE: {report['aggregate_metrics']['position_errors']['mean']:.3f}m")
    print(f"• Orientation RMSE: {report['aggregate_metrics']['orientation_errors']['mean']:.3f}rad")
    print(f"• Trajectory Correlation: {report['aggregate_metrics']['correlations']['mean']:.3f}")
    print(f"• Transfer Quality: {report['transfer_assessment']['quality']}")
    print(f"• Transfer Score: {report['transfer_assessment']['score']:.2f}")

    # Assessment based on thresholds
    pos_threshold = 0.05  # 5cm threshold
    corr_threshold = 0.7  # 70% correlation threshold

    pos_good = report['aggregate_metrics']['position_errors']['mean'] < pos_threshold
    corr_good = report['aggregate_metrics']['correlations']['mean'] > corr_threshold

    if pos_good and corr_good:
        assessment = "POSITIVE - Good sim-to-real transfer quality"
    elif pos_good or corr_good:
        assessment = "NEUTRAL - Moderate sim-to-real transfer quality"
    else:
        assessment = "NEGATIVE - Poor sim-to-real transfer quality"

    print(f"• Overall Assessment: {assessment}")

    print(f"\nDetailed results saved to: {output_dir}/")
    print("Files created:")
    print("- evaluation_report.json")
    print("- evaluation_visualizations.png")
    print("- detailed_evaluation_data/trajectory_comparison.csv")

    return report


if __name__ == "__main__":
    report = main()
    print("\nSim-to-Real evaluation completed successfully!")