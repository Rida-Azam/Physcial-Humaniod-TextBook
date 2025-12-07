#!/usr/bin/env python3

"""
Sim-to-Real Validation and Evaluation Framework

This module provides tools for evaluating sim-to-real transfer performance,
validating policies across simulation and real robot platforms, and measuring
the fidelity of simulation environments.
"""

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.spatial.distance import euclidean
from scipy.spatial.transform import Rotation as R
import json
import yaml
import csv
import os
from datetime import datetime
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional, Any
import statistics
import logging


@dataclass
class TrajectoryPair:
    """Data class to hold paired simulation and real trajectories"""
    sim_trajectory: List[Dict]
    real_trajectory: List[Dict]
    task_description: str
    timestamp: str


class SimToRealEvaluator:
    """
    Main class for evaluating sim-to-real transfer performance
    """

    def __init__(self, config_path: str = None):
        self.config = self._load_config(config_path)
        self.evaluation_results = []
        self.logger = self._setup_logger()

    def _load_config(self, config_path: str) -> Dict:
        """Load evaluation configuration"""
        if config_path and os.path.exists(config_path):
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        else:
            # Default configuration
            return {
                "evaluation_metrics": {
                    "trajectory_accuracy": {
                        "position_threshold": 0.05,  # 5cm threshold
                        "orientation_threshold": 0.1,  # 0.1 rad threshold
                        "timing_threshold": 0.1  # 100ms threshold
                    },
                    "control_fidelity": {
                        "joint_position_threshold": 0.02,  # 2 degrees
                        "velocity_threshold": 0.05,  # 5% velocity difference
                        "effort_threshold": 0.1  # 10% effort difference
                    },
                    "behavior_similarity": {
                        "dtw_weight": 0.4,  # Dynamic Time Warping weight
                        "mse_weight": 0.3,  # Mean Square Error weight
                        "correlation_weight": 0.3  # Cross-correlation weight
                    }
                },
                "validation_thresholds": {
                    "minimum_success_rate": 0.7,  # 70% success rate
                    "maximum_divergence": 0.2,  # 20% maximum divergence
                    "minimum_correlation": 0.7  # 70% minimum correlation
                }
            }

    def _setup_logger(self) -> logging.Logger:
        """Set up logger for evaluation"""
        logger = logging.getLogger("SimToRealEvaluator")
        logger.setLevel(logging.INFO)

        handler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        logger.addHandler(handler)

        return logger

    def evaluate_trajectory_similarity(self, sim_traj: List[Dict], real_traj: List[Dict]) -> Dict:
        """
        Evaluate similarity between simulation and real trajectories

        Args:
            sim_traj: Simulation trajectory data
            real_traj: Real robot trajectory data

        Returns:
            Dictionary containing similarity metrics
        """
        if len(sim_traj) == 0 or len(real_traj) == 0:
            return {
                "valid": False,
                "error": "Empty trajectory provided"
            }

        # Ensure trajectories have same length (interpolate if needed)
        if len(sim_traj) != len(real_traj):
            # Interpolate shorter trajectory to match length
            if len(sim_traj) < len(real_traj):
                sim_traj = self._interpolate_trajectory(sim_traj, len(real_traj))
            else:
                real_traj = self._interpolate_trajectory(real_traj, len(sim_traj))

        # Extract relevant data
        sim_positions = np.array([point.get('position', [0, 0, 0]) for point in sim_traj])
        real_positions = np.array([point.get('position', [0, 0, 0]) for point in real_traj])

        sim_orientations = np.array([point.get('orientation', [0, 0, 0, 1]) for point in sim_traj])
        real_orientations = np.array([point.get('orientation', [0, 0, 0, 1]) for point in real_traj])

        sim_joint_positions = np.array([point.get('joint_positions', []) for point in sim_traj])
        real_joint_positions = np.array([point.get('joint_positions', []) for point in real_traj])

        # Calculate position error
        position_errors = [euclidean(sim_pos, real_pos) for sim_pos, real_pos in zip(sim_positions, real_positions)]
        avg_position_error = np.mean(position_errors)
        max_position_error = np.max(position_errors)

        # Calculate orientation error
        orientation_errors = []
        for sim_q, real_q in zip(sim_orientations, real_orientations):
            try:
                # Convert quaternions to rotation objects
                sim_rot = R.from_quat(sim_q)
                real_rot = R.from_quat(real_q)

                # Calculate rotation difference
                diff_rot = sim_rot.inv() * real_rot
                angle_diff = diff_rot.magnitude()  # Angle in radians
                orientation_errors.append(angle_diff)
            except:
                # If conversion fails, use a simple difference
                orientation_errors.append(1.0)  # Default to large error

        avg_orientation_error = np.mean(orientation_errors)
        max_orientation_error = np.max(orientation_errors)

        # Calculate joint position error (if available)
        joint_errors = []
        min_len = min(len(sim_joint_positions), len(real_joint_positions))

        for i in range(min_len):
            if len(sim_joint_positions[i]) > 0 and len(real_joint_positions[i]) > 0:
                # Ensure same length
                min_joints = min(len(sim_joint_positions[i]), len(real_joint_positions[i]))
                sim_joints = sim_joint_positions[i][:min_joints]
                real_joints = real_joint_positions[i][:min_joints]

                joint_diff = np.mean(np.abs(np.array(sim_joints) - np.array(real_joints)))
                joint_errors.append(joint_diff)

        avg_joint_error = np.mean(joint_errors) if joint_errors else float('inf')

        # Calculate Dynamic Time Warping (DTW) distance if available
        try:
            from dtaidistance import dtw
            dtw_distance = dtw.distance(sim_positions.flatten(), real_positions.flatten())
        except ImportError:
            # If DTW library not available, use simple Euclidean distance
            dtw_distance = np.mean(position_errors)

        # Calculate correlation between trajectories
        if len(sim_positions) > 1 and len(real_positions) > 1:
            # Flatten the trajectories to calculate correlation
            flat_sim = sim_positions.flatten()
            flat_real = real_positions.flatten()

            # Calculate Pearson correlation
            if np.std(flat_sim) > 0 and np.std(flat_real) > 0:
                correlation = np.corrcoef(flat_sim, flat_real)[0, 1]
            else:
                correlation = 0.0
        else:
            correlation = 0.0

        # Calculate success rate based on thresholds
        position_success = np.mean([err < self.config["evaluation_metrics"]["trajectory_accuracy"]["position_threshold"]
                                   for err in position_errors])
        orientation_success = np.mean([err < self.config["evaluation_metrics"]["trajectory_accuracy"]["orientation_threshold"]
                                     for err in orientation_errors])

        # Combine metrics
        metrics = {
            "valid": True,
            "position": {
                "avg_error": float(avg_position_error),
                "max_error": float(max_position_error),
                "rmse": float(np.sqrt(np.mean(np.square(position_errors)))),
                "success_rate": float(position_success)
            },
            "orientation": {
                "avg_error": float(avg_orientation_error),
                "max_error": float(max_orientation_error),
                "success_rate": float(orientation_success)
            },
            "joints": {
                "avg_error": float(avg_joint_error) if not np.isnan(avg_joint_error) else float('inf')
            },
            "similarity": {
                "dtw_distance": float(dtw_distance),
                "correlation": float(correlation),
                "mse": float(np.mean(np.square(position_errors)))
            },
            "trajectory_length": len(sim_traj),
            "data_points": {
                "sim": len(sim_traj),
                "real": len(real_traj)
            }
        }

        return metrics

    def _interpolate_trajectory(self, traj: List[Dict], target_length: int) -> List[Dict]:
        """
        Interpolate trajectory to match target length
        """
        if len(traj) >= target_length:
            return traj

        interpolated_traj = []
        step_size = (len(traj) - 1) / (target_length - 1)

        for i in range(target_length):
            float_idx = i * step_size
            lower_idx = int(np.floor(float_idx))
            upper_idx = int(np.ceil(float_idx))

            if lower_idx == upper_idx:
                # Exact match
                interpolated_traj.append(traj[lower_idx])
            else:
                # Interpolate between two points
                alpha = float_idx - lower_idx

                lower_point = traj[lower_idx]
                upper_point = traj[upper_idx]

                # Interpolate position
                lower_pos = np.array(lower_point.get('position', [0, 0, 0]))
                upper_pos = np.array(upper_point.get('position', [0, 0, 0]))
                interp_pos = (1 - alpha) * lower_pos + alpha * upper_pos

                # Interpolate orientation (SLERP for quaternions)
                lower_quat = np.array(lower_point.get('orientation', [0, 0, 0, 1]))
                upper_quat = np.array(upper_point.get('orientation', [0, 0, 0, 1]))

                # Simple linear interpolation for quaternions (not ideal, but works for small differences)
                interp_quat = (1 - alpha) * lower_quat + alpha * upper_quat
                interp_quat = interp_quat / np.linalg.norm(interp_quat)  # Normalize

                # Create interpolated point
                interp_point = {
                    'position': interp_pos.tolist(),
                    'orientation': interp_quat.tolist(),
                    'timestamp': float(i) * 0.1  # Assume 100ms intervals
                }

                # Copy other fields if they exist
                for key in lower_point:
                    if key not in ['position', 'orientation', 'timestamp']:
                        if key in upper_point:
                            # Interpolate numeric values, otherwise take lower
                            lower_val = lower_point[key]
                            upper_val = upper_point[key]

                            if isinstance(lower_val, (int, float)) and isinstance(upper_val, (int, float)):
                                interp_val = (1 - alpha) * lower_val + alpha * upper_val
                                interp_point[key] = interp_val
                            else:
                                interp_point[key] = lower_val
                        else:
                            interp_point[key] = lower_val

                interpolated_traj.append(interp_point)

        return interpolated_traj

    def evaluate_control_fidelity(self, sim_controls: List[Dict], real_controls: List[Dict]) -> Dict:
        """
        Evaluate control fidelity between simulation and real robot

        Args:
            sim_controls: Control commands from simulation
            real_controls: Control commands from real robot

        Returns:
            Dictionary containing control fidelity metrics
        """
        if len(sim_controls) == 0 or len(real_controls) == 0:
            return {
                "valid": False,
                "error": "Empty control data provided"
            }

        # Ensure same length
        if len(sim_controls) != len(real_controls):
            if len(sim_controls) < len(real_controls):
                sim_controls = self._interpolate_control_sequence(sim_controls, len(real_controls))
            else:
                real_controls = self._interpolate_control_sequence(real_controls, len(sim_controls))

        # Extract control data
        sim_positions = np.array([ctrl.get('position', []) for ctrl in sim_controls])
        real_positions = np.array([ctrl.get('position', []) for ctrl in real_controls])

        sim_velocities = np.array([ctrl.get('velocity', []) for ctrl in sim_controls])
        real_velocities = np.array([ctrl.get('velocity', []) for ctrl in real_controls])

        sim_efforts = np.array([ctrl.get('effort', []) for ctrl in sim_controls])
        real_efforts = np.array([ctrl.get('effort', []) for ctrl in real_controls])

        # Calculate position fidelity
        position_errors = []
        for sim_pos, real_pos in zip(sim_positions, real_positions):
            if len(sim_pos) > 0 and len(real_pos) > 0:
                min_len = min(len(sim_pos), len(real_pos))
                pos_diff = np.mean(np.abs(np.array(sim_pos[:min_len]) - np.array(real_pos[:min_len])))
                position_errors.append(pos_diff)

        avg_position_fidelity = np.mean(position_errors) if position_errors else float('inf')

        # Calculate velocity fidelity
        velocity_errors = []
        for sim_vel, real_vel in zip(sim_velocities, real_velocities):
            if len(sim_vel) > 0 and len(real_vel) > 0:
                min_len = min(len(sim_vel), len(real_vel))
                vel_diff = np.mean(np.abs(np.array(sim_vel[:min_len]) - np.array(real_vel[:min_len])))
                velocity_errors.append(vel_diff)

        avg_velocity_fidelity = np.mean(velocity_errors) if velocity_errors else float('inf')

        # Calculate effort fidelity
        effort_errors = []
        for sim_eff, real_eff in zip(sim_efforts, real_efforts):
            if len(sim_eff) > 0 and len(real_eff) > 0:
                min_len = min(len(sim_eff), len(real_eff))
                eff_diff = np.mean(np.abs(np.array(sim_eff[:min_len]) - np.array(real_eff[:min_len])))
                effort_errors.append(eff_diff)

        avg_effort_fidelity = np.mean(effort_errors) if effort_errors else float('inf')

        # Calculate overall control similarity
        overall_similarity = 1.0 / (1.0 + avg_position_fidelity + avg_velocity_fidelity + avg_effort_fidelity)

        return {
            "valid": True,
            "position_fidelity": {
                "avg_error": float(avg_position_fidelity),
                "threshold": self.config["evaluation_metrics"]["control_fidelity"]["joint_position_threshold"]
            },
            "velocity_fidelity": {
                "avg_error": float(avg_velocity_fidelity),
                "threshold": self.config["evaluation_metrics"]["control_fidelity"]["velocity_threshold"]
            },
            "effort_fidelity": {
                "avg_error": float(avg_effort_fidelity),
                "threshold": self.config["evaluation_metrics"]["control_fidelity"]["effort_threshold"]
            },
            "overall_similarity": float(overall_similarity),
            "data_points": {
                "sim": len(sim_controls),
                "real": len(real_controls)
            }
        }

    def _interpolate_control_sequence(self, controls: List[Dict], target_length: int) -> List[Dict]:
        """
        Interpolate control sequence to match target length
        """
        if len(controls) >= target_length:
            return controls

        interpolated_controls = []
        step_size = (len(controls) - 1) / (target_length - 1)

        for i in range(target_length):
            float_idx = i * step_size
            lower_idx = int(np.floor(float_idx))
            upper_idx = int(np.ceil(float_idx))

            if lower_idx == upper_idx:
                interpolated_controls.append(controls[lower_idx])
            else:
                # Interpolate control values
                alpha = float_idx - lower_idx
                lower_ctrl = controls[lower_idx]
                upper_ctrl = controls[upper_idx]

                interp_ctrl = {}

                # Interpolate position
                if 'position' in lower_ctrl and 'position' in upper_ctrl:
                    lower_pos = np.array(lower_ctrl['position'])
                    upper_pos = np.array(upper_ctrl['position'])
                    interp_pos = (1 - alpha) * lower_pos + alpha * upper_pos
                    interp_ctrl['position'] = interp_pos.tolist()

                # Interpolate velocity
                if 'velocity' in lower_ctrl and 'velocity' in upper_ctrl:
                    lower_vel = np.array(lower_ctrl['velocity'])
                    upper_vel = np.array(upper_ctrl['velocity'])
                    interp_vel = (1 - alpha) * lower_vel + alpha * upper_vel
                    interp_ctrl['velocity'] = interp_vel.tolist()

                # Interpolate effort
                if 'effort' in lower_ctrl and 'effort' in upper_ctrl:
                    lower_eff = np.array(lower_ctrl['effort'])
                    upper_eff = np.array(upper_ctrl['effort'])
                    interp_eff = (1 - alpha) * lower_eff + alpha * upper_eff
                    interp_ctrl['effort'] = interp_eff.tolist()

                # Copy other fields
                for key in lower_ctrl:
                    if key not in ['position', 'velocity', 'effort']:
                        interp_ctrl[key] = lower_ctrl[key]

                interpolated_controls.append(interp_ctrl)

        return interpolated_controls

    def evaluate_task_success(self, sim_results: Dict, real_results: Dict, task_thresholds: Dict) -> Dict:
        """
        Evaluate task-level success between simulation and real robot

        Args:
            sim_results: Results from simulation task execution
            real_results: Results from real robot task execution
            task_thresholds: Task-specific success thresholds

        Returns:
            Dictionary containing task success evaluation
        """
        # Extract task metrics
        sim_metrics = sim_results.get('metrics', {})
        real_metrics = real_results.get('metrics', {})

        # Calculate success for each metric
        metric_success = {}
        for metric_name, threshold in task_thresholds.items():
            sim_value = sim_metrics.get(metric_name, 0)
            real_value = real_metrics.get(metric_name, 0)

            # For most metrics, closer to real value indicates better sim-to-real transfer
            if isinstance(threshold, dict):
                # Threshold with operator (e.g., {'operator': 'less_than', 'value': 0.1})
                op = threshold.get('operator', 'difference_less_than')
                thresh_val = threshold.get('value', 0)

                if op == 'difference_less_than':
                    diff = abs(sim_value - real_value)
                    success = diff <= thresh_val
                elif op == 'sim_greater_than_real_by_at_least':
                    diff = sim_value - real_value
                    success = diff >= thresh_val
                elif op == 'both_above_threshold':
                    success = sim_value >= thresh_val and real_value >= thresh_val
                else:
                    # Default to difference check
                    diff = abs(sim_value - real_value)
                    success = diff <= thresh_val
            else:
                # Simple difference threshold
                diff = abs(sim_value - real_value)
                success = diff <= threshold

            metric_success[metric_name] = {
                "sim_value": sim_value,
                "real_value": real_value,
                "difference": abs(sim_value - real_value),
                "threshold": threshold,
                "success": success
            }

        # Calculate overall success rate
        successful_metrics = sum(1 for m in metric_success.values() if m['success'])
        total_metrics = len(metric_success)
        overall_success_rate = successful_metrics / total_metrics if total_metrics > 0 else 0

        return {
            "metric_success": metric_success,
            "overall_success_rate": overall_success_rate,
            "successful_metrics": successful_metrics,
            "total_metrics": total_metrics,
            "transfer_quality": "high" if overall_success_rate >= 0.8 else "medium" if overall_success_rate >= 0.6 else "low"
        }

    def generate_evaluation_report(self, evaluation_pairs: List[TrajectoryPair], output_path: str = None) -> Dict:
        """
        Generate comprehensive evaluation report

        Args:
            evaluation_pairs: List of trajectory pairs to evaluate
            output_path: Path to save the report (optional)

        Returns:
            Dictionary containing evaluation report
        """
        self.logger.info(f"Generating evaluation report for {len(evaluation_pairs)} trajectory pairs...")

        report = {
            "timestamp": datetime.now().isoformat(),
            "evaluation_pairs_count": len(evaluation_pairs),
            "individual_evaluations": [],
            "aggregate_metrics": {},
            "transfer_assessment": {}
        }

        all_position_errors = []
        all_orientation_errors = []
        all_joint_errors = []
        all_correlations = []

        for i, pair in enumerate(evaluation_pairs):
            self.logger.info(f"Evaluating pair {i+1}/{len(evaluation_pairs)}: {pair.task_description}")

            # Evaluate trajectory similarity
            traj_metrics = self.evaluate_trajectory_similarity(pair.sim_trajectory, pair.real_trajectory)

            if traj_metrics["valid"]:
                all_position_errors.append(traj_metrics["position"]["avg_error"])
                all_orientation_errors.append(traj_metrics["orientation"]["avg_error"])
                if not np.isinf(traj_metrics["joints"]["avg_error"]):
                    all_joint_errors.append(traj_metrics["joints"]["avg_error"])
                all_correlations.append(traj_metrics["similarity"]["correlation"])

            evaluation_entry = {
                "pair_index": i,
                "task_description": pair.task_description,
                "timestamp": pair.timestamp,
                "trajectory_metrics": traj_metrics
            }

            report["individual_evaluations"].append(evaluation_entry)

        # Calculate aggregate metrics
        if all_position_errors:
            report["aggregate_metrics"] = {
                "position_errors": {
                    "mean": float(np.mean(all_position_errors)),
                    "std": float(np.std(all_position_errors)),
                    "median": float(np.median(all_position_errors)),
                    "min": float(np.min(all_position_errors)),
                    "max": float(np.max(all_position_errors)),
                    "p95": float(np.percentile(all_position_errors, 95))
                },
                "orientation_errors": {
                    "mean": float(np.mean(all_orientation_errors)),
                    "std": float(np.std(all_orientation_errors)),
                    "median": float(np.median(all_orientation_errors)),
                    "min": float(np.min(all_orientation_errors)),
                    "max": float(np.max(all_orientation_errors))
                },
                "joint_errors": {
                    "mean": float(np.mean(all_joint_errors)) if all_joint_errors else float('inf'),
                    "std": float(np.std(all_joint_errors)) if len(all_joint_errors) > 1 else 0.0,
                    "median": float(np.median(all_joint_errors)) if all_joint_errors else float('inf'),
                    "min": float(np.min(all_joint_errors)) if all_joint_errors else float('inf'),
                    "max": float(np.max(all_joint_errors)) if all_joint_errors else float('inf')
                },
                "correlations": {
                    "mean": float(np.mean(all_correlations)) if all_correlations else 0.0,
                    "std": float(np.std(all_correlations)) if len(all_correlations) > 1 else 0.0,
                    "min": float(np.min(all_correlations)) if all_correlations else 0.0,
                    "max": float(np.max(all_correlations)) if all_correlations else 0.0
                }
            }

        # Assess transfer quality
        avg_position_error = report["aggregate_metrics"].get("position_errors", {}).get("mean", float('inf'))
        avg_correlation = report["aggregate_metrics"].get("correlations", {}).get("mean", 0.0)

        position_good = avg_position_error < self.config["evaluation_metrics"]["trajectory_accuracy"]["position_threshold"]
        correlation_good = avg_correlation > 0.7  # Good correlation threshold

        if position_good and correlation_good:
            transfer_quality = "excellent"
            transfer_score = 4.0
        elif position_good or correlation_good:
            transfer_quality = "good"
            transfer_score = 3.0
        else:
            transfer_quality = "needs_improvement"
            transfer_score = 2.0

        report["transfer_assessment"] = {
            "quality": transfer_quality,
            "score": transfer_score,
            "avg_position_error": avg_position_error,
            "avg_correlation": avg_correlation,
            "thresholds_met": {
                "position_accuracy": position_good,
                "correlation": correlation_good
            }
        }

        # Save report if path provided
        if output_path:
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            with open(output_path, 'w') as f:
                json.dump(report, f, indent=2)

            self.logger.info(f"Evaluation report saved to {output_path}")

        return report

    def visualize_evaluation_results(self, report: Dict, output_path: str = None):
        """
        Create visualizations of evaluation results

        Args:
            report: Evaluation report dictionary
            output_path: Path to save visualizations (optional)
        """
        self.logger.info("Creating evaluation visualizations...")

        # Create figure with subplots
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle('Sim-to-Real Evaluation Results', fontsize=16)

        # 1. Position Error Distribution
        if "aggregate_metrics" in report and "position_errors" in report["aggregate_metrics"]:
            pos_errors = report["aggregate_metrics"]["position_errors"]
            axes[0, 0].hist(all_position_errors, bins=30, density=True, alpha=0.7, color='skyblue', edgecolor='black')
            axes[0, 0].axvline(pos_errors["mean"], color='red', linestyle='--', label=f'Mean: {pos_errors["mean"]:.3f}m')
            axes[0, 0].set_xlabel('Position Error (m)')
            axes[0, 0].set_ylabel('Density')
            axes[0, 0].set_title('Distribution of Position Errors')
            axes[0, 0].legend()
            axes[0, 0].grid(True, alpha=0.3)

        # 2. Correlation Distribution
        if "aggregate_metrics" in report and "correlations" in report["aggregate_metrics"]:
            corr_data = report["aggregate_metrics"]["correlations"]
            axes[0, 1].hist(all_correlations, bins=30, density=True, alpha=0.7, color='lightgreen', edgecolor='black')
            axes[0, 1].axvline(corr_data["mean"], color='red', linestyle='--', label=f'Mean: {corr_data["mean"]:.3f}')
            axes[0, 1].set_xlabel('Correlation Coefficient')
            axes[0, 1].set_ylabel('Density')
            axes[0, 1].set_title('Distribution of Trajectory Correlations')
            axes[0, 1].set_xlim(0, 1)
            axes[0, 1].legend()
            axes[0, 1].grid(True, alpha=0.3)

        # 3. Task Success Rates by Metric Category
        if "individual_evaluations" in report:
            success_rates = []
            task_labels = []

            for eval_data in report["individual_evaluations"][:10]:  # Just first 10 for readability
                if eval_data["trajectory_metrics"]["valid"]:
                    pos_success = eval_data["trajectory_metrics"]["position"]["success_rate"]
                    success_rates.append(pos_success)
                    task_desc = eval_data["task_description"][:20]  # Truncate for readability
                    task_labels.append(f"{task_desc}...")

            if success_rates:
                bars = axes[1, 0].bar(range(len(success_rates)), success_rates, color='coral', alpha=0.7)
                axes[1, 0].set_xlabel('Task')
                axes[1, 0].set_ylabel('Success Rate')
                axes[1, 0].set_title('Position Success Rates by Task')
                axes[1, 0].set_xticks(range(len(task_labels)))
                axes[1, 0].set_xticklabels(task_labels, rotation=45, ha='right')
                axes[1, 0].set_ylim(0, 1)
                axes[1, 0].grid(True, alpha=0.3)

                # Add value labels on bars
                for bar, rate in zip(bars, success_rates):
                    axes[1, 0].text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                                   f'{rate:.2f}', ha='center', va='bottom')

        # 4. Scatter plot of Sim vs Real Values (if we have multiple evaluations)
        if "individual_evaluations" in report:
            sim_values = []
            real_values = []

            for eval_data in report["individual_evaluations"]:
                if eval_data["trajectory_metrics"]["valid"]:
                    sim_val = eval_data["trajectory_metrics"]["position"]["avg_error"]
                    # For this example, we'll use a proxy for real value
                    # In practice, you'd have real robot data
                    real_val = sim_val * np.random.uniform(0.8, 1.2)  # Add some variance
                    sim_values.append(sim_val)
                    real_values.append(real_val)

            if sim_values and real_values:
                axes[1, 1].scatter(sim_values, real_values, alpha=0.6, color='purple')
                # Add identity line (perfect sim-to-real transfer)
                min_val = min(min(sim_values), min(real_values))
                max_val = max(max(sim_values), max(real_values))
                axes[1, 1].plot([min_val, max_val], [min_val, max_val], 'r--', label='Perfect Transfer')
                axes[1, 1].set_xlabel('Simulation Values')
                axes[1, 1].set_ylabel('Real Robot Values')
                axes[1, 1].set_title('Simulation vs Real Robot Values')
                axes[1, 1].legend()
                axes[1, 1].grid(True, alpha=0.3)

        plt.tight_layout()

        if output_path:
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            self.logger.info(f"Visualizations saved to {output_path}")

        plt.show()

    def save_evaluation_data(self, evaluation_pairs: List[TrajectoryPair], output_dir: str):
        """
        Save evaluation data to CSV files for further analysis

        Args:
            evaluation_pairs: List of trajectory pairs
            output_dir: Directory to save the data
        """
        os.makedirs(output_dir, exist_ok=True)

        # Save trajectory comparison data
        with open(os.path.join(output_dir, "trajectory_comparison.csv"), "w", newline='') as csvfile:
            fieldnames = ['pair_index', 'task_description', 'sim_step', 'real_step',
                         'sim_x', 'sim_y', 'sim_z', 'real_x', 'real_y', 'real_z',
                         'position_error', 'timestamp']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writeheader()
            for i, pair in enumerate(evaluation_pairs):
                min_len = min(len(pair.sim_trajectory), len(pair.real_trajectory))
                for j in range(min_len):
                    sim_point = pair.sim_trajectory[j]
                    real_point = pair.real_trajectory[j]

                    sim_pos = sim_point.get('position', [0, 0, 0])
                    real_pos = real_point.get('position', [0, 0, 0])

                    pos_error = euclidean(sim_pos, real_pos)

                    writer.writerow({
                        'pair_index': i,
                        'task_description': pair.task_description,
                        'sim_step': j,
                        'real_step': j,
                        'sim_x': sim_pos[0],
                        'sim_y': sim_pos[1],
                        'sim_z': sim_pos[2],
                        'real_x': real_pos[0],
                        'real_y': real_pos[1],
                        'real_z': real_pos[2],
                        'position_error': pos_error,
                        'timestamp': pair.timestamp
                    })

        self.logger.info(f"Evaluation data saved to {output_dir}")


def main():
    """
    Main function demonstrating the sim-to-real evaluation framework
    """
    print("Sim-to-Real Evaluation Framework")
    print("="*50)

    # Initialize evaluator
    evaluator = SimToRealEvaluator()

    # Create sample evaluation data
    print("Creating sample trajectory data...")

    # Generate sample simulation trajectory
    sim_trajectory = []
    for i in range(50):
        # Create a simple trajectory moving in a curve
        t = i * 0.1  # Time parameter
        x = 0.5 * np.cos(t)
        y = 0.5 * np.sin(t)
        z = 0.8 + 0.1 * np.sin(2*t)  # Slight height variation

        # Add some noise to make it more realistic
        x += np.random.normal(0, 0.01)
        y += np.random.normal(0, 0.01)
        z += np.random.normal(0, 0.005)

        point = {
            'position': [float(x), float(y), float(z)],
            'orientation': [0.0, 0.0, np.sin(t/2), np.cos(t/2)],  # Simple orientation change
            'joint_positions': [float(np.sin(t + j*0.5)) for j in range(12)],  # 12 joints example
            'timestamp': float(i * 0.1)
        }
        sim_trajectory.append(point)

    # Generate sample real trajectory (similar but with systematic differences)
    real_trajectory = []
    for i, sim_point in enumerate(sim_trajectory):
        # Add systematic differences between sim and real
        real_point = sim_point.copy()

        # Add offsets to simulate sim-to-real gap
        pos_offset = np.random.normal(0, 0.02, 3)  # 2cm average offset
        real_pos = np.array(sim_point['position']) + pos_offset
        real_point['position'] = real_pos.tolist()

        # Add differences to joint positions
        joint_diff = np.random.normal(0, 0.01, len(sim_point['joint_positions']))
        real_joints = np.array(sim_point['joint_positions']) + joint_diff
        real_point['joint_positions'] = real_joints.tolist()

        real_trajectory.append(real_point)

    # Create trajectory pair
    trajectory_pair = TrajectoryPair(
        sim_trajectory=sim_trajectory,
        real_trajectory=real_trajectory,
        task_description="Circular movement trajectory comparison",
        timestamp=datetime.now().isoformat()
    )

    # Evaluate the trajectory pair
    print("Evaluating trajectory similarity...")
    metrics = evaluator.evaluate_trajectory_similarity(sim_trajectory, real_trajectory)

    print(f"Average position error: {metrics['position']['avg_error']:.3f}m")
    print(f"Average orientation error: {metrics['orientation']['avg_error']:.3f}rad")
    print(f"Trajectory correlation: {metrics['similarity']['correlation']:.3f}")
    print(f"Position success rate: {metrics['position']['success_rate']:.2f}")

    # Generate full evaluation report
    print("\nGenerating evaluation report...")
    report = evaluator.generate_evaluation_report([trajectory_pair], "evaluation_report.json")

    print(f"Transfer quality: {report['transfer_assessment']['quality']}")
    print(f"Transfer score: {report['transfer_assessment']['score']}")
    print(f"Average position error: {report['aggregate_metrics']['position_errors']['mean']:.3f}m")

    # Create visualizations
    print("\nCreating visualizations...")
    evaluator.visualize_evaluation_results(report, "evaluation_visualizations.png")

    # Save detailed evaluation data
    print("Saving detailed evaluation data...")
    evaluator.save_evaluation_data([trajectory_pair], "evaluation_data/")

    print("\nSim-to-Real evaluation completed successfully!")
    print("Files created:")
    print("- evaluation_report.json")
    print("- evaluation_visualizations.png")
    print("- evaluation_data/trajectory_comparison.csv")


if __name__ == "__main__":
    main()