#!/usr/bin/env python3

"""
Isaac Sim Validation Framework

This module provides tools for validating Isaac Sim environments
against real-world conditions and optimizing simulation performance.
"""

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.spatial.transform import Rotation as R
from scipy import stats
import json
import yaml
import time
from datetime import datetime
import os
import pandas as pd
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
import logging


@dataclass
class ValidationResult:
    """Data class for storing validation results"""
    test_name: str
    passed: bool
    score: float
    threshold: float
    details: Dict


class IsaacSimValidator:
    """Main validation class for Isaac Sim environments"""

    def __init__(self, config_path: str = None):
        self.config = self._load_config(config_path)
        self.results = []
        self.logger = self._setup_logger()

    def _load_config(self, config_path: str) -> Dict:
        """Load validation configuration"""
        if config_path and os.path.exists(config_path):
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        else:
            # Default configuration
            return {
                "validation_thresholds": {
                    "kinematic_accuracy": 0.01,      # 1 cm
                    "dynamic_accuracy": 0.05,        # 5 cm/s
                    "balance_stability": 0.1,        # 0.1 rad
                    "locomotion_success": 0.9,       # 90%
                    "manipulation_success": 0.85     # 85%
                },
                "performance_targets": {
                    "min_fps": 30,
                    "max_physics_time": 0.01,        # 10 ms
                    "max_rendering_time": 0.02       # 20 ms
                }
            }

    def _setup_logger(self) -> logging.Logger:
        """Set up logger for validation"""
        logger = logging.getLogger("IsaacSimValidator")
        logger.setLevel(logging.INFO)

        handler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        logger.addHandler(handler)

        return logger

    def validate_kinematics(self, sim_data: Dict, real_data: Dict) -> ValidationResult:
        """Validate kinematic accuracy between simulation and real robot"""
        # Extract joint positions
        sim_joints = np.array(sim_data.get("joint_positions", []))
        real_joints = np.array(real_data.get("joint_positions", []))

        if len(sim_joints) == 0 or len(real_joints) == 0:
            return ValidationResult(
                test_name="kinematic_validation",
                passed=False,
                score=0.0,
                threshold=self.config["validation_thresholds"]["kinematic_accuracy"],
                details={"error": "Missing joint position data"}
            )

        # Calculate joint position error
        joint_error = np.mean(np.abs(sim_joints - real_joints))

        # Determine if test passes
        passed = joint_error < self.config["validation_thresholds"]["kinematic_accuracy"]

        result = ValidationResult(
            test_name="kinematic_validation",
            passed=passed,
            score=joint_error,
            threshold=self.config["validation_thresholds"]["kinematic_accuracy"],
            details={
                "mean_joint_error": joint_error,
                "sim_joints": sim_joints.tolist(),
                "real_joints": real_joints.tolist()
            }
        )

        self.results.append(result)
        return result

    def validate_dynamics(self, sim_trajectory: List[Dict], real_trajectory: List[Dict]) -> ValidationResult:
        """Validate dynamic behavior between simulation and real robot"""
        if len(sim_trajectory) == 0 or len(real_trajectory) == 0:
            return ValidationResult(
                test_name="dynamic_validation",
                passed=False,
                score=0.0,
                threshold=self.config["validation_thresholds"]["dynamic_accuracy"],
                details={"error": "Missing trajectory data"}
            )

        # Calculate trajectory similarity
        sim_positions = np.array([point["position"] for point in sim_trajectory])
        real_positions = np.array([point["position"] for point in real_trajectory])

        # Interpolate trajectories to same length if needed
        if len(sim_positions) != len(real_positions):
            min_len = min(len(sim_positions), len(real_positions))
            sim_positions = sim_positions[:min_len]
            real_positions = real_positions[:min_len]

        # Calculate position error
        position_errors = np.linalg.norm(sim_positions - real_positions, axis=1)
        mean_position_error = np.mean(position_errors)

        # Calculate velocity error
        sim_velocities = np.diff(sim_positions, axis=0)
        real_velocities = np.diff(real_positions, axis=0)

        if len(sim_velocities) > 0 and len(real_velocities) > 0:
            if len(sim_velocities) != len(real_velocities):
                min_len = min(len(sim_velocities), len(real_velocities))
                sim_velocities = sim_velocities[:min_len]
                real_velocities = real_velocities[:min_len]

            velocity_errors = np.linalg.norm(sim_velocities - real_velocities, axis=1)
            mean_velocity_error = np.mean(velocity_errors)
        else:
            mean_velocity_error = float('inf')

        result = ValidationResult(
            test_name="dynamic_validation",
            passed=mean_position_error < self.config["validation_thresholds"]["dynamic_accuracy"],
            score=mean_position_error,
            threshold=self.config["validation_thresholds"]["dynamic_accuracy"],
            details={
                "mean_position_error": mean_position_error,
                "mean_velocity_error": mean_velocity_error,
                "max_position_error": np.max(position_errors) if len(position_errors) > 0 else 0.0
            }
        )

        self.results.append(result)
        return result

    def validate_balance(self, sim_states: List[Dict], real_states: List[Dict]) -> ValidationResult:
        """Validate balance behavior between simulation and real robot"""
        # Calculate Zero-Moment Point (ZMP) for both sim and real
        sim_zmp = self._compute_zmp_batch(sim_states)
        real_zmp = self._compute_zmp_batch(real_states)

        if len(sim_zmp) == 0 or len(real_zmp) == 0:
            return ValidationResult(
                test_name="balance_validation",
                passed=False,
                score=float('inf'),
                threshold=self.config["validation_thresholds"]["balance_stability"],
                details={"error": "Unable to compute ZMP data"}
            )

        # Calculate ZMP tracking error
        if len(sim_zmp) != len(real_zmp):
            min_len = min(len(sim_zmp), len(real_zmp))
            sim_zmp = sim_zmp[:min_len]
            real_zmp = real_zmp[:min_len]

        zmp_errors = np.linalg.norm(sim_zmp[:, :2] - real_zmp[:, :2], axis=1)
        mean_zmp_error = np.mean(zmp_errors)

        # Calculate CoM stability
        sim_com = np.array([state.get("com_position", [0, 0, 0]) for state in sim_states])
        real_com = np.array([state.get("com_position", [0, 0, 0]) for state in real_states])

        if len(sim_com) != len(real_com):
            min_len = min(len(sim_com), len(real_com))
            sim_com = sim_com[:min_len]
            real_com = real_com[:min_len]

        com_errors = np.linalg.norm(sim_com - real_com, axis=1)
        mean_com_error = np.mean(com_errors)

        result = ValidationResult(
            test_name="balance_validation",
            passed=mean_zmp_error < self.config["validation_thresholds"]["balance_stability"],
            score=mean_zmp_error,
            threshold=self.config["validation_thresholds"]["balance_stability"],
            details={
                "mean_zmp_error": mean_zmp_error,
                "mean_com_error": mean_com_error,
                "max_zmp_error": np.max(zmp_errors) if len(zmp_errors) > 0 else 0.0
            }
        )

        self.results.append(result)
        return result

    def _compute_zmp_batch(self, states: List[Dict]) -> np.ndarray:
        """Compute ZMP for a batch of states"""
        zmp_points = []

        for state in states:
            # Extract CoM position and acceleration
            com_pos = np.array(state.get("com_position", [0.0, 0.0, 0.8]))
            com_acc = np.array(state.get("com_acceleration", [0.0, 0.0, 0.0]))
            gravity = 9.81

            # Calculate ZMP = CoM_xy - (CoM_z / g) * CoM_acc_xy
            if abs(gravity) > 1e-6:
                zmp_x = com_pos[0] - (com_pos[2] / gravity) * com_acc[0]
                zmp_y = com_pos[1] - (com_pos[2] / gravity) * com_acc[1]
                zmp_points.append([zmp_x, zmp_y, 0.0])

        return np.array(zmp_points)

    def validate_locomotion(self, sim_trajectory: List[Dict], real_trajectory: List[Dict]) -> ValidationResult:
        """Validate locomotion behavior"""
        # Check if robot successfully completed the task
        sim_success = self._check_task_success(sim_trajectory)
        real_success = self._check_task_success(real_trajectory)

        success_rate = 0.0
        if len(real_trajectory) > 0:
            success_rate = float(sim_success == real_success)

        # Calculate path similarity
        if len(sim_trajectory) > 0 and len(real_trajectory) > 0:
            sim_path = np.array([point.get("position", [0, 0, 0]) for point in sim_trajectory])
            real_path = np.array([point.get("position", [0, 0, 0]) for point in real_trajectory])

            if len(sim_path) != len(real_path):
                min_len = min(len(sim_path), len(real_path))
                sim_path = sim_path[:min_len]
                real_path = real_path[:min_len]

            path_errors = np.linalg.norm(sim_path[:, :2] - real_path[:, :2], axis=1)
            mean_path_error = np.mean(path_errors)
        else:
            mean_path_error = float('inf')

        result = ValidationResult(
            test_name="locomotion_validation",
            passed=success_rate >= self.config["validation_thresholds"]["locomotion_success"],
            score=success_rate,
            threshold=self.config["validation_thresholds"]["locomotion_success"],
            details={
                "success_rate": success_rate,
                "mean_path_error": mean_path_error,
                "sim_success": sim_success,
                "real_success": real_success
            }
        )

        self.results.append(result)
        return result

    def _check_task_success(self, trajectory: List[Dict]) -> bool:
        """Check if task was completed successfully"""
        # This is a simplified check - in practice, this would be more complex
        if not trajectory:
            return False

        # Check if robot reached target position (example: within 0.1m of target)
        final_pos = trajectory[-1].get("position", [0, 0, 0])
        target_pos = trajectory[0].get("target_position", [1, 0, 0])

        distance = np.linalg.norm(np.array(final_pos[:2]) - np.array(target_pos[:2]))
        return distance < 0.1  # Within 10cm of target

    def validate_manipulation(self, sim_trials: List[Dict], real_trials: List[Dict]) -> ValidationResult:
        """Validate manipulation success rates"""
        if len(sim_trials) == 0 or len(real_trials) == 0:
            return ValidationResult(
                test_name="manipulation_validation",
                passed=False,
                score=0.0,
                threshold=self.config["validation_thresholds"]["manipulation_success"],
                details={"error": "No trials provided for manipulation validation"}
            )

        # Calculate success rates
        sim_successes = sum(1 for trial in sim_trials if trial.get("success", False))
        real_successes = sum(1 for trial in real_trials if trial.get("success", False))

        sim_success_rate = sim_successes / len(sim_trials)
        real_success_rate = real_successes / len(real_trials)

        # Calculate similarity between success rates
        success_rate_similarity = 1.0 - abs(sim_success_rate - real_success_rate)

        result = ValidationResult(
            test_name="manipulation_validation",
            passed=sim_success_rate >= self.config["validation_thresholds"]["manipulation_success"],
            score=sim_success_rate,
            threshold=self.config["validation_thresholds"]["manipulation_success"],
            details={
                "sim_success_rate": sim_success_rate,
                "real_success_rate": real_success_rate,
                "success_rate_similarity": success_rate_similarity,
                "sim_successes": sim_successes,
                "real_successes": real_successes
            }
        )

        self.results.append(result)
        return result

    def run_comprehensive_validation(self, sim_data: Dict, real_data: Dict) -> Dict:
        """Run comprehensive validation suite"""
        self.logger.info("Starting comprehensive validation...")

        start_time = time.time()

        # Run all validation tests
        kinematic_result = self.validate_kinematics(
            sim_data.get("kinematic_data", {}),
            real_data.get("kinematic_data", {})
        )

        dynamic_result = self.validate_dynamics(
            sim_data.get("dynamic_trajectory", []),
            real_data.get("dynamic_trajectory", [])
        )

        balance_result = self.validate_balance(
            sim_data.get("balance_states", []),
            real_data.get("balance_states", [])
        )

        locomotion_result = self.validate_locomotion(
            sim_data.get("locomotion_trajectory", []),
            real_data.get("locomotion_trajectory", [])
        )

        manipulation_result = self.validate_manipulation(
            sim_data.get("manipulation_trials", []),
            real_data.get("manipulation_trials", [])
        )

        total_time = time.time() - start_time

        # Compile results
        overall_score = np.mean([
            kinematic_result.score if kinematic_result.passed else float('inf'),
            dynamic_result.score if dynamic_result.passed else float('inf'),
            balance_result.score if balance_result.passed else float('inf'),
            locomotion_result.score if locomotion_result.passed else float('inf'),
            manipulation_result.score if manipulation_result.passed else float('inf')
        ])

        overall_passed = all([
            kinematic_result.passed,
            dynamic_result.passed,
            balance_result.passed,
            locomotion_result.passed,
            manipulation_result.passed
        ])

        report = {
            "timestamp": datetime.now().isoformat(),
            "overall_passed": overall_passed,
            "overall_score": overall_score,
            "total_validation_time": total_time,
            "individual_results": {
                "kinematic": kinematic_result.__dict__,
                "dynamic": dynamic_result.__dict__,
                "balance": balance_result.__dict__,
                "locomotion": locomotion_result.__dict__,
                "manipulation": manipulation_result.__dict__
            },
            "summary": {
                "total_tests": 5,
                "passed_tests": sum(1 for r in [kinematic_result, dynamic_result, balance_result, locomotion_result, manipulation_result] if r.passed),
                "pass_rate": sum(1 for r in [kinematic_result, dynamic_result, balance_result, locomotion_result, manipulation_result] if r.passed) / 5.0
            }
        }

        self.logger.info(f"Validation completed in {total_time:.2f}s")
        self.logger.info(f"Overall result: {'PASS' if overall_passed else 'FAIL'}")
        self.logger.info(f"Pass rate: {report['summary']['pass_rate']*100:.1f}%")

        return report

    def generate_validation_report(self, report: Dict, output_path: str = "validation_report.pdf"):
        """Generate a comprehensive validation report"""
        # Create visualizations
        self._create_validation_plots(report)

        # Save report as JSON
        with open(output_path.replace(".pdf", ".json"), 'w') as f:
            json.dump(report, f, indent=2)

        self.logger.info(f"Validation report saved to {output_path}")

    def _create_validation_plots(self, report: Dict):
        """Create plots for validation results"""
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle('Isaac Sim Validation Results', fontsize=16)

        # 1. Pass/Fail Summary
        results = report["individual_results"]
        test_names = list(results.keys())
        passed = [results[test]["passed"] for test in test_names]

        axes[0, 0].bar(test_names, [1 if p else 0 for p in passed], color=['green' if p else 'red' for p in passed])
        axes[0, 0].set_title('Test Results (Pass/Fail)')
        axes[0, 0].set_ylabel('Pass (1) / Fail (0)')
        axes[0, 0].tick_params(axis='x', rotation=45)

        # 2. Score Comparison
        scores = [results[test]["score"] for test in test_names]
        thresholds = [results[test]["threshold"] for test in test_names]

        x = np.arange(len(test_names))
        width = 0.35

        axes[0, 1].bar(x - width/2, scores, width, label='Score', alpha=0.8)
        axes[0, 1].bar(x + width/2, thresholds, width, label='Threshold', alpha=0.8)
        axes[0, 1].set_title('Scores vs Thresholds')
        axes[0, 1].set_ylabel('Value')
        axes[0, 1].set_xticks(x)
        axes[0, 1].set_xticklabels(test_names, rotation=45)
        axes[0, 1].legend()

        # 3. Error Distribution (if applicable)
        if "dynamic" in results and "details" in results["dynamic"]:
            dynamic_details = results["dynamic"]["details"]
            if "mean_position_error" in dynamic_details:
                errors = [dynamic_details["mean_position_error"]]
                axes[0, 2].bar(["Mean Position Error"], errors, color='orange')
                axes[0, 2].set_title('Dynamic Validation Error')
                axes[0, 2].set_ylabel('Error (m)')

        # 4. Success Rates (if applicable)
        if "locomotion" in results and "details" in results["locomotion"]:
            loco_details = results["locomotion"]["details"]
            if "success_rate" in loco_details:
                success_rates = [loco_details["success_rate"]]
                axes[1, 0].bar(["Locomotion Success Rate"], success_rates, color='purple', alpha=0.7)
                axes[1, 0].set_ylim([0, 1])
                axes[1, 0].set_title('Locomotion Success Rate')
                axes[1, 0].set_ylabel('Success Rate')

        if "manipulation" in results and "details" in results["manipulation"]:
            manip_details = results["manipulation"]["details"]
            if "sim_success_rate" in manip_details:
                success_rates = [manip_details["sim_success_rate"], manip_details["real_success_rate"]]
                axes[1, 1].bar(["Sim", "Real"], success_rates, color=['blue', 'red'], alpha=0.7)
                axes[1, 1].set_ylim([0, 1])
                axes[1, 1].set_title('Manipulation Success Rates')
                axes[1, 1].set_ylabel('Success Rate')

        # 5. Performance Metrics
        performance_data = [
            report["summary"]["pass_rate"],
            report["total_validation_time"]
        ]
        perf_labels = ["Pass Rate", "Validation Time (s)"]
        axes[1, 2].bar(perf_labels, performance_data, color=['cyan', 'magenta'])
        axes[1, 2].set_title('Overall Performance')
        axes[1, 2].set_ylabel('Value')

        # Remove empty subplot
        fig.delaxes(axes[1, 2])

        plt.tight_layout()
        plt.savefig("validation_report.png", dpi=300, bbox_inches='tight')
        plt.close()

        self.logger.info("Validation plots created and saved as validation_report.png")


class PerformanceOptimizer:
    """Optimize Isaac Sim performance"""

    def __init__(self, config_path: str = None):
        self.config = self._load_config(config_path)
        self.optimization_history = []

    def _load_config(self, config_path: str) -> Dict:
        """Load optimization configuration"""
        if config_path and os.path.exists(config_path):
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        else:
            # Default configuration
            return {
                "optimization_targets": {
                    "min_fps": 60,
                    "max_memory_usage": 80,  # Percentage
                    "max_gpu_usage": 90,     # Percentage
                    "physics_accuracy": 0.01 # Error tolerance
                },
                "optimization_parameters": {
                    "physics_substeps_range": [1, 16],
                    "render_resolution_range": [0.5, 1.0],
                    "lod_distance_range": [5.0, 50.0],
                    "thread_count_range": [1, 16]
                }
            }

    def optimize_physics_settings(self, current_settings: Dict) -> Dict:
        """Optimize physics settings for performance"""
        optimized_settings = current_settings.copy()

        # Adjust physics parameters based on performance targets
        target_fps = self.config["optimization_targets"]["min_fps"]

        # If we need better performance, reduce physics accuracy
        if current_settings.get("current_fps", 60) < target_fps:
            # Reduce solver iterations
            optimized_settings["solver_position_iterations"] = max(
                4,  # Minimum reasonable value
                int(current_settings.get("solver_position_iterations", 8) * 0.75)
            )

            optimized_settings["solver_velocity_iterations"] = max(
                2,  # Minimum reasonable value
                int(current_settings.get("solver_velocity_iterations", 4) * 0.75)
            )

            # Reduce substeps if currently high
            if current_settings.get("max_substeps", 8) > 8:
                optimized_settings["max_substeps"] = max(4, current_settings.get("max_substeps", 8) // 2)

        # If we have excess performance, increase accuracy
        elif current_settings.get("current_fps", 60) > target_fps * 1.5:
            # Increase solver iterations for better accuracy
            optimized_settings["solver_position_iterations"] = min(
                16,  # Maximum reasonable value
                int(current_settings.get("solver_position_iterations", 8) * 1.25)
            )

            optimized_settings["solver_velocity_iterations"] = min(
                8,  # Maximum reasonable value
                int(current_settings.get("solver_velocity_iterations", 4) * 1.25)
            )

        return optimized_settings

    def optimize_rendering_settings(self, current_settings: Dict) -> Dict:
        """Optimize rendering settings for performance"""
        optimized_settings = current_settings.copy()

        # Adjust rendering based on performance targets
        current_fps = current_settings.get("current_fps", 60)
        target_fps = self.config["optimization_targets"]["min_fps"]

        if current_fps < target_fps:
            # Reduce rendering quality to improve performance
            optimized_settings["render_resolution_scale"] = max(
                0.5,  # Minimum reasonable scale
                current_settings.get("render_resolution_scale", 1.0) * 0.9
            )

            # Reduce shadow quality
            optimized_settings["shadow_quality"] = "low"

            # Disable expensive effects
            optimized_settings["enable_reflections"] = False
            optimized_settings["enable_global_illumination"] = False
            optimized_settings["enable_antialiasing"] = False

            # Enable level of detail
            optimized_settings["enable_lod"] = True
            optimized_settings["lod_bias"] = 1.0  # More aggressive LOD

        elif current_fps > target_fps * 1.2:
            # If we have excess performance, improve quality
            optimized_settings["render_resolution_scale"] = min(
                1.0,  # Maximum scale
                current_settings.get("render_resolution_scale", 1.0) * 1.1
            )

            # Increase shadow quality if needed
            if current_settings.get("shadow_quality", "low") == "low":
                optimized_settings["shadow_quality"] = "medium"

        return optimized_settings

    def optimize_for_training(self) -> Dict:
        """Optimize settings specifically for training"""
        training_settings = {
            # Physics settings for training
            "physics_dt": 1.0/60.0,  # 60 FPS physics
            "solver_type": "TGS",    # TGS solver for stability
            "solver_position_iterations": 4,  # Reduced for speed
            "solver_velocity_iterations": 2,  # Reduced for speed
            "max_substeps": 4,       # Reduced for speed
            "enable_ccd": False,     # Disable CCD for speed
            "contact_offset": 0.01,  # Slightly larger for stability
            "rest_offset": 0.001,    # Small rest offset

            # Rendering settings for training
            "enable_rendering": False,  # Disable rendering for training
            "render_resolution_scale": 0.5,  # Lower resolution when enabled
            "shadow_quality": "low",
            "enable_reflections": False,
            "enable_global_illumination": False,
            "enable_post_processing": False,

            # Performance settings
            "num_threads": 8,        # Use multiple threads
            "enable_multithreading": True,
            "enable_gpu_compute": True,

            # Asset settings
            "enable_texture_compression": True,
            "enable_mesh_simplification": True,
            "enable_instance_rendering": True,  # For multi-env training
        }

        return training_settings

    def optimize_for_visualization(self) -> Dict:
        """Optimize settings specifically for visualization"""
        viz_settings = {
            # Physics settings for visualization
            "physics_dt": 1.0/60.0,  # 60 FPS physics
            "solver_type": "TGS",    # TGS solver for stability
            "solver_position_iterations": 8,  # Higher for accuracy
            "solver_velocity_iterations": 4,  # Higher for accuracy
            "max_substeps": 8,       # Higher for accuracy
            "enable_ccd": True,      # Enable CCD for accurate contacts
            "contact_offset": 0.005, # Smaller for accuracy
            "rest_offset": 0.0005,   # Smaller for accuracy

            # Rendering settings for visualization
            "enable_rendering": True,  # Enable rendering
            "render_resolution_scale": 1.0,  # Full resolution
            "shadow_quality": "high",
            "enable_reflections": True,
            "enable_global_illumination": True,
            "enable_post_processing": True,

            # Performance settings
            "num_threads": 16,       # Use more threads if available
            "enable_multithreading": True,
            "enable_gpu_compute": True,

            # Asset settings
            "enable_texture_compression": False,
            "enable_mesh_simplification": False,
            "enable_instance_rendering": False,  # Better quality for single env
        }

        return viz_settings

    def benchmark_settings(self, settings: Dict, test_scenario: str = "default") -> Dict:
        """Benchmark specific settings configuration"""
        # This would run Isaac Sim with the given settings and measure performance
        # For this implementation, we'll simulate benchmarking results

        import random

        # Simulate performance metrics based on settings
        fps = 60.0  # Base FPS

        # Adjust FPS based on rendering settings
        if not settings.get("enable_rendering", True):
            fps *= 3.0  # Rendering disabled, expect big boost
        elif settings.get("render_resolution_scale", 1.0) < 0.7:
            fps *= 1.5  # Lower resolution gives boost

        if settings.get("shadow_quality", "high") == "low":
            fps *= 1.2

        if not settings.get("enable_global_illumination", True):
            fps *= 1.1

        # Adjust based on physics settings
        if settings.get("solver_position_iterations", 8) < 6:
            fps *= 1.1

        if settings.get("max_substeps", 8) < 4:
            fps *= 1.05

        # Add some randomness to simulate real-world variation
        fps *= random.uniform(0.9, 1.1)

        # Memory and GPU usage simulation
        memory_usage = 30.0  # Base memory usage in %
        if settings.get("render_resolution_scale", 1.0) > 0.8:
            memory_usage += 20
        if settings.get("enable_global_illumination", False):
            memory_usage += 15
        if settings.get("enable_reflections", False):
            memory_usage += 10

        gpu_usage = 40.0  # Base GPU usage in %
        if settings.get("enable_rendering", True):
            gpu_usage += 30
        if settings.get("shadow_quality", "high") == "high":
            gpu_usage += 20
        if settings.get("enable_global_illumination", False):
            gpu_usage += 25

        benchmark_results = {
            "fps": fps,
            "memory_usage_percent": min(memory_usage, 100.0),
            "gpu_usage_percent": min(gpu_usage, 100.0),
            "physics_accuracy_estimate": 1.0 / (settings.get("solver_position_iterations", 8) * 0.5 + 1),
            "simulation_stability": random.uniform(0.8, 1.0)  # 0-1 scale
        }

        return benchmark_results

    def find_optimal_settings(self, objective: str = "balanced") -> Tuple[Dict, Dict]:
        """
        Find optimal settings based on objective

        Args:
            objective: "performance", "quality", "balanced", or "training"

        Returns:
            Tuple of (optimal_settings, benchmark_results)
        """
        if objective == "training":
            settings = self.optimize_for_training()
        elif objective == "visualization":
            settings = self.optimize_for_visualization()
        elif objective == "performance":
            # Start with training settings and adjust
            settings = self.optimize_for_training()
            settings["enable_rendering"] = True  # Need some rendering for visualization
            settings["render_resolution_scale"] = 0.7  # Medium resolution
        elif objective == "quality":
            settings = self.optimize_for_visualization()
            settings["solver_position_iterations"] = 12  # Extra accuracy
            settings["max_substeps"] = 16  # Extra stability
        else:  # balanced
            settings = {
                "physics_dt": 1.0/60.0,
                "solver_type": "TGS",
                "solver_position_iterations": 6,
                "solver_velocity_iterations": 3,
                "max_substeps": 8,
                "enable_ccd": True,
                "contact_offset": 0.005,
                "rest_offset": 0.001,

                "enable_rendering": True,
                "render_resolution_scale": 0.8,
                "shadow_quality": "medium",
                "enable_reflections": False,
                "enable_global_illumination": False,
                "enable_post_processing": True,

                "num_threads": 12,
                "enable_multithreading": True,
                "enable_gpu_compute": True,

                "enable_texture_compression": True,
                "enable_mesh_simplification": False,
                "enable_instance_rendering": False,
            }

        # Benchmark the settings
        results = self.benchmark_settings(settings)

        # Store in optimization history
        self.optimization_history.append({
            "settings": settings,
            "results": results,
            "objective": objective,
            "timestamp": datetime.now().isoformat()
        })

        return settings, results


def main():
    """Main function for validation and optimization"""
    print("Isaac Sim Validation and Optimization Framework")
    print("=" * 50)

    # Initialize validator
    validator = IsaacSimValidator()

    # Initialize optimizer
    optimizer = PerformanceOptimizer()

    # Example usage of validation
    print("\n1. Running Validation Example:")

    # Simulated data for validation (in practice, this would come from real experiments)
    sim_data = {
        "kinematic_data": {
            "joint_positions": [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]] * 100  # 100 timesteps
        },
        "dynamic_trajectory": [
            {"position": [i*0.01, 0, 0], "timestamp": i*0.01} for i in range(100)
        ],
        "balance_states": [
            {"com_position": [0.1, 0.0, 0.8], "com_acceleration": [0.01, 0.0, 0.0]}
            for _ in range(50)
        ],
        "locomotion_trajectory": [
            {"position": [i*0.02, 0, 0], "target_position": [1.0, 0, 0]}
            for i in range(50)
        ],
        "manipulation_trials": [
            {"success": True, "trial_id": i} for i in range(20)
        ]
    }

    real_data = {
        "kinematic_data": {
            "joint_positions": [[0.11, 0.21, 0.31, 0.41, 0.51, 0.61, 0.71]] * 100  # Slightly different
        },
        "dynamic_trajectory": [
            {"position": [i*0.0105, 0.001, 0.001], "timestamp": i*0.01} for i in range(100)
        ],
        "balance_states": [
            {"com_position": [0.105, 0.001, 0.801], "com_acceleration": [0.011, 0.001, 0.001]}
            for _ in range(50)
        ],
        "locomotion_trajectory": [
            {"position": [i*0.0205, 0.001, 0.001], "target_position": [1.0, 0, 0]}
            for i in range(50)
        ],
        "manipulation_trials": [
            {"success": True, "trial_id": i} for i in range(18)  # Slightly lower success
        ] + [
            {"success": False, "trial_id": i} for i in range(18, 20)
        ]
    }

    # Run validation
    validation_report = validator.run_comprehensive_validation(sim_data, real_data)

    print(f"Overall validation result: {'PASS' if validation_report['overall_passed'] else 'FAIL'}")
    print(f"Pass rate: {validation_report['summary']['pass_rate']*100:.1f}%")

    # Example usage of optimization
    print("\n2. Running Optimization Example:")

    # Find optimal settings for different objectives
    objectives = ["balanced", "performance", "quality", "training"]

    for obj in objectives:
        settings, results = optimizer.find_optimal_settings(obj)
        print(f"\n{obj.upper()} SETTINGS:")
        print(f"  FPS: {results['fps']:.2f}")
        print(f"  Memory Usage: {results['memory_usage_percent']:.1f}%")
        print(f"  GPU Usage: {results['gpu_usage_percent']:.1f}%")
        print(f"  Physics Accuracy: {results['physics_accuracy_estimate']:.3f}")

    # Generate validation report
    print("\n3. Generating Validation Report...")
    validator.generate_validation_report(validation_report)

    print("\nValidation and optimization framework completed successfully!")


if __name__ == "__main__":
    main()