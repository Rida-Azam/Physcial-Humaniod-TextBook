#!/usr/bin/env python3

"""
Entry point script for Isaac Sim validation and optimization
"""

import argparse
import sys
import os
import json
import yaml
from pathlib import Path

# Add the project root to the path to import modules
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from isaac_sim_validation.validation_framework import IsaacSimValidator, PerformanceOptimizer


def main():
    parser = argparse.ArgumentParser(description='Isaac Sim Validation and Optimization Tool')

    parser.add_argument(
        'action',
        choices=['validate', 'optimize', 'benchmark', 'generate-report'],
        help='Action to perform: validate, optimize, benchmark, or generate-report'
    )

    parser.add_argument(
        '--config',
        type=str,
        default='config/validation_config.yaml',
        help='Path to validation configuration file'
    )

    parser.add_argument(
        '--sim-data',
        type=str,
        help='Path to simulation data file (for validation)'
    )

    parser.add_argument(
        '--real-data',
        type=str,
        help='Path to real-world data file (for validation)'
    )

    parser.add_argument(
        '--objective',
        choices=['performance', 'quality', 'balanced', 'training', 'visualization'],
        default='balanced',
        help='Optimization objective'
    )

    parser.add_argument(
        '--output',
        type=str,
        default='./validation_results',
        help='Output directory for results'
    )

    args = parser.parse_args()

    # Create output directory
    os.makedirs(args.output, exist_ok=True)

    if args.action == 'validate':
        if not args.sim_data or not args.real_data:
            print("Error: Both --sim-data and --real-data are required for validation")
            sys.exit(1)

        print("Starting Isaac Sim validation...")

        # Initialize validator
        validator = IsaacSimValidator(config_path=args.config)

        # Load data
        with open(args.sim_data, 'r') as f:
            sim_data = json.load(f)

        with open(args.real_data, 'r') as f:
            real_data = json.load(f)

        # Run validation
        validation_report = validator.run_comprehensive_validation(sim_data, real_data)

        # Save results
        output_path = os.path.join(args.output, 'validation_report.json')
        with open(output_path, 'w') as f:
            json.dump(validation_report, f, indent=2)

        # Generate report
        report_path = os.path.join(args.output, 'validation_report.pdf')
        validator.generate_validation_report(validation_report, report_path)

        print(f"Validation completed. Results saved to {args.output}")
        print(f"Overall result: {'PASS' if validation_report['overall_passed'] else 'FAIL'}")
        print(f"Pass rate: {validation_report['summary']['pass_rate']*100:.1f}%")

    elif args.action == 'optimize':
        print(f"Starting Isaac Sim optimization for {args.objective}...")

        # Initialize optimizer
        optimizer = PerformanceOptimizer(config_path=args.config)

        # Find optimal settings
        optimal_settings, benchmark_results = optimizer.find_optimal_settings(args.objective)

        # Save results
        settings_output = os.path.join(args.output, f'optimal_settings_{args.objective}.json')
        with open(settings_output, 'w') as f:
            json.dump(optimal_settings, f, indent=2)

        results_output = os.path.join(args.output, f'benchmark_results_{args.objective}.json')
        with open(results_output, 'w') as f:
            json.dump(benchmark_results, f, indent=2)

        print(f"Optimization completed for {args.objective} objective.")
        print(f"Results saved to {args.output}")
        print(f"Optimized FPS: {benchmark_results['fps']:.2f}")
        print(f"Memory usage: {benchmark_results['memory_usage_percent']:.1f}%")
        print(f"GPU usage: {benchmark_results['gpu_usage_percent']:.1f}%")

    elif args.action == 'benchmark':
        print("Starting Isaac Sim benchmarking...")

        # Initialize optimizer for benchmarking
        optimizer = PerformanceOptimizer(config_path=args.config)

        # Run benchmark with default settings
        settings = optimizer.get_default_settings()
        benchmark_results = optimizer.benchmark_settings(settings)

        # Save results
        output_path = os.path.join(args.output, 'benchmark_results.json')
        with open(output_path, 'w') as f:
            json.dump(benchmark_results, f, indent=2)

        print(f"Benchmark completed. Results saved to {output_path}")
        print(f"FPS: {benchmark_results['fps']:.2f}")
        print(f"Memory usage: {benchmark_results['memory_usage_percent']:.1f}%")
        print(f"GPU usage: {benchmark_results['gpu_usage_percent']:.1f}%")

    elif args.action == 'generate-report':
        print("Generating validation report from existing data...")

        # Look for validation results in output directory
        validation_report_path = os.path.join(args.output, 'validation_report.json')

        if os.path.exists(validation_report_path):
            with open(validation_report_path, 'r') as f:
                validation_report = json.load(f)

            validator = IsaacSimValidator(config_path=args.config)
            report_path = os.path.join(args.output, 'validation_report.pdf')
            validator.generate_validation_report(validation_report, report_path)

            print(f"Report generated at {report_path}")
        else:
            print(f"Error: No validation report found at {validation_report_path}")
            sys.exit(1)


if __name__ == '__main__':
    main()