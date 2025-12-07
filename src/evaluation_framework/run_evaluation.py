#!/usr/bin/env python3

"""
Main Entry Point for the Comprehensive Evaluation Framework
"""

import argparse
import sys
import os
from pathlib import Path
import json
import yaml
from datetime import datetime

# Add the current directory to the path to import our modules
sys.path.insert(0, str(Path(__file__).parent))

from evaluation_framework import ComprehensiveEvaluator


def main():
    parser = argparse.ArgumentParser(description='Comprehensive Evaluation Framework for Physical AI & Humanoid Robotics Textbook')

    parser.add_argument(
        '--config',
        type=str,
        default='config/evaluation_config.yaml',
        help='Path to evaluation configuration file'
    )

    parser.add_argument(
        '--output',
        type=str,
        default='evaluation_reports/comprehensive_evaluation.json',
        help='Path to output evaluation report'
    )

    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Enable verbose output'
    )

    parser.add_argument(
        '--modules',
        nargs='+',
        choices=['module1', 'module2', 'module3', 'module4'],
        default=['module1', 'module2', 'module3', 'module4'],
        help='Specific modules to evaluate (default: all modules)'
    )

    args = parser.parse_args()

    if args.verbose:
        print("Initializing Comprehensive Evaluation Framework...")
        print(f"Configuration file: {args.config}")
        print(f"Output path: {args.output}")
        print(f"Modules to evaluate: {args.modules}")

    # Create output directory if it doesn't exist
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)

    # Initialize evaluator with configuration
    evaluator = ComprehensiveEvaluator(config_path=args.config)

    if args.verbose:
        print("Starting evaluation of specified modules...")

    # Evaluate selected modules
    results = {}
    for module in args.modules:
        if args.verbose:
            print(f"Evaluating {module}...")

        # Depending on the module, call the appropriate evaluator
        if module == "module1":
            module_evaluator = evaluator.module_evaluators["module1"]
        elif module == "module2":
            module_evaluator = evaluator.module_evaluators["module2"]
        elif module == "module3":
            module_evaluator = evaluator.module_evaluators["module3"]
        elif module == "module4":
            module_evaluator = evaluator.module_evaluators["module4"]

        module_results = {
            "content_quality": module_evaluator.evaluate_content_quality(),
            "implementation_quality": module_evaluator.evaluate_implementation_quality(),
            "pedagogical_effectiveness": module_evaluator.evaluate_pedagogical_effectiveness()
        }

        # Calculate overall score for the module
        content_score = evaluator._calculate_content_score(module_results["content_quality"])
        implementation_score = evaluator._calculate_implementation_score(module_results["implementation_quality"])
        pedagogy_score = evaluator._calculate_pedagogical_score(module_results["pedagogical_effectiveness"])

        # Apply weights from config
        weights = evaluator.config["evaluation_criteria"]
        overall_score = (
            content_score * weights["content_quality_weight"] +
            implementation_score * weights["implementation_quality_weight"] +
            pedagogy_score * weights["pedagogical_effectiveness_weight"]
        )

        # Determine grade
        if overall_score >= evaluator.config["thresholds"]["excellent_threshold"]:
            grade = "Excellent"
        elif overall_score >= evaluator.config["thresholds"]["good_threshold"]:
            grade = "Good"
        elif overall_score >= evaluator.config["thresholds"]["minimum_passing_score"]:
            grade = "Satisfactory"
        else:
            grade = "Needs Improvement"

        results[module] = {
            "content_quality": {
                "score": content_score,
                "details": module_results["content_quality"]
            },
            "implementation_quality": {
                "score": implementation_score,
                "details": module_results["implementation_quality"]
            },
            "pedagogical_effectiveness": {
                "score": pedagogy_score,
                "details": module_results["pedagogical_effectiveness"]
            },
            "overall_score": overall_score,
            "grade": grade
        }

        if args.verbose:
            print(f"  Content Quality Score: {content_score:.2f}")
            print(f"  Implementation Quality Score: {implementation_score:.2f}")
            print(f"  Pedagogical Effectiveness Score: {pedagogy_score:.2f}")
            print(f"  Overall Score: {overall_score:.2f} ({grade})")

    # Calculate global statistics
    global_stats = evaluator._calculate_global_statistics(results)

    if args.verbose:
        print("\nCalculating global statistics...")
        print(f"Average overall score: {global_stats['average_overall_score']:.2f}")
        print(f"Passing modules: {global_stats['passing_modules']}/{global_stats['total_modules']}")

    # Add global stats to results
    final_results = {
        "evaluation_metadata": {
            "timestamp": datetime.now().isoformat(),
            "evaluated_modules": args.modules,
            "configuration_used": args.config
        },
        "module_results": results,
        "global_statistics": global_stats
    }

    # Generate comprehensive report
    report_path = evaluator.generate_evaluation_report(final_results, args.output)

    # Create visualizations
    if evaluator.config["output_settings"]["include_visualizations"]:
        if args.verbose:
            print("Generating visualizations...")
        evaluator.visualize_evaluation_results(results)

    print(f"\nEvaluation completed successfully!")
    print(f"Report saved to: {report_path}")

    # Print summary
    print("\n" + "="*60)
    print("EVALUATION SUMMARY")
    print("="*60)
    for module, result in results.items():
        print(f"{module.upper()}: {result['overall_score']:.2f} ({result['grade']})")

    print(f"\nGLOBAL STATISTICS:")
    print(f"  Average Score: {global_stats['average_overall_score']:.2f}")
    print(f"  Passing Rate: {global_stats['passing_modules']}/{global_stats['total_modules']} ({global_stats['passing_modules']/global_stats['total_modules']*100:.1f}%)")
    print(f"  Overall Grade: {global_stats['overall_grade']}")


if __name__ == "__main__":
    main()