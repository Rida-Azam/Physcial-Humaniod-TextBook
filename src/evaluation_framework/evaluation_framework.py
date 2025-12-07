#!/usr/bin/env python3

"""
Comprehensive Evaluation Framework for Physical AI & Humanoid Robotics Textbook

This module provides a complete evaluation framework for assessing the quality and effectiveness
of the humanoid robotics implementations across all 4 modules.
"""

import os
import json
import yaml
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import statistics
import logging


class ModuleEvaluator:
    """
    Base class for evaluating individual modules
    """

    def __init__(self, module_name: str, config: Dict):
        self.module_name = module_name
        self.config = config
        self.evaluation_results = {}
        self.logger = self._setup_logger()

    def _setup_logger(self) -> logging.Logger:
        """Set up logger for the evaluator"""
        logger = logging.getLogger(f"{self.module_name}_evaluator")
        logger.setLevel(logging.INFO)

        handler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        logger.addHandler(handler)

        return logger

    def evaluate_content_quality(self) -> Dict:
        """
        Evaluate the quality of content in the module
        """
        raise NotImplementedError("Subclasses must implement evaluate_content_quality")

    def evaluate_implementation_quality(self) -> Dict:
        """
        Evaluate the quality of implementation in the module
        """
        raise NotImplementedError("Subclasses must implement evaluate_implementation_quality")

    def evaluate_pedagogical_effectiveness(self) -> Dict:
        """
        Evaluate the pedagogical effectiveness of the module
        """
        raise NotImplementedError("Subclasses must implement evaluate_pedagogical_effectiveness")


class Module1Evaluator(ModuleEvaluator):
    """
    Evaluator for Module 1: Introduction to Physical AI & Humanoid Robotics
    """

    def evaluate_content_quality(self) -> Dict:
        """Evaluate content quality for Module 1"""
        self.logger.info("Evaluating content quality for Module 1...")

        # Check for essential topics in the chapters
        chapters_path = Path("docs/module1")
        chapters = ["chapter1.mdx", "chapter2.mdx", "chapter3.mdx"]

        content_quality = {
            "completeness": {
                "introduction_to_physical_ai": self._check_topic_coverage(chapters_path / "chapter1.mdx", ["embodied", "intelligence", "physical", "ai"]),
                "humanoid_landscape": self._check_topic_coverage(chapters_path / "chapter2.mdx", ["humanoid", "landscape", "2025", "2030", "robots"]),
                "ros_architecture": self._check_topic_coverage(chapters_path / "chapter3.mdx", ["ros", "architecture", "nodes", "topics", "services"])
            },
            "accuracy": self._check_citations_and_references(chapters_path),
            "readability": self._check_readability_metrics(chapters_path)
        }

        return content_quality

    def evaluate_implementation_quality(self) -> Dict:
        """Evaluate implementation quality for Module 1"""
        self.logger.info("Evaluating implementation quality for Module 1...")

        implementation_quality = {
            "ros_workspace_setup": self._check_ros_workspace_completeness("src/ros2_workspaces"),
            "urdf_models": self._check_urdf_models("src/urdf_models"),
            "code_examples": self._check_code_examples_runnable("docs/module1")
        }

        return implementation_quality

    def evaluate_pedagogical_effectiveness(self) -> Dict:
        """Evaluate pedagogical effectiveness for Module 1"""
        self.logger.info("Evaluating pedagogical effectiveness for Module 1...")

        pedagogical_effectiveness = {
            "learning_objectives_met": self._check_learning_objectives("docs/module1"),
            "progressive_difficulty": self._check_progressive_difficulty("docs/module1"),
            "exercises_and_projects": self._check_exercises_presence("docs/module1")
        }

        return pedagogical_effectiveness

    def _check_topic_coverage(self, chapter_path: Path, keywords: List[str]) -> float:
        """Check how well a chapter covers the required topics"""
        if not chapter_path.exists():
            return 0.0

        with open(chapter_path, 'r', encoding='utf-8') as f:
            content = f.read().lower()

        keyword_count = sum(1 for keyword in keywords if keyword.lower() in content)
        return keyword_count / len(keywords)

    def _check_citations_and_references(self, chapters_path: Path) -> float:
        """Check presence and quality of citations and references"""
        citations_found = 0
        total_chapters = 0

        for chapter_file in chapters_path.glob("*.mdx"):
            with open(chapter_file, 'r', encoding='utf-8') as f:
                content = f.read()
                if "cite" in content.lower() or "reference" in content.lower() or "[@]" in content:
                    citations_found += 1
            total_chapters += 1

        return citations_found / total_chapters if total_chapters > 0 else 0.0

    def _check_readability_metrics(self, chapters_path: Path) -> Dict:
        """Check readability metrics for chapters"""
        avg_word_count = 0
        avg_sentence_count = 0
        chapter_count = 0

        for chapter_file in chapters_path.glob("*.mdx"):
            with open(chapter_file, 'r', encoding='utf-8') as f:
                content = f.read()

                # Simple word and sentence counting
                words = content.split()
                sentences = [s for s in content.split('.') if len(s.strip()) > 0]

                avg_word_count += len(words)
                avg_sentence_count += len(sentences)
                chapter_count += 1

        if chapter_count > 0:
            avg_word_count /= chapter_count
            avg_sentence_count /= chapter_count

        return {
            "avg_words_per_chapter": avg_word_count,
            "avg_sentences_per_chapter": avg_sentence_count,
            "estimated_grade_level": self._estimate_grade_level(avg_word_count, avg_sentence_count)
        }

    def _estimate_grade_level(self, avg_words, avg_sentences) -> float:
        """Estimate grade level using Flesch-Kincaid"""
        # Simplified Flesch-Kincaid Grade Level calculation
        if avg_sentences > 0:
            avg_words_per_sentence = avg_words / avg_sentences
            return (0.39 * avg_words_per_sentence) + (11.8 * 10.0) - 15.59  # Using 10 syllables per 100 words as approximation
        return 0.0

    def _check_ros_workspace_completeness(self, workspace_path: str) -> bool:
        """Check if ROS workspaces are properly set up"""
        workspace_dir = Path(workspace_path)
        if not workspace_dir.exists():
            return False

        # Check for essential files/components
        has_package_xml = any(workspace_dir.rglob("package.xml"))
        has_launch_files = any(workspace_dir.rglob("*.launch.py"))
        has_config_files = any(workspace_dir.rglob("*.yaml")) or any(workspace_dir.rglob("*.yml"))

        return has_package_xml and has_launch_files and has_config_files

    def _check_urdf_models(self, urdf_path: str) -> bool:
        """Check if URDF models are properly defined"""
        urdf_dir = Path(urdf_path)
        if not urdf_dir.exists():
            return False

        urdf_files = list(urdf_dir.rglob("*.urdf")) + list(urdf_dir.rglob("*.xacro"))
        return len(urdf_files) > 0

    def _check_code_examples_runnable(self, docs_path: str) -> float:
        """Check if code examples in docs are properly formatted and potentially runnable"""
        docs_dir = Path(docs_path)
        if not docs_dir.exists():
            return 0.0

        code_blocks_found = 0
        runnable_indicators = 0

        for mdx_file in docs_dir.rglob("*.mdx"):
            with open(mdx_file, 'r', encoding='utf-8') as f:
                content = f.read()
                # Count code blocks
                code_blocks = content.count("```")
                code_blocks_found += code_blocks // 2  # Each block has opening and closing

                # Look for indicators of runnable code
                if "ros2" in content or "rclpy" in content or "launch" in content:
                    runnable_indicators += 1

        return runnable_indicators / code_blocks_found if code_blocks_found > 0 else 0.0

    def _check_learning_objectives(self, docs_path: str) -> bool:
        """Check if learning objectives are defined in chapters"""
        docs_dir = Path(docs_path)
        objectives_found = 0
        total_chapters = 0

        for mdx_file in docs_dir.rglob("*.mdx"):
            with open(mdx_file, 'r', encoding='utf-8') as f:
                content = f.read()
                if "learning objective" in content.lower() or "learning goals" in content.lower():
                    objectives_found += 1
            total_chapters += 1

        return objectives_found / total_chapters > 0.5 if total_chapters > 0 else False  # At least 50% should have objectives

    def _check_progressive_difficulty(self, docs_path: str) -> bool:
        """Check if content follows progressive difficulty"""
        # For now, just check if chapters are numbered and seem to progress logically
        docs_dir = Path(docs_path)
        chapter_files = sorted(docs_dir.rglob("*.mdx"))

        # Check if we have a reasonable number of chapters with increasing complexity
        return len(chapter_files) >= 3

    def _check_exercises_presence(self, docs_path: str) -> bool:
        """Check if exercises are present in chapters"""
        docs_dir = Path(docs_path)
        exercises_found = 0
        total_chapters = 0

        for mdx_file in docs_dir.rglob("*.mdx"):
            with open(mdx_file, 'r', encoding='utf-8') as f:
                content = f.read()
                if "exercise" in content.lower() or "problem" in content.lower() or "assignment" in content.lower():
                    exercises_found += 1
            total_chapters += 1

        return exercises_found > 0


class Module2Evaluator(ModuleEvaluator):
    """
    Evaluator for Module 2: The Digital Twin - Simulation
    """

    def evaluate_content_quality(self) -> Dict:
        """Evaluate content quality for Module 2"""
        self.logger.info("Evaluating content quality for Module 2...")

        # Module 2 covers simulation topics
        content_quality = {
            "completeness": {
                "gazebo_integration": self._check_topic_coverage("docs/module2/chapter4.mdx", ["gazebo", "simulation", "physics", "engine"]),
                "isaac_sim": self._check_topic_coverage("docs/module2/chapter5.mdx", ["isaac", "simulation", "nvidia", "environment"]),
                "digital_twin": self._check_topic_coverage("docs/module2/chapter6.mdx", ["digital twin", "simulation", "real-world", "mapping"])
            },
            "accuracy": self._check_citations_and_references(Path("docs/module2")),
            "readability": self._check_readability_metrics(Path("docs/module2"))
        }

        return content_quality

    def evaluate_implementation_quality(self) -> Dict:
        """Evaluate implementation quality for Module 2"""
        self.logger.info("Evaluating implementation quality for Module 2...")

        implementation_quality = {
            "gazebo_setup": self._check_gazebo_integration("docker/gazebo_env"),
            "isaac_sim_integration": self._check_isaac_sim_integration("docker/isaac_sim_env"),
            "simulation_scenarios": self._check_simulation_scenarios("src/gazebo_worlds")
        }

        return implementation_quality

    def evaluate_pedagogical_effectiveness(self) -> Dict:
        """Evaluate pedagogical effectiveness for Module 2"""
        self.logger.info("Evaluating pedagogical effectiveness for Module 2...")

        pedagogical_effectiveness = {
            "simulation_understanding": self._check_simulation_concepts("docs/module2"),
            "hands_on_exercises": self._check_hands_on_content("docs/module2"),
            "real_world_connection": self._check_real_world_connection("docs/module2")
        }

        return pedagogical_effectiveness

    def _check_gazebo_integration(self, gazebo_path: str) -> bool:
        """Check if Gazebo integration is properly set up"""
        gazebo_dir = Path(gazebo_path)
        if not gazebo_dir.exists():
            return False

        # Look for essential Gazebo files
        has_world_files = any(gazebo_dir.rglob("*.world"))
        has_models = any(gazebo_dir.rglob("model.sdf")) or any(gazebo_dir.rglob("model.config"))
        has_launch_files = any(gazebo_dir.rglob("*gazebo*.launch.py"))

        return has_world_files and (has_models or has_launch_files)

    def _check_isaac_sim_integration(self, isaac_path: str) -> bool:
        """Check if Isaac Sim integration is properly set up"""
        isaac_dir = Path(isaac_path)
        if not isaac_dir.exists():
            return False

        # Look for Isaac Sim specific files
        has_config = any(isaac_dir.rglob("config*.py")) or any(isaac_dir.rglob("*.yaml"))
        has_scripts = any(isaac_dir.rglob("*isaac*.py"))
        has_docs = any(isaac_dir.rglob("README.md"))

        return has_config and has_scripts

    def _check_simulation_scenarios(self, worlds_path: str) -> bool:
        """Check if simulation scenarios are properly defined"""
        worlds_dir = Path(worlds_path)
        if not worlds_dir.exists():
            # Check if the directory exists in our created structure
            return Path("src/gazebo_worlds").exists() or Path("docker/gazebo_env").exists()

        world_files = list(worlds_dir.rglob("*.world")) + list(worlds_dir.rglob("*.sdf"))
        return len(world_files) > 0

    def _check_simulation_concepts(self, docs_path: str) -> bool:
        """Check if simulation concepts are properly explained"""
        docs_dir = Path(docs_path)
        concepts_found = 0
        total_chapters = 0

        for mdx_file in docs_dir.rglob("*.mdx"):
            with open(mdx_file, 'r', encoding='utf-8') as f:
                content = f.read()
                # Look for simulation-related concepts
                if any(concept in content.lower() for concept in ["physics", "rendering", "collision", "dynamics", "kinematics"]):
                    concepts_found += 1
            total_chapters += 1

        return concepts_found > 0

    def _check_hands_on_content(self, docs_path: str) -> bool:
        """Check if hands-on exercises are present"""
        docs_dir = Path(docs_path)
        hands_on_found = 0
        total_chapters = 0

        for mdx_file in docs_dir.rglob("*.mdx"):
            with open(mdx_file, 'r', encoding='utf-8') as f:
                content = f.read()
                if any(term in content.lower() for term in ["exercise", "try", "practice", "implement", "experiment"]):
                    hands_on_found += 1
            total_chapters += 1

        return hands_on_found > 0

    def _check_real_world_connection(self, docs_path: str) -> bool:
        """Check if connection to real-world robotics is established"""
        docs_dir = Path(docs_path)
        connection_found = 0
        total_chapters = 0

        for mdx_file in docs_dir.rglob("*.mdx"):
            with open(mdx_file, 'r', encoding='utf-8') as f:
                content = f.read()
                if any(term in content.lower() for term in ["real robot", "physical", "deployment", "sim-to-real", "transfer"]):
                    connection_found += 1
            total_chapters += 1

        return connection_found > 0


class Module3Evaluator(ModuleEvaluator):
    """
    Evaluator for Module 3: Vision-Language-Action Models
    """

    def evaluate_content_quality(self) -> Dict:
        """Evaluate content quality for Module 3"""
        self.logger.info("Evaluating content quality for Module 3...")

        content_quality = {
            "completeness": {
                "isaac_ros": self._check_topic_coverage("docs/module3/chapter10.mdx", ["isaac", "ros", "perception", "navigation"]),
                "bipedal_locomotion": self._check_topic_coverage("docs/module3/chapter11.mdx", ["bipedal", "locomotion", "walking", "zmp", "mpc"]),
                "dexterous_manipulation": self._check_topic_coverage("docs/module3/chapter12.mdx", ["manipulation", "grasp", "transfer", "dexterous"])
            },
            "accuracy": self._check_citations_and_references(Path("docs/module3")),
            "readability": self._check_readability_metrics(Path("docs/module3"))
        }

        return content_quality

    def evaluate_implementation_quality(self) -> Dict:
        """Evaluate implementation quality for Module 3"""
        self.logger.info("Evaluating implementation quality for Module 3...")

        implementation_quality = {
            "nav2_humanoid": self._check_nav2_implementation("src/nav2_humanoid"),
            "isaac_gym_rl": self._check_rl_implementation("src/isaac_gym_rl"),
            "perception_pipeline": self._check_perception_implementation("src/perception_pipeline")
        }

        return implementation_quality

    def evaluate_pedagogical_effectiveness(self) -> Dict:
        """Evaluate pedagogical effectiveness for Module 3"""
        self.logger.info("Evaluating pedagogical effectiveness for Module 3...")

        pedagogical_effectiveness = {
            "practical_application": self._check_practical_focus("docs/module3"),
            "algorithmic_understanding": self._check_algorithmic_content("docs/module3"),
            "integration_skills": self._check_integration_content("docs/module3")
        }

        return pedagogical_effectiveness

    def _check_nav2_implementation(self, nav2_path: str) -> bool:
        """Check if Nav2 stack is properly implemented for humanoid"""
        nav2_dir = Path(nav2_path)
        if not nav2_dir.exists():
            # Check in our created structure
            return Path("src/nav2_humanoid").exists()

        has_config = any(nav2_dir.rglob("*config*.yaml"))
        has_launch = any(nav2_dir.rglob("*launch*.py"))
        has_params = any(nav2_dir.rglob("*param*.yaml"))

        return has_config or has_launch

    def _check_rl_implementation(self, rl_path: str) -> bool:
        """Check if RL implementation is properly set up"""
        rl_dir = Path(rl_path)
        if not rl_dir.exists():
            # Check in our created structure
            return Path("src/isaac_gym_rl").exists()

        has_training_scripts = any(rl_dir.rglob("*train*.py")) or any(rl_dir.rglob("*learn*.py"))
        has_policy_files = any(rl_dir.rglob("*.pt")) or any(rl_dir.rglob("*.pth")) or any(rl_dir.rglob("*.onnx"))

        return has_training_scripts  # At least training scripts should exist

    def _check_perception_implementation(self, perception_path: str) -> bool:
        """Check if perception pipeline is properly implemented"""
        perception_dir = Path(perception_path)
        if not perception_dir.exists():
            # Check in our created structure
            return Path("src/perception_pipeline").exists()

        has_detection = any(perception_dir.rglob("*detect*.py")) or any(perception_dir.rglob("*vision*.py"))
        has_config = any(perception_dir.rglob("*.yaml")) or any(perception_dir.rglob("*.json"))

        return has_detection  # At minimum, detection components should exist

    def _check_practical_focus(self, docs_path: str) -> bool:
        """Check if content focuses on practical implementation"""
        docs_dir = Path(docs_path)
        practical_found = 0
        total_chapters = 0

        for mdx_file in docs_dir.rglob("*.mdx"):
            with open(mdx_file, 'r', encoding='utf-8') as f:
                content = f.read()
                if any(term in content.lower() for term in ["implementation", "code", "practical", "example", "tutorial", "how to"]):
                    practical_found += 1
            total_chapters += 1

        return practical_found > 0

    def _check_algorithmic_content(self, docs_path: str) -> bool:
        """Check if algorithmic concepts are properly explained"""
        docs_dir = Path(docs_path)
        algorithmic_found = 0
        total_chapters = 0

        for mdx_file in docs_dir.rglob("*.mdx"):
            with open(mdx_file, 'r', encoding='utf-8') as f:
                content = f.read()
                if any(term in content.lower() for term in ["algorithm", "equation", "mathematical", "model", "network", "policy"]):
                    algorithmic_found += 1
            total_chapters += 1

        return algorithmic_found > 0

    def _check_integration_content(self, docs_path: str) -> bool:
        """Check if system integration concepts are covered"""
        docs_dir = Path(docs_path)
        integration_found = 0
        total_chapters = 0

        for mdx_file in docs_dir.rglob("*.mdx"):
            with open(mdx_file, 'r', encoding='utf-8') as f:
                content = f.read()
                if any(term in content.lower() for term in ["integration", "system", "pipeline", "workflow", "framework"]):
                    integration_found += 1
            total_chapters += 1

        return integration_found > 0


class Module4Evaluator(ModuleEvaluator):
    """
    Evaluator for Module 4: Capstone Project - Autonomous Humanoid from Spoken Command
    """

    def evaluate_content_quality(self) -> Dict:
        """Evaluate content quality for Module 4"""
        self.logger.info("Evaluating content quality for Module 4...")

        content_quality = {
            "completeness": {
                "vision_language_action": self._check_topic_coverage("docs/module4/chapter13.mdx", ["vision", "language", "action", "vla", "model"]),
                "voice_to_plan": self._check_topic_coverage("docs/module4/chapter14.mdx", ["voice", "plan", "action", "nlp", "planning"]),
                "capstone_project": self._check_topic_coverage("docs/module4/chapter15.mdx", ["capstone", "project", "integration", "end-to-end"])
            },
            "accuracy": self._check_citations_and_references(Path("docs/module4")),
            "readability": self._check_readability_metrics(Path("docs/module4"))
        }

        return content_quality

    def evaluate_implementation_quality(self) -> Dict:
        """Evaluate implementation quality for Module 4"""
        self.logger.info("Evaluating implementation quality for Module 4...")

        implementation_quality = {
            "openvla_finetune": self._check_openvla_implementation("capstone/openvla_finetune.ipynb"),
            "capstone_repo": self._check_capstone_implementation("capstone/"),
            "pretrained_models": self._check_pretrained_weights("datasets/")
        }

        return implementation_quality

    def evaluate_pedagogical_effectiveness(self) -> Dict:
        """Evaluate pedagogical effectiveness for Module 4"""
        self.logger.info("Evaluating pedagogical effectiveness for Module 4...")

        pedagogical_effectiveness = {
            "end_to_end_integration": self._check_end_to_end_content("docs/module4"),
            "evaluation_rubric": self._check_evaluation_content("capstone/evaluation_rubric.md"),
            "comprehensive_assessment": self._check_comprehensive_assessment("capstone/")
        }

        return pedagogical_effectiveness

    def _check_openvla_implementation(self, notebook_path: str) -> bool:
        """Check if OpenVLA fine-tuning notebook is properly set up"""
        notebook_file = Path(notebook_path)
        if not notebook_file.exists():
            # Check in our created structure
            return Path("capstone/openvla_finetune.ipynb").exists()

        with open(notebook_file, 'r', encoding='utf-8') as f:
            content = f.read()
            # Check for key OpenVLA elements
            has_imports = "openvla" in content.lower() or "transformers" in content.lower()
            has_training = "train" in content.lower() or "finetune" in content.lower()
            has_model = "model" in content.lower() or "processor" in content.lower()

        return has_imports and (has_training or has_model)

    def _check_capstone_implementation(self, capstone_path: str) -> bool:
        """Check if capstone implementation is properly structured"""
        capstone_dir = Path(capstone_path)
        if not capstone_dir.exists():
            return False

        has_readme = (capstone_dir / "README.md").exists()
        has_config = any(capstone_dir.rglob("*.yaml")) or any(capstone_dir.rglob("*.json"))
        has_notebooks = any(capstone_dir.rglob("*.ipynb"))
        has_scripts = any(capstone_dir.rglob("*.py"))

        return has_readme and (has_config or has_notebooks or has_scripts)

    def _check_pretrained_weights(self, datasets_path: str) -> bool:
        """Check if pretrained weights/datasets are available"""
        datasets_dir = Path(datasets_path)
        if not datasets_dir.exists():
            # Check in our created structure
            return Path("datasets/").exists()

        # Look for model files or dataset files
        has_models = any(datasets_dir.rglob("*.pt")) or any(datasets_dir.rglob("*.pth"))
        has_datasets = any(datasets_dir.rglob("*.json")) or any(datasets_dir.rglob("*.csv")) or any(datasets_dir.rglob("*.pkl"))

        return has_models or has_datasets or len(list(datasets_dir.iterdir())) > 0

    def _check_end_to_end_content(self, docs_path: str) -> bool:
        """Check if end-to-end integration concepts are covered"""
        docs_dir = Path(docs_path)
        e2e_found = 0
        total_chapters = 0

        for mdx_file in docs_dir.rglob("*.mdx"):
            with open(mdx_file, 'r', encoding='utf-8') as f:
                content = f.read()
                if any(term in content.lower() for term in ["end-to-end", "integration", "complete", "full system", "entire pipeline"]):
                    e2e_found += 1
            total_chapters += 1

        return e2e_found > 0

    def _check_evaluation_content(self, rubric_path: str) -> bool:
        """Check if evaluation rubric is properly defined"""
        rubric_file = Path(rubric_path)
        if not rubric_file.exists():
            # Check in our created structure
            return Path("capstone/evaluation_rubric.md").exists()

        with open(rubric_file, 'r', encoding='utf-8') as f:
            content = f.read()
            return any(term in content.lower() for term in ["evaluation", "rubric", "criteria", "assessment", "metrics"])

    def _check_comprehensive_assessment(self, capstone_path: str) -> bool:
        """Check if comprehensive assessment is available"""
        capstone_dir = Path(capstone_path)
        if not capstone_dir.exists():
            return False

        # Look for assessment-related files
        assessment_files = list(capstone_dir.rglob("*eval*")) + list(capstone_dir.rglob("*assess*")) + list(capstone_dir.rglob("*test*"))
        return len(assessment_files) > 0


class ComprehensiveEvaluator:
    """
    Main evaluator that brings together all module evaluators
    """

    def __init__(self, config_path: Optional[str] = None):
        self.config = self._load_config(config_path)
        self.module_evaluators = {
            "module1": Module1Evaluator("Module 1", self.config),
            "module2": Module2Evaluator("Module 2", self.config),
            "module3": Module3Evaluator("Module 3", self.config),
            "module4": Module4Evaluator("Module 4", self.config)
        }
        self.global_results = {}

    def _load_config(self, config_path: Optional[str]) -> Dict:
        """Load evaluation configuration"""
        if config_path and Path(config_path).exists():
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        else:
            # Default configuration
            return {
                "evaluation_criteria": {
                    "content_quality_weight": 0.3,
                    "implementation_quality_weight": 0.4,
                    "pedagogical_effectiveness_weight": 0.3
                },
                "thresholds": {
                    "minimum_passing_score": 0.6,  # 60% minimum
                    "excellent_threshold": 0.9,   # 90% for excellent
                    "good_threshold": 0.7         # 70% for good
                },
                "output_settings": {
                    "output_directory": "evaluation_reports",
                    "include_visualizations": True,
                    "detailed_analysis": True
                }
            }

    def evaluate_all_modules(self) -> Dict:
        """Evaluate all modules comprehensively"""
        self.logger.info("Starting comprehensive evaluation of all modules...")

        all_results = {}

        for module_name, evaluator in self.module_evaluators.items():
            self.logger.info(f"Evaluating {evaluator.module_name}...")

            # Get individual evaluations
            content_q = evaluator.evaluate_content_quality()
            implementation_q = evaluator.evaluate_implementation_quality()
            pedagogical_e = evaluator.evaluate_pedagogical_effectiveness()

            # Calculate weighted score for this module
            content_score = self._calculate_content_score(content_q)
            implementation_score = self._calculate_implementation_score(implementation_q)
            pedagogical_score = self._calculate_pedagogical_score(pedagogical_e)

            # Weighted module score
            weights = self.config["evaluation_criteria"]
            module_score = (
                content_score * weights["content_quality_weight"] +
                implementation_score * weights["implementation_quality_weight"] +
                pedagogical_score * weights["pedagogical_effectiveness_weight"]
            )

            # Determine module grade
            if module_score >= self.config["thresholds"]["excellent_threshold"]:
                module_grade = "Excellent"
            elif module_score >= self.config["thresholds"]["good_threshold"]:
                module_grade = "Good"
            elif module_score >= self.config["thresholds"]["minimum_passing_score"]:
                module_grade = "Satisfactory"
            else:
                module_grade = "Needs Improvement"

            all_results[module_name] = {
                "content_quality": {
                    "score": content_score,
                    "details": content_q
                },
                "implementation_quality": {
                    "score": implementation_score,
                    "details": implementation_q
                },
                "pedagogical_effectiveness": {
                    "score": pedagogical_score,
                    "details": pedagogical_e
                },
                "overall_score": module_score,
                "grade": module_grade
            }

        # Calculate global statistics
        self.global_results = self._calculate_global_statistics(all_results)

        return all_results

    def _calculate_content_score(self, content_q: Dict) -> float:
        """Calculate content quality score"""
        if not content_q or "completeness" not in content_q:
            return 0.0

        completeness_vals = []
        for category, val in content_q["completeness"].items():
            if isinstance(val, (int, float)):
                completeness_vals.append(val)
            elif isinstance(val, bool):
                completeness_vals.append(1.0 if val else 0.0)
            elif isinstance(val, dict):
                # If it's a dict, take the average of its values
                subvals = [v for v in val.values() if isinstance(v, (int, float, bool))]
                if subvals:
                    completeness_vals.append(sum(subvals) / len(subvals))

        if not completeness_vals:
            return 0.0

        return sum(completeness_vals) / len(completeness_vals)

    def _calculate_implementation_score(self, impl_q: Dict) -> float:
        """Calculate implementation quality score"""
        if not impl_q:
            return 0.0

        implementation_vals = []
        for component, val in impl_q.items():
            if isinstance(val, (int, float)):
                implementation_vals.append(val)
            elif isinstance(val, bool):
                implementation_vals.append(1.0 if val else 0.0)
            elif isinstance(val, dict):
                # If it's a dict, take the average of its values
                subvals = [v for v in val.values() if isinstance(v, (int, float, bool))]
                if subvals:
                    implementation_vals.append(sum(subvals) / len(subvals))

        if not implementation_vals:
            return 0.0

        return sum(implementation_vals) / len(implementation_vals)

    def _calculate_pedagogical_score(self, pedagogy_e: Dict) -> float:
        """Calculate pedagogical effectiveness score"""
        if not pedagogy_e:
            return 0.0

        pedagogy_vals = []
        for aspect, val in pedagogy_e.items():
            if isinstance(val, (int, float)):
                pedagogy_vals.append(val)
            elif isinstance(val, bool):
                pedagogy_vals.append(1.0 if val else 0.0)
            elif isinstance(val, dict):
                # If it's a dict, take the average of its values
                subvals = [v for v in val.values() if isinstance(v, (int, float, bool))]
                if subvals:
                    pedagogy_vals.append(sum(subvals) / len(subvals))

        if not pedagogy_vals:
            return 0.0

        return sum(pedagogy_vals) / len(pedagogy_vals)

    def _calculate_global_statistics(self, all_results: Dict) -> Dict:
        """Calculate global statistics across all modules"""
        module_scores = [result["overall_score"] for result in all_results.values()]

        return {
            "average_overall_score": statistics.mean(module_scores) if module_scores else 0.0,
            "median_overall_score": statistics.median(module_scores) if module_scores else 0.0,
            "min_overall_score": min(module_scores) if module_scores else 0.0,
            "max_overall_score": max(module_scores) if module_scores else 0.0,
            "std_dev_overall_score": statistics.stdev(module_scores) if len(module_scores) > 1 else 0.0,
            "all_module_grades": [result["grade"] for result in all_results.values()],
            "passing_modules": sum(1 for result in all_results.values()
                                 if result["overall_score"] >= self.config["thresholds"]["minimum_passing_score"]),
            "total_modules": len(all_results)
        }

    def generate_evaluation_report(self, results: Dict, output_path: Optional[str] = None) -> str:
        """Generate a comprehensive evaluation report"""
        self.logger.info("Generating comprehensive evaluation report...")

        if output_path is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = f"evaluation_reports/comprehensive_evaluation_{timestamp}.json"

        # Create output directory if it doesn't exist
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)

        # Add global results and timestamp to the report
        report = {
            "evaluation_timestamp": datetime.now().isoformat(),
            "evaluation_config": self.config,
            "module_results": results,
            "global_statistics": self.global_results,
            "summary": self._create_summary(results)
        }

        # Write the report
        with open(output_path, 'w') as f:
            json.dump(report, f, indent=2)

        self.logger.info(f"Comprehensive evaluation report generated: {output_path}")
        return output_path

    def _create_summary(self, results: Dict) -> Dict:
        """Create a summary of the evaluation"""
        passing_count = sum(1 for r in results.values()
                           if r["overall_score"] >= self.config["thresholds"]["minimum_passing_score"])
        total_count = len(results)

        return {
            "total_modules_evaluated": total_count,
            "passing_modules": passing_count,
            "pass_rate": passing_count / total_count if total_count > 0 else 0,
            "average_score": statistics.mean([r["overall_score"] for r in results.values()]) if results else 0,
            "highest_scoring_module": max(results.keys(),
                                         key=lambda k: results[k]["overall_score"]) if results else None,
            "lowest_scoring_module": min(results.keys(),
                                        key=lambda k: results[k]["overall_score"]) if results else None,
            "overall_grade": self._determine_overall_grade([r["overall_score"] for r in results.values()])
        }

    def _determine_overall_grade(self, scores: List[float]) -> str:
        """Determine overall grade based on average score"""
        if not scores:
            return "No Data"

        avg_score = sum(scores) / len(scores)
        if avg_score >= self.config["thresholds"]["excellent_threshold"]:
            return "Excellent"
        elif avg_score >= self.config["thresholds"]["good_threshold"]:
            return "Good"
        elif avg_score >= self.config["thresholds"]["minimum_passing_score"]:
            return "Satisfactory"
        else:
            return "Needs Improvement"

    def visualize_evaluation_results(self, results: Dict, output_path: Optional[str] = None):
        """Create visualizations of the evaluation results"""
        self.logger.info("Creating evaluation visualizations...")

        if output_path is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = f"evaluation_reports/evaluation_visualization_{timestamp}.png"

        # Create output directory if it doesn't exist
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)

        # Prepare data for visualization
        module_names = list(results.keys())
        overall_scores = [results[mod]["overall_score"] for mod in module_names]
        content_scores = [results[mod]["content_quality"]["score"] for mod in module_names]
        implementation_scores = [results[mod]["implementation_quality"]["score"] for mod in module_names]
        pedagogy_scores = [results[mod]["pedagogical_effectiveness"]["score"] for mod in module_names]

        # Create a comprehensive visualization
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle('Comprehensive Evaluation Results - Physical AI & Humanoid Robotics Textbook', fontsize=16)

        # 1. Overall Scores by Module
        bars1 = axes[0, 0].bar(module_names, overall_scores, color=['skyblue', 'lightgreen', 'lightcoral', 'lightsalmon'])
        axes[0, 0].set_title('Overall Scores by Module')
        axes[0, 0].set_ylabel('Score')
        axes[0, 0].set_ylim(0, 1)
        axes[0, 0].grid(axis='y', alpha=0.3)

        # Add value labels on bars
        for bar, score in zip(bars1, overall_scores):
            axes[0, 0].text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                           f'{score:.2f}', ha='center', va='bottom')

        # 2. Component Breakdown
        x = np.arange(len(module_names))
        width = 0.25

        bars2_content = axes[0, 1].bar(x - width, content_scores, width, label='Content Quality', color='skyblue')
        bars2_impl = axes[0, 1].bar(x, implementation_scores, width, label='Implementation Quality', color='lightgreen')
        bars2_ped = axes[0, 1].bar(x + width, pedagogy_scores, width, label='Pedagogical Effectiveness', color='lightcoral')

        axes[0, 1].set_title('Component Breakdown by Module')
        axes[0, 1].set_ylabel('Score')
        axes[0, 1].set_ylim(0, 1)
        axes[0, 1].set_xticks(x)
        axes[0, 1].set_xticklabels(module_names)
        axes[0, 1].legend()
        axes[0, 1].grid(axis='y', alpha=0.3)

        # 3. Score Distribution
        all_scores = content_scores + implementation_scores + pedagogy_scores
        axes[1, 0].hist(all_scores, bins=10, density=True, alpha=0.7, color='steelblue', edgecolor='black')
        axes[1, 0].axvline(statistics.mean(all_scores), color='red', linestyle='--', label=f'Mean: {statistics.mean(all_scores):.2f}')
        axes[1, 0].set_title('Distribution of All Scores')
        axes[1, 0].set_xlabel('Score')
        axes[1, 0].set_ylabel('Density')
        axes[1, 0].legend()
        axes[1, 0].grid(alpha=0.3)

        # 4. Radar Chart for Module 4 (Capstone)
        if "module4" in results:
            module4_results = results["module4"]
            categories = ['Content\nQuality', 'Implementation\nQuality', 'Pedagogical\nEffectiveness', 'Overall']
            values = [
                module4_results["content_quality"]["score"],
                module4_results["implementation_quality"]["score"],
                module4_results["pedagogical_effectiveness"]["score"],
                module4_results["overall_score"]
            ]

            # Create radar chart
            angles = np.linspace(0, 2 * np.pi, len(categories), endpoint=False).tolist()
            values += values[:1]  # Complete the circle
            angles += angles[:1]

            ax_radar = fig.add_subplot(2, 2, 4, projection='polar')
            ax_radar.fill(angles, values, color='gold', alpha=0.5)
            ax_radar.plot(angles, values, color='orange', linewidth=2)
            ax_radar.set_xticks(angles[:-1])
            ax_radar.set_xticklabels(categories)
            ax_radar.set_ylim(0, 1)
            ax_radar.set_title('Module 4 (Capstone) Breakdown', pad=20)

        plt.tight_layout()
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()

        self.logger.info(f"Evaluation visualizations saved to: {output_path}")


def main():
    """Main function to run the comprehensive evaluation"""
    print("Starting Comprehensive Evaluation of Physical AI & Humanoid Robotics Textbook")
    print("=" * 80)

    # Initialize evaluator
    evaluator = ComprehensiveEvaluator()

    # Run evaluation on all modules
    results = evaluator.evaluate_all_modules()

    # Generate report
    report_path = evaluator.generate_evaluation_report(results)

    # Create visualizations
    evaluator.visualize_evaluation_results(results)

    # Print summary
    print("\n" + "=" * 80)
    print("COMPREHENSIVE EVALUATION SUMMARY")
    print("=" * 80)

    for module_name, result in results.items():
        print(f"\n{result['grade']:<15} {module_name.replace('_', ' ').title()}: {result['overall_score']:.2f}")
        print(f"  Content Quality:          {result['content_quality']['score']:.2f}")
        print(f"  Implementation Quality:   {result['implementation_quality']['score']:.2f}")
        print(f"  Pedagogical Effectiveness: {result['pedagogical_effectiveness']['score']:.2f}")

    print(f"\nGLOBAL STATISTICS:")
    print(f"  Average Score:      {evaluator.global_results['average_overall_score']:.2f}")
    print(f"  Passing Modules:    {evaluator.global_results['passing_modules']}/{evaluator.global_results['total_modules']}")
    print(f"  Overall Grade:      {evaluator.global_results['all_module_grades']}")

    print(f"\nDetailed report saved to: {report_path}")
    print("Visualizations saved to: evaluation_reports/")
    print("\nEvaluation completed successfully!")


if __name__ == "__main__":
    main()