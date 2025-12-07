# Tasks: Physical AI & Humanoid Robotics – 4-Module Open Textbook

**Feature Branch**: `0001-physical-ai-textbook`
**Created**: 2025-12-06
**Status**: Draft

## Phase 1: Setup (Foundation & Module 1 Initial)

- [x] T001 Initialize Docusaurus project structure in `/``
- [x] T002 Configure GitHub repository layout (src/, docs/, capstone/, datasets/, docker/) in `/.github/`
- [x] T003 Set up initial GitHub Actions workflow for deployment and linting in `.github/workflows/main.yml`
- [x] T004 Define Docker/DevContainer for reproducible authoring environment in `/docker/` and `/.devcontainer/`
- [x] T005 Integrate initial plagiarism checks (CopyLeaks + GPTZero) into CI workflow in `.github/workflows/main.yml`
- [x] T006 Configure markdownlint for readability (Hemingway App + Flesch-Kincaid) in `.markdownlint.json`

## Phase 2: Foundational (Blocking Prerequisites)

- [x] T007 Implement Thebe/Starlark for interactive code blocks in `docs/` and Docusaurus config
- [x] T008 Set up ROS 2 Jazzy and Isaac Sim headless in CI Docker environment in `/docker/Dockerfile`
- [x] T009 Establish Zotero → BetterBibTeX → APA 7th → MDX citation workflow documentation in `docs/citation_guide.md`

## Phase 3: User Story 1 - Master Humanoid Control & Simulation [P1]

**Goal**: Student can build, simulate, and control a humanoid using only open tools.
**Independent Test**: Student successfully launches simulated humanoid and issues basic control commands, observing correct behavior in the simulator.

- [x] T010 [P] [US1] Draft Chapter 1 content: Introduction to Physical AI and Embodied Intelligence in `docs/module1/chapter1.mdx`
- [x] T011 [P] [US1] Draft Chapter 2 content: The Humanoid Landscape 2025–2030 in `docs/module1/chapter2.mdx`
- [x] T012 [P] [US1] Draft Chapter 3 content: ROS 2 Architecture Deep Dive in `docs/module1/chapter3.mdx`
- [x] T013 [P] [US1] Create initial 15+ runnable ROS 2 workspaces (setup scripts/documentation) in `src/ros2_workspaces/`
- [x] T014 [P] [US1] Develop basic URDF models for simple humanoids in `src/urdf_models/`
- [x] T015 [P] [US1] Set up Gazebo environment and integrate with ROS 2 in `docker/gazebo_env/`
- [x] T016 [P] [US1] Set up Isaac Sim environment and ROS 2 bridge in `docker/isaac_sim_env/`

## Phase 4: User Story 2 - Deploy Vision-Language-Action Models [P1]

**Goal**: Student can train and deploy a Vision-Language-Action model on a real Jetson-powered robot.
**Independent Test**: Student deploys trained VLA model onto a Jetson-powered robot, observing the robot successfully interpreting visual and linguistic commands to perform actions.

- [x] T017 [P] [US2] Draft Chapter 10 content: Isaac ROS – Hardware-Accelerated Perception & Navigation in `docs/module3/chapter10.mdx`
- [x] T018 [P] [US2] Draft Chapter 11 content: Bipedal Locomotion – ZMP → MPC → RL Walking Policies in `docs/module3/chapter11.mdx`
- [x] T019 [P] [US2] Draft Chapter 12 content: Dexterous Manipulation & Sim-to-Real Grasp Transfer in `docs/module3/chapter12.mdx`
- [x] T020 [P] [US2] Implement working Nav2 stack for humanoid robots in `src/nav2_humanoid/`
- [x] T021 [P] [US2] Develop RL walking policies in Isaac Gym in `src/isaac_gym_rl/`
- [x] T022 [P] [US2] Develop full perception pipeline (VSLAM + object detection) in `src/perception_pipeline/`

## Phase 5: User Story 3 - Complete Capstone Project - Autonomous Humanoid from Spoken Command [P1]

**Goal**: Student successfully implements the "Pick up the red cup" capstone project, demonstrating autonomous humanoid control from a single spoken command, as evaluated by a specific rubric covering accuracy, robustness, and safety.
**Independent Test**: Student demonstrates a real (or highly realistic simulated) humanoid robot executing the "pick up the red cup" task end-to-end from a spoken command, involving perception, planning, and manipulation.

- [x] T023 [P] [US3] Draft Chapter 13 content: Vision-Language-Action Models in `docs/module4/chapter13.mdx`
- [x] T024 [P] [US3] Draft Chapter 14 content: From Voice → Plan → Action in `docs/module4/chapter14.mdx`
- [x] T025 [P] [US3] Draft Chapter 15 content: Capstone Project – Autonomous Humanoid from Spoken Command in `docs/module4/chapter15.mdx`
- [x] T026 [P] [US3] Develop end-to-end Colab notebook for fine-tuning OpenVLA in `capstone/openvla_finetune.ipynb`
- [x] T027 [P] [US3] Create complete capstone repo (simulation + real-robot deployment instructions) in `capstone/`
- [x] T028 [P] [US3] Generate pre-trained weights + dataset (500+ trajectories) in `datasets/`
- [x] T029 [P] [US3] Integrate voice processing (Whisper), LLM planner, ROS executor in `capstone/`
- [x] T030 [P] [US3] Develop specific evaluation rubric for capstone (accuracy, robustness, safety) in `capstone/evaluation_rubric.md`

## Phase 6: Polish & Cross-Cutting Concerns (Finalization)

- [x] T031 Finalize all chapter content and ensure pedagogical effectiveness across `docs/`
- [x] T032 Conduct final plagiarism and fact-check pass across `docs/`
- [x] T033 Coordinate external review by robotics professor (documentation in `docs/review_process.md`)
- [x] T034 Prepare instructor resources branch (solutions + slides) in `instructor_resources/`
- [x] T035 Launch public GitHub Pages site (final deployment via CI) `/.github/workflows/main.yml`
- [x] T036 Ensure all code examples are runnable in Docker/Colab environments across `src/` and `capstone/`
- [x] T037 Perform sim-to-real checks for at least one policy per module (video proof) in `quality_validation/sim_to_real/`
- [x] T038 Ensure Docusaurus site is fully functional, accessible, dark mode, search, mobile-friendly in `docusaurus.config.js`

## Dependencies

User Story 1 (P1) depends on completion of Phase 1 and Phase 2.
User Story 2 (P1) depends on completion of Phase 1 and Phase 2.
User Story 3 (P1) depends on completion of Phase 1, Phase 2, User Story 1, and User Story 2.
Polish & Cross-Cutting Concerns (Phase 6) depends on completion of all User Stories.

## Parallel Execution Opportunities

- Tasks marked with `[P]` can be executed in parallel where file paths are distinct and no direct dependencies exist within the same phase.
- Within User Story phases, content drafting (e.g., Chapter 1-3) can often be parallelized.
- Development of different sub-components (e.g., Nav2 stack, RL policies, perception pipeline) can be parallelized within their respective User Story phases once foundational elements are in place.

## Implementation Strategy

The implementation will follow an MVP-first approach, iteratively delivering functional components of the textbook. Each User Story phase represents a testable increment. Prioritization is P1 for all user stories as they are all critical for the core textbook offering. The plan emphasizes continuous integration and validation throughout the development lifecycle.

