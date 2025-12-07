# Implementation Plan: Physical AI & Humanoid Robotics – 4-Module Open Textbook

**Project**: Physical AI & Humanoid Robotics – 4-Module Open Textbook
**Feature Branch**: `0001-physical-ai-textbook`
**Created**: 2025-12-06
**Status**: Draft
**Timeline Goal**: 12 weeks to complete MVP textbook (publicly deployable and course-ready)

## 1. Architecture Sketch (Textbook + Code + Deployment)

### Docusaurus v3 Site Structure
- **Docs**: Primary content for all 15 chapters, organized into 4 modules. Each chapter will be an MDX file.
- **Blog**: Optional, for project updates or supplementary articles. Not a primary focus for MVP.
- **Versioned Docs Strategy**: Not applicable for initial MVP, will be considered for future editions if content undergoes significant changes.

### MDX Component Library
- **Interactive Code Blocks**: Enabled by `Thebe/Starlark` (as clarified in spec).
- **Tabs**: For presenting alternative code examples or content views.
- **Admonitions**: For notes, warnings, tips, and important information.
- **Embedded Colab/Thebe Notebooks**: Direct embedding of executable code examples for hands-on learning.

### GitHub Repository Layout
- `src/`: Source code for runnable examples, ROS 2 packages, simulation environments, and capstone projects. Organized by module and chapter.
- `docs/`: Docusaurus MDX files for the textbook content, structured by module and chapter.
- `capstone/`: Dedicated directory for the capstone project, including code, models, and deployment instructions.
- `datasets/`: Small datasets or links to larger datasets required for exercises and projects.
- `docker/`: Dockerfiles and Docker Compose configurations for reproducible development and CI environments.

### GitHub Actions Workflow
- **Automatic Pages Deployment**: Triggered on merge to `main` branch. Builds Docusaurus site and deploys to GitHub Pages.
- **Linting**: Enforces code style for Python, MDX, and Docusaurus configuration files.
- **Plagiarism Check**: Integrates CopyLeaks + GPTZero scans on new/modified text content.
- **Code Execution**: Runs all ROS 2 examples within CI Docker (Jazzy + Isaac Sim headless).
- **Readability Check**: Hemingway App + Flesch-Kincaid 10–12 enforced via markdownlint rules for all textbook content.

### Docker/DevContainer Definition
- **Reproducible Authoring Environment**: Dockerfile and `.devcontainer` configuration for a consistent development setup, including ROS 2 Jazzy, Python 3.10+, Isaac Sim 2024.1+, and Docusaurus build tools.

## 2. Section Structure (Final 15 Chapters grouped into 4 Modules)

| Module # | Module Title                         | Chapters   | Chapter Titles                                                         | Estimated Page Count | Primary Deliverables (Per Chapter)                                   |
|----------|--------------------------------------|------------|------------------------------------------------------------------------|----------------------|----------------------------------------------------------------------|
| 1        | The Robotic Nervous System (ROS 2)   | 1–5        | 1. Introduction to Physical AI and Embodied Intelligence               | 15-20                | Conceptual understanding, ROS 2 overview.                            |
|          |                                      |            | 2. The Humanoid Landscape 2025–2030                                    | 15-20                | Future trends, ethical considerations.                               |
|          |                                      |            | 3. ROS 2 Architecture Deep Dive (Nodes, Topics, Services, Actions)     | 20-25                | ROS 2 fundamental concepts, diagrams.                                |
|          |                                      |            | 4. Building Real ROS 2 Packages in Python (rclpy)                      | 25-30                | Runnable ROS 2 Python examples, basic packages.                      |
|          |                                      |            | 5. URDF & Robot Description Mastery                                    | 20-25                | URDF models of simple humanoids.                                     |
| **Module Deliverables** | | | | | 15+ runnable ROS 2 workspaces, Full Docusaurus sidebar grouping. |
| 2        | The Digital Twin (Simulation)        | 6–9        | 6. Simulation Fundamentals – Gazebo Harmonic & Physics Engines         | 20-25                | Gazebo environment setup, basic physics concepts.                    |
|          |                                      |            | 7. NVIDIA Isaac Sim – The New Standard for Humanoid Training           | 25-30                | Isaac Sim setup, ROS 2 bridge.                                       |
|          |                                      |            | 8. Building High-Fidelity Environments & Sensor Simulation             | 25-30                | USD scene examples, sensor models.                                   |
|          |                                      |            | 9. Unity as Alternative/Complementary Visualizer (optional path)       | 15-20                | Unity integration concepts (conceptual only).                        |
| **Module Deliverables** | | | | | Complete Isaac Sim + ROS 2 bridge setup, USD scenes, domain randomization scripts. |
| 3        | The AI-Robot Brain (NVIDIA Isaac Platform) | 10–12   | 10. Isaac ROS – Hardware-Accelerated Perception & Navigation          | 25-30                | Isaac ROS introduction, Nav2 stack for humanoids.                    |
|          |                                      |            | 11. Bipedal Locomotion – ZMP → MPC → RL Walking Policies              | 30-35                | RL walking policies in Isaac Gym.                                    |
|          |                                      |            | 12. Dexterous Manipulation & Sim-to-Real Grasp Transfer               | 25-30                | Grasping algorithms, sim-to-real transfer concepts.                  |
| **Module Deliverables** | | | | | Working Nav2 stack, RL walking policy (Isaac Gym), full perception pipeline. |
| 4        | Vision-Language-Action & Conversational Humanoids | 13–15 | 13. Vision-Language-Action Models (OpenVLA, RT-2-X, π0, Octo)     | 30-35                | VLA model overview, fine-tuning concepts.                            |
|          |                                      |            | 14. From Voice → Plan → Action (Whisper + LLM Planner + ROS Executor) | 30-35                | Voice processing, LLM planning, ROS execution.                       |
|          |                                      |            | 15. Capstone Project – Autonomous Humanoid from Spoken Command        | 35-40                | End-to-end capstone project, real-robot deployment.                  |
| **Module Deliverables** | | | | | End-to-end Colab for OpenVLA, complete capstone repo, pre-trained weights/dataset. |

## 3. Research Approach

-   **Research-Concurrent Model**: Sources will be gathered iteratively while drafting each chapter. No upfront full literature review.
-   **Weekly Source Target**: Minimum 12 new traceable sources per week, with at least 7 being peer-reviewed.
-   **Citation Workflow**: Zotero will be used for source management, integrating with BetterBibTeX for BibTeX export, then converted to APA 7th edition format for auto-import into MDX.
-   **Primary Venues**: RSS, ICRA, IROS, CoRL, NeurIPS, Science Robotics, arXiv (focus 2020–2025), NVIDIA GTC talks, Unitree/Figure/Boston Dynamics technical reports.

## 4. Quality Validation & Testing Strategy (tied to Success Criteria)

-   **Plagiarism**: CopyLeaks and GPTZero scans will be integrated into CI on every commit. A threshold of <2% allowed (excluding citations and code blocks) will be enforced.
-   **Code Execution**: All ROS 2 examples and simulation code must run successfully within a CI Docker environment configured with ROS 2 Jazzy and Isaac Sim headless.
-   **Sim-to-Real Checks**: At least one policy per module will be tested on a real Unitree Go2/G1 robot, with video proof of successful execution.
-   **Readability**: Hemingway App and Flesch-Kincaid grade levels 10–12 will be enforced via markdownlint rules for all textbook content.
-   **Deployment Validation**: GitHub Actions will build and deploy the Docusaurus site with zero errors after every chapter merge.
-   **Pedagogical Review**: The textbook will undergo external review by at least one robotics professor before final public release.

## 5. Decisions Needing Documentation

**Decision**: Primary robot for real-world examples: Unitree G1 vs. Unitree Go2 (quadruped proxy) vs. simulation-only

**Chosen Option**: Unitree Go2 (quadruped proxy)

**Justification**:
Given the clarification that the textbook will "focus primarily on simulation, minimal real-hardware requirement" to ensure accessibility, the Unitree Go2 offers a practical balance. It's a widely recognized and accessible quadruped platform often used in robotics education, providing a tangible real-world component for students without requiring the full complexity and cost of a true bipedal humanoid like the G1 for initial learning. While the ultimate capstone targets a humanoid, using the Go2 for "minimal real-hardware examples" allows for practical application and video proof, aligning with pedagogical goals while managing hardware expectations. Simulation will cover the majority of complex humanoid locomotion.

## 6. Phased Implementation Roadmap (12 weeks)

| Phase                          | Weeks     | Key Activities                                                                             |
|--------------------------------|-----------|--------------------------------------------------------------------------------------------|
| Phase 1 – Foundation & Module 1| 1–2       | Repository setup, Docusaurus initialization, CI/CD pipeline, devcontainer setup. Chapters 1–3 drafting, first 25 sources gathered. |
| Phase 2 – Module 1 completion + Module 2 | 3–5   | Finish ROS 2 mastery chapters (4-5). Full Isaac Sim + Gazebo setup guides. All module 1 & 2 code running in Docker CI. |
| Phase 3 – Module 3 + Module 4 core | 6–9   | Drafting locomotion, manipulation, and VLA models chapters (Module 3 & 4 core). First two capstone milestones working in simulation. |
| Phase 4 – Real-robot deployment + Capstone | 10–11 | Unitree Go2/G1 real deployment guide development. Full voice → action pipeline integration. Pre-trained weights and datasets published. |
| Phase 5 – Polish & Launch      | 12        | Instructor resources branch (solutions + slides). Final plagiarism and fact-check pass. External review sign-off. Public GitHub Pages launch + announcement. |
