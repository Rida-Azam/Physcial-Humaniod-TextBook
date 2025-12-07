---
id: 0003
title: Clarify Textbook Specification
stage: spec
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20251101
feature: physical-ai-textbook
branch: 0001-physical-ai-textbook
user: unknown
command: /sp.clarify
labels: ["clarification", "specification-refinement"]
links:
  spec: specs/0001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
- specs/0001-physical-ai-textbook/spec.md
tests:
- null
---

## Prompt

/sp.clarify
My textbook specification is at /sp.specify (Physical AI & Humanoid Robotics textbook)

Please analyze it for:
1. AMBIGUOUS TERMS
   - What does "complete, open-source, Docusaurus-based textbook" mean exactly? (e.g., which Docusaurus plugins must be included for interactivity, search, or theming?)
   - What does "hands-on mastery" imply for exercises? (e.g., simulation-only, or mandatory real-hardware components?)
   - What constitutes "high-fidelity simulation"? (e.g., minimum physics accuracy levels, frame rates, or realism metrics in Isaac Sim/Gazebo?)
   - What does "executable code examples" mean in practice? (e.g., must they run on free tiers like Colab Basic, or assume paid GPU access?)

2. MISSING ASSUMPTIONS
   - What are the prerequisite knowledge levels for students? (e.g., prior Python proficiency, linear algebra, basic ML, or robotics intro?)
   - What is the expected course duration and pacing? (e.g., 12 weeks exactly, or flexible 10–14 weeks; how many hours per module?)
   - How should updates to tools be handled? (e.g., if ROS 2 or Isaac Sim versions change post-2025, auto-update docs or pin versions?)
   - What budget assumptions for hardware? (e.g., per-student cost caps, or alternatives for low-resource institutions?)

3. INCOMPLETE REQUIREMENTS
   - What should each module's introduction contain? (e.g., learning outcomes, prerequisites, real-world applications, or module roadmap?)
   - How to handle tool incompatibilities or failures? (e.g., fallback if Jetson Orin unavailable; cloud alternatives like AWS RoboMaker?)
   - What defines a "working" capstone project? (e.g., success metrics like 80% task completion rate, or specific evaluation rubrics?)
   - How should chapters integrate assessments? (e.g., quizzes, projects per module, or only end-of-chapter exercises?)

4. SCOPE CONFLICTS
   - Is the focus purely on NVIDIA/Isaac ecosystem, or should alternatives (e.g., MuJoCo, PyBullet) be mentioned for broader accessibility?
   - Is this simulation-heavy or balanced with real-robot deployment? (e.g., proxy robots like Go2 required, or optional?)
   - Are advanced topics like multi-agent collaboration or ethics included? (e.g., briefly in capstone, or excluded per out-of-scope?)
   - Historical vs. forward-looking: How much on past humanoid milestones vs. 2030+ predictions?

List any gaps or questions. Which ones are CRITICAL (planning won't work without them) vs NICE-TO-HAVE (improve quality but not blocking)?
- CRITICAL: Prerequisite knowledge (blocks chapter planning); Hardware budget assumptions (impacts deliverables feasibility); Exact Docusaurus plugins (required for deployment success); Capstone success metrics (defines project completeness).
- NICE-TO-HAVE: Tool update handling (future-proofing); Alternative ecosystems (enhances inclusivity); Detailed module intros (improves pedagogy); Assessment integration (adds value for instructors).

## Response snapshot

All critical clarification questions have been addressed, and the spec has been updated.

## Outcome

- ✅ Impact: Specification is now clearer for planning.
- 🧪 Tests: Clarifications integrated into spec.
- 📁 Files: specs/0001-physical-ai-textbook/spec.md
- 🔁 Next prompts: /sp.plan
- 🧠 Reflection: Successfully navigated the clarification process and updated the spec, including an ADR suggestion to highlight a key decision.

## Evaluation notes (flywheel)

- Failure modes observed: `pwsh` command not found, requiring manual assumptions for feature directory. This was a repeat of the issue from `/sp.specify`.
- Graders run and results (PASS/FAIL): All critical questions addressed.
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Ensure `check-prerequisites.ps1` or similar scripts have cross-platform compatibility or provide clear fallback instructions within the script itself.
