<!--
Sync Impact Report:
- Version change: 1.1.0 → 1.2.0
- Added sections: RAG chatbot integration, agent skills, databases, enhanced auth
- Modified sections: Subagents, Bonus Features, Additional Constraints
- Templates requiring updates: ✅ Updated
- Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Accuracy through Primary Source Verification
All content must be grounded in peer-reviewed research, technical documentation, and real-world validation. Claims must be supported by verifiable sources with proper citations in APA 7th edition format. Zero tolerance for plagiarism with all content subject to verification through plagiarism detection tools.

### II. Clarity for Technical Audience
Content must be accessible to upper-undergraduate to graduate-level students with backgrounds in computer science, robotics, or mechanical/electrical engineering. Maintain Flesch-Kincaid Grade Level 10-12 readability. Complex concepts should be explained with progressive difficulty and hands-on exercises.

### III. Reproducibility and Open Access
All code examples, simulations, and demonstrations must be reproducible using open-source tools only. All claims must be testable, all code runnable in Docker/Colab environments, and all simulations reproducible with provided configurations. All content licensed as CC-BY-SA 4.0 (text) / MIT (code).

### IV. Practical Relevance for Real Systems
Focus on building and teaching with real humanoid systems using only open tools. Content must bridge the gap between theory and practice, enabling students to build, simulate, and control actual humanoid robots using ROS 2, Gazebo, Isaac Sim, and Vision-Language-Action pipelines.

### V. Pedagogical Effectiveness
Content must follow progressive difficulty with clear learning objectives, hands-on exercises, and programming projects. Each chapter must include 5-8 exercises plus 1 programming project, with embedded executable code examples via interactive platforms like Thebe/Starlark.

### VI. Interactive and Multilingual Accessibility
Textbook must include AI-native interactive features including live code blocks, interactive quizzes, text selection explain popup, hover tooltips, accordion sections, and progress tracking. Support multiple languages including English, Urdu, and Roman Urdu with translation capabilities.

## Additional Constraints

Technology Stack: Docusaurus v3 + MDX for deployment to GitHub Pages, with Vercel backend for advanced features. ROS 2 Jazzy framework for robotics content, NVIDIA Isaac Sim 2024.1+ for simulation, Python 3.10+ for all code examples.

Deployment: Fully built with Docusaurus v3 + MDX, automatically deployed to GitHub Pages. Include subagents for content generation, personalization, translation, and quiz management.

Bonus Features: Include subagents_claude_code, better_auth_background, personalization_button, urdu_roman_translation_button, and agent_skills_rag_chatbot.

Languages: English, Urdu, and Roman Urdu.

Interactive Features: live_code_blocks, interactive_quizzes, text_selection_popup, hover_tooltips, accordion_sections, and progress_tracking.

Subagents: ContentGenerator, Personalizer, UrduTranslator, RomanUrduConverter, QuizMaster, DiagramExplainer, RagIngester.

Agent Skills: explain_concept, generate_quiz, translate_to_urdu, translate_to_roman_urdu, simplify_for_beginner, add_advanced_code, explain_diagram_vision, retrieve_rag.

Databases: Neon PostgreSQL and Qdrant Cloud for RAG functionality.

Authentication: Better Auth with background questions for enhanced security.

## Development Workflow

All content must be developed following Spec-Driven Development (SDD) methodology with constitution, specification, clarification, planning, and task generation phases. Each chapter must include learning objectives, exercises, embedded executable code, and APA 7th references. Quality validation includes plagiarism checks, code execution verification, and external review by robotics professors. The project targets 300 points guaranteed through Spec Kit Plus implementation with hackathon-focused deliverables and full RAG chatbot integration.

## Governance

This constitution supersedes all other development practices. All implementations must comply with these principles. Amendments require documentation of changes, approval process, and migration plan for existing content. All PRs and reviews must verify compliance with accuracy, reproducibility, and pedagogical effectiveness standards.

**Version**: 1.2.0 | **Ratified**: 2025-12-10 | **Last Amended**: 2025-12-10
