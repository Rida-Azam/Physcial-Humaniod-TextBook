---
id: 3
title: "Updated Constitution RAG Integration"
stage: "constitution"
date: "2025-12-10"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "physical-ai-textbook"
branch: "0001-physical-ai-textbook"
user: "user"
command: "sp.constitution"
labels: ["constitution", "rag-chatbot", "databases", "agent-skills"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - ".specify/memory/constitution.md"
tests: []
---

# Updated Constitution with RAG Chatbot Integration

## Prompt
```
# Updated SP Constitution – Includes RAG Chatbot Full Integration
constitution:
  project_name: "Physical AI & Humanoid Robotics Textbook"
  version: "1.1"  # Updated for RAG focus
  spec_kit_version: "plus"
  deployment: {frontend: github_pages, backend: vercel}
  bonuses: [subagents_claude_code, better_auth_background, personalization_button, urdu_roman_translation_button, agent_skills_rag_chatbot]
  languages: [english, urdu, roman_urdu]
  interactive_features: [live_code_blocks, interactive_quizzes, text_selection_popup, hover_tooltips, accordion_sections, progress_tracking]
  subagents: [ContentGenerator, Personalizer, UrduTranslator, RomanUrduConverter, QuizMaster, DiagramExplainer, RagIngester]  # Added RagIngester
  agent_skills: [explain_concept, generate_quiz, translate_to_urdu, translate_to_roman_urdu, simplify_for_beginner, add_advanced_code, explain_diagram_vision, retrieve_rag]  # Added retrieve_rag
  databases: {neon_postgres: true, qdrant_cloud: true}
  auth: {provider: better_auth, backgrou uodate this also
```

## Response
The constitution file has been updated to include comprehensive details about the RAG chatbot integration and additional technical specifications. The update includes:

- Version incremented from 1.1.0 to 1.2.0 to reflect the significant additions
- Added "RagIngester" to the list of subagents
- Added comprehensive agent skills: explain_concept, generate_quiz, translate_to_urdu, translate_to_roman_urdu, simplify_for_beginner, add_advanced_code, explain_diagram_vision, retrieve_rag
- Added database specifications: Neon PostgreSQL and Qdrant Cloud for RAG functionality
- Enhanced authentication details: Better Auth with background questions for enhanced security
- Updated bonus features to use the new naming conventions
- Updated interactive features to use the new naming conventions
- Updated the Development Workflow to mention full RAG chatbot integration

The constitution now fully reflects the advanced RAG capabilities and backend infrastructure of the textbook project.

## Outcome
The constitution has been successfully updated to version 1.2.0 with comprehensive details about RAG chatbot integration, database specifications, enhanced authentication, and expanded agent skills for the Physical AI & Humanoid Robotics textbook project.