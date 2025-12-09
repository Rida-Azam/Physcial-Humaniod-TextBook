---
id: 2
title: "Updated Constitution AI Native Features"
stage: "constitution"
date: "2025-12-10"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "physical-ai-textbook"
branch: "0001-physical-ai-textbook"
user: "user"
command: "sp.constitution"
labels: ["constitution", "ai-native", "multilingual", "subagents"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - ".specify/memory/constitution.md"
tests: []
---

# Updated Constitution with AI-Native Features

## Prompt
```
# =====================================================
# OFFICIAL SP CONSTITUTION – Physical AI & Humanoid Textbook
# Hackathon I – 300/300 Points Guaranteed
# =====================================================

constitution:
  project_name: "Physical AI & Humanoid Robotics – AI-Native Textbook"
  version: "1.0.0"
  target_score: 300
  spec_kit_version: "plus"
  deployment:
    frontend: github_pages
    backend: vercel

  bonuses:
    - subagents_claude_code
    - better_auth_with_background_questions
    - personalization_button_per_chapter
    - urdu_and_roman_urdu_translation_button
    - agent_skills_in_rag_chatbot

  languages:
    - english
    - urdu
    - roman_urdu

  interactive_features:
    - live_code_blocks
    - interactive_quizzes
    - text_selection_explain_popup
    - hover_tooltips
    - accordion_sections
    - progress_tracking

  subagents:
    - ContentGenerator
    - Personalizer
    - UrduTranslator
    - RomanUrduConverter
    - QuizMaster
    - DiagramExplainer

  agent_
```

## Response
The constitution file has been updated to include specific details about the AI-native features of the Physical AI & Humanoid Robotics textbook. The update includes:

- Version incremented from 1.0.0 to 1.1.0
- Added specific bonus features: subagents_claude_code, better_auth_with_background_questions, personalization_button_per_chapter, urdu_and_roman_urdu_translation_button, and agent_skills_in_rag_chatbot
- Added language support for English, Urdu, and Roman Urdu
- Added interactive features: live_code_blocks, interactive_quizzes, text_selection_explain_popup, hover_tooltips, accordion_sections, and progress_tracking
- Added subagents: ContentGenerator, Personalizer, UrduTranslator, RomanUrduConverter, QuizMaster, DiagramExplainer
- Updated the Development Workflow to mention the 300-point target through Spec Kit Plus implementation with hackathon-focused deliverables

The constitution now fully reflects the AI-native nature of the textbook with multilingual support and advanced interactive capabilities.

## Outcome
The constitution has been successfully updated to version 1.1.0 with comprehensive details about AI-native features, multilingual support, and interactive capabilities for the Physical AI & Humanoid Robotics textbook project.