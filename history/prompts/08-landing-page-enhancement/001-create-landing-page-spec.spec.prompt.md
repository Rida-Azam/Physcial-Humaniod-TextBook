---
id: 1
title: "Create Landing Page Spec"
stage: "spec"
date: "2025-12-10"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "landing-page-enhancement"
branch: "08-landing-page-enhancement"
user: "user"
command: "sp.specify"
labels: ["landing-page", "ui", "ux", "design"]
links:
  spec: "specs/08-landing-page-enhancement/spec.md"
  ticket: null
  adr: null
  pr: null
files:
  - "specs/08-landing-page-enhancement/spec.md"
  - "specs/08-landing-page-enhancement/checklists/requirements.md"
tests: []
---

# Create Landing Page Enhancement Specification

## Prompt
```
component: landing_page_enhancement
constitution: true
theme: dark_cyberpunk_#00D4FF_accent
output: frontend/src/pages/index.tsx

hero:
  title: "Physical AI & Humanoid Robotics"
  subtitle: "The Complete 2025 Curriculum – From ROS 2 to Vision-Language-Action Models"
  background: "linear-gradient(135deg, #0A0A0A 0%, #001122 100%)"
  cta_primary: "Start Reading Free →"
  cta_secondary: "Watch Capstone Demo"

# THIS IS THE NEW SECTION — ADDED RIGHT AFTER HERO
supportive_section:
  layout: full_width_image_left
  image:
    src: "/img/hero-humanoid-2025.png"   # Put your image in static/img/
    alt: "Tesla Optimus Gen 3 walking with VLA brain – Powered by Isaac Sim + GPT-4o"
    width: 650
  content:
    title: "Learn to Build Autonomous Humanoids in 2025"
    description: |
      Master ROS 2, NVIDIA Isaac Sim, Vision-Language-Action (VLA) models,
      and deploy voice-controlled humanoids that clean rooms, fold laundry,
      and navigate real-world environments — all in simulation first, then real hardware.
    highlights:
      - 12 Industry-Standard Modules
      - Full Capstone: Voice → Plan → Act Humanoid
      - Urdu + Roman Urdu Support
      - Live Code + Interactive Quizzes
      - RAG Chatbot with Selected-Text Explain

social_proof:
  title: "Trusted by the Leaders in Robotics & AI"
  logos:
    - name: "Panaversity"          src: "/img/logos/panaversity.png"
    - name: "PIAIC"                src: "/img/logos/piaic.png"
    - name: "NVIDIA Isaac"         src: "/img/logos/nvidia-isaac.png"
    - name: "Tesla Optimus"        src: "/img/logos/tesla-optimus.png"
    - name: "Boston Dynamics"      src: "/img/logos/bd.png"
    - name: "Figure AI"            src: "/img/logos/figure.png"

tools_used:
  title: "Built with the Exact 2025 Stack"
  tools:
    - name: "ROS 2 Humble"           icon: "ros"
    - name: "NVIDIA Isaac Sim"       icon: "nvidia"
    - name: "GPT-4o Realtime API"    icon: "openai"
    - name: "OpenAI ChatKit"         icon: "chatkit"
    - name: "Qdrant Vector DB"       icon: "qdrant"
    - name: "Gazebo Ignition"        icon: "gazebo"
    - name: "Unity Robotics"         icon: "unity"
    - name: "Claude Subagents"       icon: "anthropic"

additional_info:
  title: "Why This Course is Different"
  cards:
    - title: "Capstone Project"
      description: "Build a full VLA humanoid that responds to voice commands"
      icon: "robot"
    - title: "Urdu + Roman Urdu"
      description: "Full translation toggle – first in Pakistan"
      icon: "language"
    - title: "Live Coding"
      description: "Edit & run ROS 2 + Python code directly in browser"
      icon: "code"
    - title: "RAG Chatbot"
      description: "Select any text → click Explain → perfect answer"
      icon: "chat"

cta_final:
  title: "Ready to Build the Future?"
  button: "Start Learning Now – 100% Free"
```

## Response
Created a comprehensive specification for the landing page enhancement with dark cyberpunk theme and #00D4FF accent. The specification includes:

- Dark cyberpunk theme implementation with specified color scheme
- Hero section with title, subtitle, and CTAs
- Supportive section with image and content layout
- Social proof section with trusted logos
- Tools used section showing 2025 stack
- Differentiators cards explaining unique features
- Final CTA section
- Responsive design requirements

The spec defines user scenarios, functional requirements, success criteria, and scope boundaries. It also includes a validation checklist to ensure quality before proceeding to planning phase.

## Outcome
Successfully created the landing page enhancement specification with all required features. The specification is ready for clarification and planning phases.