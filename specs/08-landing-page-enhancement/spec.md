# Spec: Landing Page Enhancement for Physical AI & Humanoid Robotics

**Feature Branch**: `08-landing-page-enhancement`

**Created**: 2025-12-10

**Status**: Draft

## Overview

Create an enhanced landing page for the Physical AI & Humanoid Robotics textbook with a dark cyberpunk theme and #00D4FF accent color. The page will include a hero section, supportive content section with image, social proof logos, tools used section, differentiators cards, and a final call-to-action.

## User Scenarios & Testing

### Primary User Scenario
As a potential student or educator interested in robotics and AI, I want to visit the landing page and immediately understand the value proposition of the Physical AI & Humanoid Robotics textbook, so that I can decide to start learning or recommend it to others.

1. User lands on the homepage and sees the hero section with title and subtitle
2. User scrolls down to see the supportive section with image and key features
3. User notices the social proof section with trusted brand logos
4. User sees the tools used section showing the 2025 technology stack
5. User reads the differentiators cards explaining unique features
6. User clicks one of the CTAs to start learning or watch a demo

### Acceptance Scenarios
- [ ] Hero section displays title "Physical AI & Humanoid Robotics" and subtitle
- [ ] Page uses dark cyberpunk theme with #00D4FF accent color
- [ ] Supportive section shows image on left with content on right
- [ ] Social proof section displays logos of trusted organizations
- [ ] Tools section shows the 2025 technology stack used
- [ ] Differentiators cards explain unique value propositions
- [ ] Multiple CTAs are available (Start Reading Free, Watch Demo, etc.)
- [ ] Page is responsive and works on all device sizes

## Functional Requirements

### FR-1: Dark Cyberpunk Theme Implementation
- The landing page must use a dark theme with background gradient: linear-gradient(135deg, #0A0A0A 0%, #001122 100%)
- The accent color must be #00D4FF for buttons, links, and highlights
- All text must have proper contrast against the dark background
- Theme must be consistent across all elements of the landing page
- Color scheme must align with the "cyberpunk" aesthetic

### FR-2: Hero Section
- The hero section must display "Physical AI & Humanoid Robotics" as the main title
- The subtitle must be "The Complete 2025 Curriculum – From ROS 2 to Vision-Language-Action Models"
- Primary CTA button must say "Start Reading Free →"
- Secondary CTA button must say "Watch Capstone Demo"
- Both CTAs must be prominently displayed and clickable
- Hero section must be visually striking and immediately communicate value

### FR-3: Supportive Section
- Layout must be full-width with image on the left and content on the right
- Image source must be "/img/hero-humanoid-2025.png"
- Image alt text must describe "Tesla Optimus Gen 3 walking with VLA brain – Powered by Isaac Sim + GPT-4o"
- Content title must be "Learn to Build Autonomous Humanoids in 2025"
- Description must explain mastering ROS 2, NVIDIA Isaac Sim, VLA models, etc.
- Highlights list must include: 12 Industry-Standard Modules, Full Capstone, Urdu + Roman Urdu Support, Live Code + Interactive Quizzes, RAG Chatbot with Selected-Text Explain

### FR-4: Social Proof Section
- Section title must be "Trusted by the Leaders in Robotics & AI"
- Must display logos for Panaversity, PIAIC, NVIDIA Isaac, Tesla Optimus, Boston Dynamics, and Figure AI
- All logos must be accessible via "/img/logos/[name].png" path
- Logos must be properly sized and aligned
- Section must build credibility and trust

### FR-5: Tools Used Section
- Section title must be "Built with the Exact 2025 Stack"
- Must display tools: ROS 2 Humble, NVIDIA Isaac Sim, GPT-4o Realtime API, OpenAI ChatKit, Qdrant Vector DB, Gazebo Ignition, Unity Robotics, Claude Subagents
- Each tool must have an appropriate icon representation
- Section must demonstrate technical credibility and modern approach

### FR-6: Additional Info Cards
- Section title must be "Why This Course is Different"
- Must include 4 cards: Capstone Project, Urdu + Roman Urdu, Live Coding, RAG Chatbot
- Each card must have title, description, and icon
- Cards must clearly communicate unique value propositions
- Visual design must be consistent with the cyberpunk theme

### FR-7: Final CTA Section
- Section title must be "Ready to Build the Future?"
- CTA button must say "Start Learning Now – 100% Free"
- Button must be prominent and encourage action
- Section must create urgency and clear next step

## Non-Functional Requirements

### Performance
- Page must load within 3 seconds on standard internet connections
- Images must be properly optimized for web delivery
- All assets must be cached appropriately
- JavaScript execution must not block page rendering

### Accessibility
- All content must meet WCAG 2.1 AA standards
- Sufficient color contrast between text and background
- Proper alt text for all images
- Keyboard navigable interface
- Screen reader compatibility

### Compatibility
- Page must render correctly across all modern browsers
- Responsive design for mobile, tablet, and desktop
- Consistent experience across different devices
- Proper fallbacks for missing assets

### Maintainability
- Code structure must allow for easy updates to content
- Theme variables must be centralized for easy theming
- Component-based architecture for reusability
- Proper documentation for future maintenance

## Key Entities

### LandingPageSection
- Section type (hero, supportive, social_proof, tools, additional_info, cta)
- Content elements, styling properties, layout configuration
- Responsive behavior, accessibility attributes

### CallToAction
- Button text, link destination, styling
- Primary/secondary designation, tracking parameters
- Visual feedback on interaction

### SocialProofLogo
- Logo source, alt text, organization name
- Display size, positioning, accessibility attributes

### FeatureCard
- Title, description, icon representation
- Styling, layout, interaction behavior

## Dependencies

- Docusaurus framework for page routing
- React for component implementation
- CSS/SCSS for styling with theme variables
- Image assets for logos and hero image
- Icon library for tool representations

## Assumptions

- Image assets will be provided in the specified paths
- Icon library is available for tool representations
- Users have modern browsers that support CSS gradients
- Internet connectivity is available for asset loading
- Users are familiar with the mentioned technologies (ROS, NVIDIA Isaac, etc.)

## Success Criteria

- 80% of visitors click at least one CTA button on the landing page
- Page load time under 3 seconds for 95% of visits
- 90% of users find the value proposition clear within 10 seconds
- Mobile responsiveness validated across major devices
- All accessibility standards met (WCAG 2.1 AA)
- Social proof logos clearly displayed and recognizable
- 100% of links and CTAs function correctly
- Theme consistency maintained across all sections

## Scope

### In Scope
- Dark cyberpunk theme implementation with #00D4FF accent
- Hero section with title, subtitle, and CTAs
- Supportive section with image and content layout
- Social proof section with trusted logos
- Tools used section showing 2025 stack
- Differentiators cards explaining unique features
- Final CTA section
- Responsive design for all device sizes

### Out of Scope
- Backend functionality beyond static page
- User account creation or management
- Content management system integration
- Advanced animations beyond basic interactions
- SEO optimization beyond basic meta tags

## Risks

- Image assets may not be available at specified paths
- Color contrast may not meet accessibility requirements
- Responsive design may break on certain screen sizes
- Social proof logos may not be available or properly licensed
- Performance issues with asset loading