# Data Model for Landing Page Components

## LandingPageSection
- id: string (UUID)
- section_type: enum (supportive_hero, social_proof, tools_grid, feature_cards, final_cta)
- title: string
- content: string (markdown/HTML)
- order: integer
- is_active: boolean
- created_at: timestamp
- updated_at: timestamp

## SupportiveHeroContent
- id: string (UUID)
- image_src: string (URL path)
- image_alt: string
- image_width: integer
- title: string
- description: string (markdown)
- highlights: array of strings
- created_at: timestamp
- updated_at: timestamp

## SocialProofLogo
- id: string (UUID)
- organization_name: string
- logo_src: string (URL path)
- display_order: integer
- is_active: boolean
- created_at: timestamp
- updated_at: timestamp

## ToolItem
- id: string (UUID)
- name: string
- icon: string (icon identifier)
- display_order: integer
- is_active: boolean
- created_at: timestamp
- updated_at: timestamp

## FeatureCard
- id: string (UUID)
- title: string
- description: string
- icon: string (icon identifier)
- display_order: integer
- created_at: timestamp
- updated_at: timestamp

## CallToAction
- id: string (UUID)
- text: string
- link_url: string
- button_type: enum (primary, secondary)
- tracking_id: string
- created_at: timestamp
- updated_at: timestamp

## LandingPageConfiguration
- id: string (UUID)
- theme: string (dark_cyberpunk, light_academic, etc.)
- accent_color: string (hex color code)
- preserve_header: boolean
- preserve_hero: boolean
- preserve_footer: boolean
- is_active: boolean
- created_at: timestamp
- updated_at: timestamp