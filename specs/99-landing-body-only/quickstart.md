# Quickstart Guide: Landing Page Enhancement

## Overview
This guide will help you implement the enhanced landing page with dark cyberpunk theme and preserved header/hero/footer sections.

## Prerequisites
- Node.js 18+ installed
- Docusaurus project set up
- Access to image assets
- Understanding of React/JSX

## Setup Steps

### 1. Install Dependencies
```bash
cd frontend
npm install
```

### 2. Create Image Directories
```bash
mkdir -p static/img
mkdir -p static/img/logos
```

### 3. Add Required Images
Place the following images in the appropriate directories:
- `static/img/hero-humanoid-2025.png` - Main supportive hero image
- `static/img/logos/panaversity.png` - Panaversity logo
- `static/img/logos/piaic.png` - PIAIC logo
- `static/img/logos/nvidia-isaac.png` - NVIDIA Isaac logo
- `static/img/logos/tesla-optimus.png` - Tesla Optimus logo
- `static/img/logos/bd.png` - Boston Dynamics logo
- `static/img/logos/figure.png` - Figure AI logo

### 4. Update Main Page
Replace the content in `frontend/src/pages/index.tsx` with the enhanced version that includes all new sections while preserving the existing header/hero/footer.

### 5. Add CSS Styles
Ensure your `src/css/custom.css` includes the dark cyberpunk theme with #00D4FF accent color:

```css
/* Dark Cyberpunk Theme */
.hero--primary {
  background: linear-gradient(135deg, #0A0A0A 0%, #001122 100%);
}

.button--primary {
  background-color: #00D4FF;
  border-color: #00D4FF;
}

.button--primary:hover {
  background-color: #00b8e6;
  border-color: #00b8e6;
}

/* Additional cyberpunk styling for new sections */
.supportive-section {
  background-color: #0a0a0a;
  padding: 2rem 0;
}

.social-proof-section {
  background-color: #0f0f0f;
  padding: 2rem 0;
}

.tools-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
  gap: 1.5rem;
  padding: 2rem 0;
}

.feature-cards {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 1.5rem;
  padding: 2rem 0;
}
```

### 6. Environment Variables (if using API)
If you plan to use the API contracts for dynamic content:
```bash
# Add to your .env file
REACT_APP_LANDING_API_URL=your_api_url_here
```

### 7. Run Development Server
```bash
npm start
```

## Key Implementation Points

### Preserving Existing Elements
- The header and hero sections will remain unchanged
- Only new sections will be added after the hero
- Footer will remain at the bottom

### New Sections to Add
1. **Supportive Hero Section** - Full-width image with content layout
2. **Social Proof Section** - Trusted organization logos
3. **Tools Grid** - 2025 technology stack
4. **Feature Cards** - Unique value propositions
5. **Final CTA** - Strong call-to-action

### Responsive Design
- All sections are designed to be responsive
- Mobile-first approach with appropriate breakpoints
- Touch-friendly elements for mobile users

## Testing Checklist
- [ ] All new sections display correctly
- [ ] Dark cyberpunk theme applied consistently
- [ ] #00D4FF accent color used appropriately
- [ ] Existing header/hero/footer preserved
- [ ] All images load correctly
- [ ] Page is responsive on mobile devices
- [ ] All CTAs are functional
- [ ] Social proof logos display properly
- [ ] Tools grid shows correctly
- [ ] Feature cards layout works

## Next Steps
1. Customize content with your specific information
2. Add analytics tracking for CTAs
3. Implement A/B testing for different layouts
4. Add performance optimization
5. Deploy to production