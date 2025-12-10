# Pull Request Instructions

A pull request needs to be created to merge the changes from the `0001-physical-ai-textbook` branch to the `main` branch.

## To create the pull request manually:

1. Go to: https://github.com/Rida-Azam/Physcial-Humaniod-TextBook/compare/main...0001-physical-ai-textbook

2. Set the base branch to: `main`
3. Set the compare branch to: `0001-physical-ai-textbook`

4. Use the following title:
   ```
   feat: Add Docusaurus frontend and FastAPI backend structure
   ```

5. Use the following description:
   ```
   This PR introduces the complete restructure of the Physical AI & Humanoid Robotics textbook project with:

   - Docusaurus v3 frontend with interactive components (Chatbot, PersonalizeButton, TranslateToggle)
   - FastAPI backend with RAG services, Qdrant vector database, and Neon PostgreSQL
   - All textbook content organized in frontend/docs
   - Specification files for all components
   - Constitution updates with RAG and multilingual features
   - GitHub Pages deployment configuration

   The project now follows the requested structure with clear separation between frontend and backend components.
   ```

6. Review the changes and create the pull request.

## Changes included:
- 56 files changed with 11,340 insertions and 38 deletions
- Complete project restructure with frontend/backend separation
- New Docusaurus frontend implementation
- FastAPI backend with RAG services
- All textbook content and interactive features
- Specification files for all components