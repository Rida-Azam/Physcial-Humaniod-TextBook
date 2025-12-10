# Spec: RAG Chatbot for Physical AI Textbook

**Feature Branch**: `03-rag-chatbot`

**Created**: 2025-12-10

**Status**: Draft

## Overview

Create a Retrieval Augmented Generation (RAG) chatbot system with OpenAI Agents SDK, ChatKit embed, Qdrant vector database for book content, Neon PostgreSQL for user context, and specialized agent skills for textbook interaction. The system will prioritize selected text as context when provided and include an ingestion script for book content.

## User Scenarios & Testing

### Primary User Scenario
As a student using the Physical AI & Humanoid Robotics textbook, I want to interact with an AI assistant that can answer questions about the textbook content, explain concepts, and generate quizzes based on specific sections, so that I can have a personalized and enhanced learning experience.

1. User selects text in a textbook chapter and clicks "Explain" or "Ask"
2. Frontend sends selected text context to the RAG system
3. RAG system prioritizes the selected text and retrieves related content
4. OpenAI agent processes the query with the context
5. User receives a relevant response with citations
6. If needed, user can request concept explanations or quizzes

### Acceptance Scenarios
- [ ] User can ask questions about textbook content and receive accurate answers
- [ ] When text is selected, the system prioritizes that content in responses
- [ ] System can explain complex concepts from the textbook
- [ ] System can generate relevant quizzes based on textbook content
- [ ] User context from Neon PostgreSQL is used for personalization
- [ ] ChatKit widget is embedded and functional in the frontend
- [ ] Book content is properly ingested and chunked for RAG retrieval

## Functional Requirements

### FR-1: OpenAI Agents SDK Integration
- The system must use GPT-4o Mini as the agent for decision-making
- Agent must be able to process natural language queries about textbook content
- Agent must follow conversation context across multiple exchanges
- Agent must be able to delegate tasks to specialized skills when appropriate

### FR-2: ChatKit Embed
- A React component must be provided for embedding the chat widget
- The widget must be positioned consistently in the UI (e.g., bottom-right)
- The widget must support real-time messaging
- The widget must display conversation history
- The widget must handle typing indicators and loading states

### FR-3: Qdrant Collection for Book Content
- Book content must be stored in Qdrant collection named "book_chunks"
- Content must be chunked with size of 800 tokens and 200-token overlap
- Vector embeddings must be generated for efficient semantic search
- Chunks must preserve document context and source information
- Indexing must be optimized for fast retrieval

### FR-4: Neon Integration for User Context
- User background information must be fetched from Neon PostgreSQL
- User preferences and learning history must be accessible for personalization
- User context must be integrated into query responses when relevant
- User data must be accessed securely with proper authentication
- User context must be updated based on interactions

### FR-5: Selected Text Priority Context
- When selected text is provided, it must be prioritized as primary context
- The system must retrieve related chunks based on the selected text
- Only the selected text and related chunks should be used for response generation
- The priority context must be clearly distinguished from general context

### FR-6: Agent Skills
- The system must implement a "retrieve_book" skill for fetching textbook content
- The system must implement an "explain_concept" skill for clarifying complex topics
- The system must implement a "generate_quiz" skill for creating assessment questions
- Skills must be reusable across different RAG contexts
- Skills must return structured data that can be formatted for user display

### FR-7: Query Endpoint
- The system must provide a POST endpoint at /query
- The endpoint must accept query, selected_text, and user_id parameters
- The endpoint must validate input parameters
- The endpoint must return structured responses with answers, sources, and confidence scores
- The endpoint must handle errors gracefully and return appropriate status codes

### FR-8: Ingestion Script
- A load_book.py script must be provided for ingesting book content
- The script must process book content into appropriate chunks
- The script must generate vector embeddings for each chunk
- The script must store chunks in the Qdrant collection
- The script must handle different file formats and content structures

## Non-Functional Requirements

### Performance
- Query responses must be delivered within 3 seconds for 95% of requests
- Vector search in Qdrant must complete within 500ms for 95% of queries
- Agent processing time must be optimized to minimize response latency
- Content ingestion must handle large books efficiently

### Security
- All API endpoints must validate user authentication when required
- User context data must be accessed securely with proper permissions
- OpenAI API keys must be protected and not exposed to clients
- Input validation must prevent injection attacks

### Reliability
- System must have 99% uptime for chatbot functionality
- Fallback mechanisms must be in place for when vector database is unavailable
- Error handling must provide graceful degradation of service
- Content ingestion must be resilient to partial failures

### Scalability
- System must handle concurrent users without degradation
- Vector database must scale with growing content library
- Agent processing must be optimized for cost and performance
- User context retrieval must scale with user base growth

## Key Entities

### ChatQuery
- Query text, selected text context, user ID, conversation history
- Response text, sources, confidence score, related content

### BookChunk
- Content text, embedding vector, source document, chunk position
- Metadata (chapter, section, page), related chunks, embedding quality

### UserContext
- User preferences, learning history, personalization settings
- Background information, interaction patterns, quiz history

### AgentSkill
- Skill name, parameters, execution logic, response format
- Reusability, error handling, integration with RAG system

## Dependencies

- OpenAI API for agent processing
- Qdrant Cloud for vector database
- Neon PostgreSQL for user data
- ChatKit SDK for frontend widget
- Text processing libraries for content ingestion
- Embedding models for vector generation

## Assumptions

- OpenAI GPT-4o Mini API is available and properly configured
- Qdrant Cloud provides reliable vector database service
- Neon PostgreSQL handles user context storage efficiently
- Book content is available in digital format for ingestion
- Frontend can properly embed the ChatKit component

## Success Criteria

- 90% of user queries receive relevant, accurate responses
- Average response time under 2 seconds
- 95% of selected text contexts are properly prioritized
- Users rate the chatbot helpfulness at 4.0+ stars
- 80% of generated quizzes are rated as relevant and useful
- Content ingestion processes books without significant data loss
- Zero security incidents related to user context access

## Scope

### In Scope
- OpenAI Agent integration with GPT-4o Mini
- ChatKit React component embed
- Qdrant collection with book chunks (size: 800, overlap: 200)
- Neon integration for user context
- Selected text priority context handling
- Agent skills: retrieve_book, explain_concept, generate_quiz
- Query endpoint with specified parameters
- Book content ingestion script

### Out of Scope
- Frontend UI design beyond the chat widget
- Advanced user authentication (beyond user_id)
- Real-time collaboration features
- Offline functionality
- Third-party integrations beyond those specified

## Risks

- OpenAI API rate limits affecting response times
- Vector database costs scaling with content size
- Quality of generated explanations and quizzes
- Privacy concerns with user context storage
- Accuracy of content retrieval from vector database