<!--
Sync Impact Report:
- Version change: N/A (initial creation) → 1.0.0
- Modified principles: N/A (new project)
- Added sections: All principles and sections for RAG Chatbot project
- Removed sections: N/A
- Templates requiring updates: N/A (initial creation)
- Follow-up TODOs: None
-->

# Integrated RAG Chatbot Constitution

## Core Principles

### I. Faithfulness
Responses must be grounded strictly in retrieved book content. All answers must be traceable to specific sections of the book content without deviation or assumption.

### II. Precision
Zero hallucination tolerance; if information is missing, clearly state it. When the required information is not present in the book, explicitly respond with "This information is not present in the book."

### III. Modularity
Clean separation of ingestion, retrieval, reasoning, and UI layers. Each component must be independently developable, testable, and replaceable without affecting other components.

### IV. Professional Engineering
Maintainable, documented, and testable codebase. All code must include type hints, meaningful variable names, inline docstrings for complex logic, and follow clean architecture principles.

### V. Privacy & Control
Respect user-selected text boundaries and data isolation. The system must honor the selected-text-only mode and never leak information outside the specified context.

### VI. AI-Native Design
Built from the ground up for RAG-first workflows. All components must be designed specifically for retrieval-augmented generation patterns.

## Technical Constraints

### LLM & AI Requirements
- Primary LLM Provider: Cohere (Command / Command-R / Command-R+)
- OpenAI APIs must NOT be used for generation
- Embeddings: Cohere Embeddings API only
- Reasoning must be retrieval-first, never purely generative

### Architecture Standards
- Backend Framework: FastAPI (Python)
- API Design: RESTful with clear schema definitions (Pydantic)
- Vector Database: Qdrant Cloud (Free Tier)
- Relational Database: Neon Serverless PostgreSQL
- RAG Pattern: Query → Retrieve (Qdrant) → Context Filter → Generate (Cohere)
- Context Window Management: Strict token budgeting

### Data & Retrieval Rules
- Source of truth: Book content only
- Each response must be traceable to retrieved chunks
- Chunking strategy: Semantic chunking with overlap
- Metadata: chapter, section, page, source_id for each chunk
- Top-k retrieval with relevance threshold
- Optional reranking (Cohere Rerank) when needed

### Special Feature: Selected-Text-Only Mode
- If user selects text:
  - Ignore full-book index
  - Build temporary context only from selected text
  - Answer strictly within that scope
- Clearly indicate when this mode is active

## Development Standards

### Code Quality Requirements
- Clean architecture (services, routers, schemas, utils)
- Type hints everywhere
- Inline docstrings for complex logic
- Meaningful variable and function names
- No unused code or dead imports

### Testing & Validation
- Unit tests for:
  - Retrieval logic
  - Context filtering
  - Selected-text mode
- Manual validation against sample book questions
- Hallucination checks before release

### Security & Configuration
- API keys stored via environment variables
- No hard-coded secrets
- Rate limiting and basic abuse protection enabled
- Logging without leaking sensitive data

## Documentation Requirements

### Required Documentation
- System architecture diagram
- RAG flow explanation
- API endpoint documentation
- Environment setup guide
- Limitations clearly stated

### Deployment Expectations
- Backend deployable on cloud (Railway/Fly.io/Vercel backend)
- Qdrant Cloud and Neon configured via environment variables
- Clear setup and run instructions in README

## Governance

This constitution governs all development decisions for the Integrated RAG Chatbot project. All features, modifications, and architectural decisions must comply with these principles. Amendments to this constitution require explicit documentation and approval from project stakeholders. All pull requests and code reviews must verify compliance with these principles.

**Version**: 1.0.0 | **Ratified**: 2025-01-15 | **Last Amended**: 2025-01-15
