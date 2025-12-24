# Research for Integrated RAG Chatbot

**Feature**: 001-integrated-rag-chatbot  
**Date**: 2025-01-15

## Decision: Technology Stack

**Rationale**: The technology stack is mandated by the constitution and spec, which requires:
- Backend: FastAPI (Python)
- LLM Provider: Cohere (Command / Command-R family only)
- Embeddings: Cohere Embeddings API
- Vector DB: Qdrant Cloud (Free Tier)
- Relational DB: Neon Serverless PostgreSQL

**Alternatives considered**: Various other LLM providers like OpenAI, Anthropic, etc. were considered but rejected due to the hard requirement to use only Cohere APIs.

## Decision: Architecture Pattern

**Rationale**: Following the RAG pattern mandated by the constitution: Query → Retrieve (Qdrant) → Context Filter → Generate (Cohere). This ensures a clean separation of concerns between retrieval and generation components.

**Alternatives considered**: Direct generation without retrieval, but this would violate the "Faithfulness" and "Precision" principles requiring responses to be grounded in book content.

## Decision: Selected-Text-Only Mode Implementation

**Rationale**: Implement as a temporary in-memory context that bypasses the global Qdrant index. This ensures strict isolation as required by the "Privacy & Control" principle.

**Alternatives considered**: Storing selected text in database, but this would be unnecessary complexity and potential security risk.

## Decision: Chunking Strategy

**Rationale**: Using semantic chunking with overlap to preserve context across chunks while maintaining the ability to retrieve relevant information. Chunk metadata will include chapter, section, page, and source_id as required by the constitution.

**Alternatives considered**: Fixed-size chunking without semantic awareness, but this could break important contextual relationships.

## Decision: Security Implementation

**Rationale**: All API keys loaded from environment variables with no hard-coded credentials, and no sensitive data in logs, per security requirements.

**Alternatives considered**: Configuration files for credentials, but environment variables are more secure and align with cloud deployment practices.