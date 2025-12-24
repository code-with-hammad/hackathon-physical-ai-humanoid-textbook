# Feature Specification: Integrated RAG Chatbot

**Feature Branch**: `001-integrated-rag-chatbot`
**Created**: 2025-01-15
**Status**: Draft
**Input**: User description: "Integrated RAG Chatbot for an AI-Native Interactive Book Target audience: - Readers of the published book - Developers embedding AI assistants into documentation or books - Technical reviewers evaluating RAG system quality Primary goal: Design and implement a production-grade Retrieval-Augmented Generation (RAG) chatbot embedded inside a digital book, capable of answering questions strictly from book content, including a selected-text-only answering mode. Functional scope: - Full-book question answering using RAG - Selected-text-only question answering (strict isolation) - Chapter/section-aware responses - Clear not found in book fallback behavior Technology stack (MANDATORY): - Backend: FastAPI (Python) - LLM Provider: Cohere (Command / Command-R family only) - Embeddings: Cohere Embeddings API - Vector DB: Qdrant Cloud (Free Tier) - Relational DB: Neon Serverless PostgreSQL - Deployment-ready cloud architecture Credential handling rules: - All API keys and database URLs must be loaded from environment variables - No secrets may appear in source code or prompts - Configuration via .env or platform environment settings only RAG behavior requirements: - Retrieval-first generation (no free-form answers) - Context strictly limited to retrieved chunks - Metadata-aware responses (chapter/section/page when available) - Optional reranking for higher answer precision - Token-efficient context window management Selected-text-only mode: - Activated only when user provides selected text - Ignores global vector index - Builds temporary context from user-provided text only - Must never leak information outside selected content Success criteria: - Answers are fully grounded in book content - Zero hallucinations accepted - Selected-text mode respects strict boundaries - Stable performance on free-tier infrastructure - Clean, readable, production-quality code - Ready for embedding into a live published book Constraints: - No OpenAI API usage for generation - No hard-coded credentials - No external knowledge beyond the book - No unnecessary overengineering Not building: - A general web search chatbot - Multi-book or internet-wide retrieval - Authentication or payment systems - UI-heavy frontend (focus on backend + integration readiness)"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Full-book question answering (Priority: P1)

A reader wants to ask questions about the book content and receive accurate answers based only on the book's content, with proper attribution to the relevant chapters/sections.

**Why this priority**: This represents the core functionality of the RAG chatbot - allowing readers to interact with the book content through questions.

**Independent Test**: Reader can ask a specific question about the book content and receive a response that is grounded only in the book's content with appropriate chapter/section references.

**Acceptance Scenarios**:

1. **Given** a user has access to the book content, **When** they ask a question about the book content, **Then** the system returns an answer based only on the book content with chapter/section references.
2. **Given** a user asks a question not found in the book, **When** they submit the question, **Then** the system responds with "This information is not present in the book".

---

### User Story 2 - Selected-text-only question answering (Priority: P2)

A reader wants to select specific text from the book and ask questions only about that selected text, without the system pulling from other parts of the book.

**Why this priority**: This provides a more focused interaction mode where users can get answers specifically about text they've highlighted.

**Independent Test**: Reader can select text from the book content, ask a question about that text, and receive a response that only considers the selected text.

**Acceptance Scenarios**:

1. **Given** a user has selected text from the book, **When** they ask a question in selected-text-only mode, **Then** the system returns an answer based only on the selected text.
2. **Given** a user is in selected-text-only mode, **When** they ask a question that requires information outside the selected text, **Then** the system responds that the information is not in the selected text.

---

### User Story 3 - Chapter/section-aware responses (Priority: P3)

A reader wants to receive responses that are aware of the chapter/section context, providing relevant information based on the specific part of the book they're interacting with.

**Why this priority**: This enhances the contextual understanding of the chatbot, making responses more relevant to the specific section the user is reading.

**Independent Test**: When the user is reading in a specific chapter/section, the system prioritizes responses that are relevant to that context.

**Acceptance Scenarios**:

1. **Given** a user is viewing a specific chapter/section of the book, **When** they ask a question related to the current context, **Then** the system provides responses that are prioritized based on the current chapter/section.

---

### Edge Cases

- What happens when the user's question requires information from multiple sections of the book?
- How does the system handle queries that span across chapters?
- What happens if the selected text is too short to provide meaningful context?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST answer questions based only on the book's content using RAG methodology
- **FR-002**: System MUST provide a selected-text-only mode that ignores global vector index
- **FR-003**: System MUST return "This information is not present in the book" when information is not found
- **FR-004**: System MUST provide chapter/section references when answering questions
- **FR-005**: System MUST not use any external knowledge beyond the book content
- **FR-006**: System MUST use Cohere APIs only (no OpenAI) for embeddings and generation
- **FR-007**: System MUST load all API keys and database URLs from environment variables
- **FR-008**: System MUST implement optional reranking for higher answer precision
- **FR-009**: System MUST manage token-efficient context window management
- **FR-010**: System MUST never leak information outside the selected content in selected-text-only mode

### Key Entities *(include if feature involves data)*

- **BookContent**: The complete book content with chapter/section structure
- **RetrievedChunk**: Semantic chunks of book content with metadata (chapter, section, page, source_id)
- **UserQuery**: Query input from the user with optional selected text context
- **Response**: Generated response with source attribution and confidence level
- **SessionContext**: Current user interaction state (selected-text mode, current chapter, etc.)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of responses are grounded in book content with proper attribution
- **SC-002**: 0% hallucination rate in responses
- **SC-003**: Selected-text-only mode respects strict boundaries with 100% accuracy
- **SC-004**: System performs stably on Qdrant Cloud Free Tier with response times under 5 seconds
- **SC-005**: Users can successfully ask questions and receive relevant answers based on book content 95% of the time
- **SC-006**: Clear indication when information is not present in the book is provided 100% of the time