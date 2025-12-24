# Data Model: Integrated RAG Chatbot

**Feature**: 001-integrated-rag-chatbot  
**Date**: 2025-01-15

## BookContent
The complete book content with chapter/section structure

**Fields:**
- id: str - Unique identifier for the book
- title: str - Title of the book
- content: str - Full text content of the book
- chapters: List[Chapter] - List of chapters with their content
- metadata: Dict[str, Any] - Additional book metadata

## Chapter
A chapter within the book

**Fields:**
- id: str - Unique identifier for the chapter
- title: str - Title of the chapter
- content: str - Text content of the chapter
- section: List[Section] - List of sections within the chapter
- page_start: int - Starting page number
- page_end: int - Ending page number

## Section
A section within a chapter

**Fields:**
- id: str - Unique identifier for the section
- title: str - Title of the section
- content: str - Text content of the section
- page_number: int - Page number in the book

## RetrievedChunk
Semantic chunks of book content with metadata (chapter, section, page, source_id)

**Fields:**
- id: str - Unique identifier for the chunk
- content: str - Text content of the chunk
- source_id: str - ID of the original source (book/chapter/section)
- chapter: str - Chapter name/identifier
- section: str - Section name/identifier
- page: int - Page number where chunk originates
- embedding: List[float] - Vector embedding of the content
- score: float - Relevance score for the chunk

## UserQuery
Query input from the user with optional selected text context

**Fields:**
- query: str - The user's question
- selected_text: Optional[str] - Optional selected text for selected-text-only mode
- context: Optional[str] - Additional context information
- user_id: Optional[str] - Optional user identifier
- metadata: Dict[str, Any] - Additional query metadata

## Response
Generated response with source attribution and confidence level

**Fields:**
- answer: str - The generated answer
- sources: List[RetrievedChunk] - List of chunks used to generate the answer
- confidence: float - Confidence level of the response
- references: List[str] - Chapter/section references
- mode: str - The mode used ("full-book" or "selected-text-only")
- metadata: Dict[str, Any] - Additional response metadata

## SessionContext
Current user interaction state (selected-text mode, current chapter, etc.)

**Fields:**
- session_id: str - Unique identifier for the session
- selected_text_mode: bool - Whether selected-text-only mode is active
- current_context: Optional[str] - Current chapter/section context
- user_preferences: Dict[str, Any] - User preferences for the session
- history: List[Dict] - Conversation history