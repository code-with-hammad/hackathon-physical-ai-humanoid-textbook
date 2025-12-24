# API Contract: Integrated RAG Chatbot

**Feature**: 001-integrated-rag-chatbot  
**Date**: 2025-01-15

## Query Endpoints

### POST /api/v1/query
Submit a question to the RAG system for response generation

**Request Body:**
```json
{
  "query": "What is the main concept discussed in chapter 3?",
  "selected_text": "Optional selected text for selected-text-only mode",
  "context": {
    "current_chapter": "Chapter 3",
    "current_section": "3.2 Advanced Concepts"
  }
}
```

**Response (200 OK):**
```json
{
  "answer": "The main concept discussed in chapter 3 is...",
  "sources": [
    {
      "id": "chunk-123",
      "content": "The primary concept in this chapter is...",
      "chapter": "Chapter 3",
      "section": "3.1 Basic Principles",
      "page": 45,
      "score": 0.89
    }
  ],
  "references": [
    "Chapter 3, Section 3.1, page 45",
    "Chapter 3, Section 3.2, page 47"
  ],
  "confidence": 0.92,
  "mode": "full-book" // or "selected-text-only"
}
```

**Response (404 Not Found):**
```json
{
  "answer": "This information is not present in the book",
  "sources": [],
  "references": [],
  "confidence": 0.0,
  "mode": "full-book" // or "selected-text-only"
}
```

**Validation:**
- `query` is required and must be a non-empty string
- `selected_text` is optional, if provided activates selected-text-only mode
- `context` is optional object with chapter/section information

---

### POST /api/v1/ingest
Ingest book content into the system and create vector embeddings

**Request Body:**
```json
{
  "book_id": "unique-book-identifier",
  "title": "Book Title",
  "content": "Full text content of the book...",
  "chapters": [
    {
      "id": "chapter-1",
      "title": "Chapter 1 Title",
      "content": "Content of chapter 1...",
      "page_start": 1,
      "page_end": 25
    }
  ]
}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "message": "Book content ingested and indexed successfully",
  "chunks_processed": 150,
  "book_id": "unique-book-identifier"
}
```

**Response (400 Bad Request):**
```json
{
  "status": "error",
  "message": "Invalid input data"
}
```

**Validation:**
- `book_id` is required and must be a valid identifier
- `title` and `content` are required
- `chapters` is optional array of chapter objects

---

### GET /api/v1/health
Check the health status of the API

**Response (200 OK):**
```json
{
  "status": "healthy",
  "timestamp": "2025-01-15T10:30:00Z",
  "dependencies": {
    "qdrant": "connected",
    "cohere": "connected"
  }
}
```