# Integrated RAG Chatbot for AI-Native Interactive Book

This project implements a production-grade Retrieval-Augmented Generation (RAG) chatbot embedded inside a digital book, capable of answering questions strictly from book content, including a selected-text-only answering mode.

## Features

- Full-book question answering using RAG methodology
- Selected-text-only question answering with strict isolation
- Chapter/section-aware responses
- Clear "not found in book" fallback behavior
- Zero hallucination tolerance

## Tech Stack

- **Backend**: FastAPI (Python)
- **LLM Provider**: Cohere (Command-R+)
- **Embeddings**: Cohere Embeddings API
- **Vector DB**: Qdrant Cloud (Free Tier)
- **Relational DB**: Neon Serverless PostgreSQL

## Architecture

The system follows the RAG pattern: Query → Retrieve (Qdrant) → Context Filter → Generate (Cohere)

### Core Components

- `src/models/` - Data models for book content, chunks, queries, and responses
- `src/services/` - Business logic for ingestion, retrieval, generation, and context filtering
- `src/api/` - FastAPI endpoints and schemas
- `src/utils/` - Utility functions for configuration, logging, token management, and chunking

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Configure environment variables in `.env`:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   NEON_DB_URL=your_neon_db_connection_string
   DEBUG=false
   LOG_LEVEL=info
   ```

3. Run the application:
   ```bash
   uvicorn src.main:app --reload --port 8000
   ```

## API Endpoints

- `POST /api/v1/query` - Submit a question to the RAG system
- `POST /api/v1/ingest` - Ingest book content into the system
- `GET /api/v1/health` - Check the health status of the API

## Usage

### Ingest Book Content
```bash
curl -X POST http://localhost:8000/api/v1/ingest \
  -H "Content-Type: application/json" \
  -d '{
    "book_id": "my-book-1",
    "title": "My Book Title",
    "content": "Full content of my book...",
    "chapters": [
      {
        "id": "ch1",
        "title": "Chapter 1",
        "content": "Content of chapter 1...",
        "page_start": 1,
        "page_end": 25
      }
    ]
  }'
```

### Query the RAG System
```bash
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is the main concept in the book?",
    "context": {
      "current_chapter": "Chapter 1"
    }
  }'
```

### Query in Selected-Text-Only Mode
```bash
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What does this text mean?",
    "selected_text": "The specific text the user selected..."
  }'
```

## Testing

Run the unit tests:
```bash
pytest
```

## Security & Privacy

- All API keys loaded from environment variables (no hard-coded credentials)
- Selected-text-only mode respects strict boundaries, never leaking information outside selected content
- No sensitive data in logs

## Limitations

- Requires Cohere API access
- Relies on Qdrant Cloud for vector storage
- No OpenAI API usage for generation