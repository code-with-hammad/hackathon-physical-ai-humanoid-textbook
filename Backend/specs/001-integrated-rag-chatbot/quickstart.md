# Quickstart Guide: Integrated RAG Chatbot

**Feature**: 001-integrated-rag-chatbot  
**Date**: 2025-01-15

## Prerequisites

- Python 3.11+
- Access to Cohere API (Command-R+ recommended)
- Qdrant Cloud account (Free Tier)
- Neon Serverless PostgreSQL account

## Environment Setup

Create a `.env` file in the project root with the following variables:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DB_URL=your_neon_db_connection_string
DEBUG=false
LOG_LEVEL=info
```

## Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd <repository-name>
```

2. Create a virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

## Running the Application

1. Start the development server:
```bash
uvicorn src.main:app --reload --port 8000
```

2. The API will be available at `http://localhost:8000`

## Basic Usage

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

## Health Check

Verify the service is running:
```bash
curl http://localhost:8000/api/v1/health
```

## Configuration Notes

- The system will automatically detect if selected_text is provided and switch to selected-text-only mode
- All responses are grounded in book content with proper chapter/section references
- If information is not found in the book, the system will respond with "This information is not present in the book"