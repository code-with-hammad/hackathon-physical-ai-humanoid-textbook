from pydantic import BaseModel
from typing import List, Optional
from decimal import Decimal


class RetrievedChunk(BaseModel):
    """
    Semantic chunks of book content with metadata (chapter, section, page, source_id)
    """
    id: str
    content: str
    source_id: str  # ID of the original source (book/chapter/section)
    chapter: str    # Chapter name/identifier
    section: str    # Section name/identifier
    page: int       # Page number where chunk originates
    embedding: List[float]  # Vector embedding of the content
    score: float    # Relevance score for the chunk