from pydantic import BaseModel
from typing import List, Dict, Optional, Any
from src.models.retrieved_chunk import RetrievedChunk


class Response(BaseModel):
    """
    Generated response with source attribution and confidence level
    """
    answer: str
    sources: List[RetrievedChunk]  # List of chunks used to generate the answer
    confidence: float  # Confidence level of the response
    references: List[str]  # Chapter/section references
    mode: str  # The mode used ("full-book" or "selected-text-only")
    metadata: Dict[str, Any] = {}  # Additional response metadata