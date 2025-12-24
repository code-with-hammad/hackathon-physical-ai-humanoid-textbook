from pydantic import BaseModel, Field
from typing import List, Dict, Optional, Any


class QueryRequest(BaseModel):
    query: str = Field(..., description="The user's question")
    selected_text: Optional[str] = Field(
        None, 
        description="Optional selected text for selected-text-only mode"
    )
    context: Optional[Dict[str, Any]] = Field(
        None, 
        description="Additional context information"
    )


class QueryResponse(BaseModel):
    answer: str
    sources: List[Dict[str, Any]]
    references: List[str]
    confidence: float
    mode: str  # "full-book" or "selected-text-only"


class IngestionRequest(BaseModel):
    book_id: str = Field(..., description="Unique identifier for the book")
    title: str = Field(..., description="Title of the book")
    content: str = Field(..., description="Full text content of the book")
    chapters: Optional[List[Dict[str, Any]]] = Field(
        None,
        description="List of chapters with their content"
    )