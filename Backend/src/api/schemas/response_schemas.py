from pydantic import BaseModel
from typing import List, Dict, Any, Optional


class IngestionResponse(BaseModel):
    status: str
    message: str
    chunks_processed: Optional[int] = None
    book_id: str


class HealthResponse(BaseModel):
    status: str
    timestamp: str
    dependencies: Dict[str, str]