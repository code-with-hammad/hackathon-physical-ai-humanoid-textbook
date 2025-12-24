from pydantic import BaseModel
from typing import List, Dict, Optional, Any


class UserQuery(BaseModel):
    """
    Query input from the user with optional selected text context
    """
    query: str
    selected_text: Optional[str] = None  # Optional selected text for selected-text-only mode
    context: Optional[Dict[str, Any]] = None  # Additional context information
    user_id: Optional[str] = None  # Optional user identifier
    metadata: Dict[str, Any] = {}  # Additional query metadata