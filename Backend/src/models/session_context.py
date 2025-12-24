from pydantic import BaseModel
from typing import List, Dict, Optional, Any


class SessionContext(BaseModel):
    """
    Current user interaction state (selected-text mode, current chapter, etc.)
    """
    session_id: str
    selected_text_mode: bool = False  # Whether selected-text-only mode is active
    current_context: Optional[str] = None  # Current chapter/section context
    user_preferences: Dict[str, Any] = {}  # User preferences for the session
    history: List[Dict[str, Any]] = []  # Conversation history