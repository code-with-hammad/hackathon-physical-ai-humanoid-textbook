from pydantic import BaseModel
from typing import List, Dict, Optional, Any
from enum import Enum


class BookContent(BaseModel):
    """
    The complete book content with chapter/section structure
    """
    id: str
    title: str
    content: str
    chapters: List['Chapter']
    metadata: Dict[str, Any] = {}


class Chapter(BaseModel):
    """
    A chapter within the book
    """
    id: str
    title: str
    content: str
    sections: List['Section'] = []
    page_start: int
    page_end: int


class Section(BaseModel):
    """
    A section within a chapter
    """
    id: str
    title: str
    content: str
    page_number: int


# Forward reference setup
Chapter.update_forward_refs()