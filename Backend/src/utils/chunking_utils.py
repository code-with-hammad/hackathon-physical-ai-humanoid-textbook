import logging
from typing import List
from src.models.book_content import BookContent, Chapter, Section
from src.models.retrieved_chunk import RetrievedChunk


def create_chunks(book: BookContent) -> List[RetrievedChunk]:
    """
    Create semantic chunks from book content with metadata.
    """
    logger = logging.getLogger(__name__)
    logger.info(f"Creating chunks for book: {book.title}")
    
    chunks = []
    chunk_id = 1
    
    # Process book content if it exists
    if book.content:
        # Create chunks from the main book content
        book_chunks = _create_chunks_from_text(
            text=book.content,
            source_id=book.id,
            chapter="Full Book Content",
            section="General",
            page=1
        )
        for chunk in book_chunks:
            chunk.id = f"book-{book.id}-chunk-{chunk_id}"
            chunks.append(chunk)
            chunk_id += 1
    
    # Process chapters
    for chapter in book.chapters:
        # Create chunks from chapter content
        chapter_chunks = _create_chunks_from_text(
            text=chapter.content,
            source_id=chapter.id,
            chapter=chapter.title,
            section="General",
            page=chapter.page_start
        )
        for chunk in chapter_chunks:
            chunk.id = f"chap-{chapter.id}-chunk-{chunk_id}"
            chunks.append(chunk)
            chunk_id += 1
        
        # Process sections within the chapter
        for section in chapter.sections:
            section_chunks = _create_chunks_from_text(
                text=section.content,
                source_id=section.id,
                chapter=chapter.title,
                section=section.title,
                page=section.page_number
            )
            for chunk in section_chunks:
                chunk.id = f"sect-{section.id}-chunk-{chunk_id}"
                chunks.append(chunk)
                chunk_id += 1
    
    logger.info(f"Created {len(chunks)} chunks for book: {book.title}")
    return chunks


def _create_chunks_from_text(
    text: str, 
    source_id: str, 
    chapter: str, 
    section: str, 
    page: int,
    max_chunk_size: int = 500,
    overlap: int = 50
) -> List[RetrievedChunk]:
    """
    Helper function to create chunks from a text block with overlap.
    """
    if not text:
        return []
    
    chunks = []
    start_idx = 0
    
    while start_idx < len(text):
        # Determine the end index for this chunk
        end_idx = start_idx + max_chunk_size
        
        # If we're near the end, adjust to include remaining text
        if end_idx > len(text):
            end_idx = len(text)
        
        # Extract the chunk text
        chunk_text = text[start_idx:end_idx]
        
        # Create a RetrievedChunk with metadata
        chunk = RetrievedChunk(
            id="",  # Will be set by caller
            content=chunk_text,
            source_id=source_id,
            chapter=chapter,
            section=section,
            page=page,
            embedding=[],  # Will be populated with actual embeddings in a real implementation
            score=0.0  # Will be set during retrieval in a real implementation
        )
        
        chunks.append(chunk)
        
        # Move start index with overlap
        start_idx = end_idx - overlap if end_idx < len(text) else len(text)
    
    return chunks