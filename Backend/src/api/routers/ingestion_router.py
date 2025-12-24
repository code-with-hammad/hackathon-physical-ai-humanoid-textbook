from fastapi import APIRouter, HTTPException
from src.api.schemas.query_schemas import IngestionRequest
from src.api.schemas.response_schemas import IngestionResponse
from src.services.ingestion_service import IngestionService
from src.utils.logging_utils import get_logger

router = APIRouter()
logger = get_logger(__name__)


@router.post("/ingest", response_model=IngestionResponse)
async def ingest_endpoint(request: IngestionRequest):
    """
    Ingest endpoint to add book content to the system and create vector embeddings.
    """
    try:
        logger.info(f"Received ingestion request for book: {request.title}")
        
        # Create an ingestion service instance
        ingestion_service = IngestionService()
        
        # Process the book content
        result = await ingestion_service.ingest_book(
            book_id=request.book_id,
            title=request.title,
            content=request.content,
            chapters=request.chapters
        )
        
        response = IngestionResponse(
            status="success",
            message="Book content ingested and indexed successfully",
            chunks_processed=result.get('chunks_processed'),
            book_id=request.book_id
        )
        
        logger.info(f"Book {request.book_id} ingested successfully")
        return response
        
    except Exception as e:
        logger.error(f"Error processing ingestion: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))