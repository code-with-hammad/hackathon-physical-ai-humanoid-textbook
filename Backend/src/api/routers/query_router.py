from fastapi import APIRouter, HTTPException, Depends
from typing import Dict, Any
from src.models.user_query import UserQuery
from src.models.response import Response as ModelResponse
from src.api.schemas.query_schemas import QueryRequest, QueryResponse
from src.services.retrieval_service import RetrievalService
from src.services.generation_service import GenerationService
from src.services.context_filter_service import ContextFilterService
from src.utils.logging_utils import get_logger

router = APIRouter()
logger = get_logger(__name__)


@router.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """
    Query endpoint for the RAG system.
    """
    try:
        logger.info(f"Received query: {request.query}")
        
        # Create a UserQuery model instance from the request
        user_query = UserQuery(
            query=request.query,
            selected_text=request.selected_text,
            context=request.context
        )
        
        # Determine whether to use selected-text-only mode
        if user_query.selected_text is not None and user_query.selected_text.strip() != "":
            # Selected-text-only mode
            logger.info("Using selected-text-only mode")
            
            # Create temporary context from selected text
            context_filter_service = ContextFilterService()
            temporary_context = context_filter_service.create_temporary_context(
                user_query.selected_text
            )
            
            # Use the temporary context for retrieval
            retrieval_service = RetrievalService()
            retrieved_chunks = retrieval_service.retrieve_from_context(
                user_query.query, 
                temporary_context
            )
            
            mode = "selected-text-only"
        else:
            # Full-book question answering mode
            logger.info("Using full-book mode")
            
            retrieval_service = RetrievalService()
            retrieved_chunks = retrieval_service.retrieve(user_query.query)
            
            mode = "full-book"
        
        # Generate response using the retrieved chunks
        generation_service = GenerationService()
        
        if not retrieved_chunks:
            # If no relevant chunks found, indicate the information is not in the book
            response = ModelResponse(
                answer="This information is not present in the book",
                sources=[],
                references=[],
                confidence=0.0,
                mode=mode
            )
        else:
            # Generate an answer based on the retrieved chunks
            generated_answer = generation_service.generate_answer(
                user_query.query,
                retrieved_chunks
            )
            
            # Extract references from the chunks
            references = list(set([f"{chunk.chapter}, {chunk.section}, page {chunk.page}" 
                                 for chunk in retrieved_chunks]))
            
            response = ModelResponse(
                answer=generated_answer,
                sources=retrieved_chunks,
                references=references,
                confidence=0.95,  # This would be calculated in a real implementation
                mode=mode
            )
        
        # Format response for API
        api_response = QueryResponse(
            answer=response.answer,
            sources=[chunk.dict() for chunk in response.sources],
            references=response.references,
            confidence=response.confidence,
            mode=response.mode
        )
        
        logger.info(f"Returning response with mode: {response.mode}")
        return api_response
        
    except Exception as e:
        logger.error(f"Error processing query: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))