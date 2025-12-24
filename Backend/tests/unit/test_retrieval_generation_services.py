import pytest
from src.services.retrieval_service import RetrievalService
from src.services.generation_service import GenerationService


@pytest.fixture
def retrieval_service():
    return RetrievalService()


@pytest.fixture
def generation_service():
    return GenerationService()


def test_retrieval_service_initialization(retrieval_service):
    """Test that the retrieval service initializes correctly."""
    assert retrieval_service is not None
    assert hasattr(retrieval_service, 'retrieve')


def test_generation_service_initialization(generation_service):
    """Test that the generation service initializes correctly."""
    assert generation_service is not None
    assert hasattr(generation_service, 'generate_answer')


def test_retrieval_service_returns_list(retrieval_service):
    """Test that the retrieval service returns a list of chunks."""
    query = "test query"
    result = retrieval_service.retrieve(query)
    
    assert isinstance(result, list)
    # The mock implementation returns at least one chunk for any query
    # In a real implementation, this might return an empty list if no relevant chunks found


def test_generation_service_handles_empty_chunks(generation_service):
    """Test that the generation service handles empty chunks list."""
    from src.models.retrieved_chunk import RetrievedChunk
    
    query = "test query"
    empty_chunks = []
    
    result = generation_service.generate_answer(query, empty_chunks)
    
    # The service should return a specific message when no chunks are provided
    assert result == "This information is not present in the book"


if __name__ == "__main__":
    pytest.main()