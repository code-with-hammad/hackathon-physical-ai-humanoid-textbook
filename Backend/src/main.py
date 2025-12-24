from fastapi import FastAPI
from src.api.routers import query_router, ingestion_router
from src.utils.logging_utils import setup_logging
from src.utils.config_loader import settings


# Setup logging based on configuration
setup_logging()

# Create the FastAPI application instance
app = FastAPI(
    title="Integrated RAG Chatbot API",
    description="A production-grade Retrieval-Augmented Generation (RAG) chatbot for an AI-Native Interactive Book",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# Include API routers
app.include_router(query_router.router, prefix="/api/v1", tags=["query"])
app.include_router(ingestion_router.router, prefix="/api/v1", tags=["ingestion"])


@app.get("/")
async def root():
    """
    Root endpoint to verify the server is running.
    """
    return {"message": "Server is running!"}


@app.get("/api/v1/health")
async def health_check():
    """
    Health check endpoint to verify the service is running.
    """
    return {
        "status": "healthy",
        "timestamp": "2025-01-15T10:30:00Z",
        "dependencies": {
            "qdrant": "connected",  # This would be dynamically checked in a real implementation
            "cohere": "connected"   # This would be dynamically checked in a real implementation
        }
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=settings.debug
    )