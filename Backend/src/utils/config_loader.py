from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    cohere_api_key: str
    qdrant_url: str
    qdrant_api_key: str
    neon_db_url: str
    debug: bool = False
    log_level: str = "INFO"

    class Config:
        env_file = ".env"


settings = Settings()