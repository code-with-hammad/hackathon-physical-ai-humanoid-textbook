import logging
import sys
from typing import Optional
from src.utils.config_loader import settings


def setup_logging() -> None:
    """
    Configure the logging system based on settings.
    """
    log_level = getattr(logging, settings.log_level.upper(), logging.INFO)
    
    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Configure root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(log_level)
    
    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(formatter)
    
    # Clear existing handlers and add our new one
    root_logger.handlers = []
    root_logger.addHandler(console_handler)


def get_logger(name: str) -> logging.Logger:
    """
    Get a configured logger instance.
    """
    logger = logging.getLogger(name)
    if not logger.handlers:
        # Add a null handler to prevent propagation to root logger if needed
        logger.addHandler(logging.NullHandler())
    return logger