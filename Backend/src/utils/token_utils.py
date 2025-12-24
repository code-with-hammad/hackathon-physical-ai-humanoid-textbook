import logging
from typing import List, Optional


def count_tokens(text: str) -> int:
    """
    Simple token counting implementation.
    In a real implementation, you might use a tokenizer specific to your LLM.
    """
    # This is a very basic tokenization that splits on whitespace
    # For production use, consider using tiktoken or the specific tokenizer for your LLM
    return len(text.split())


def truncate_text_to_tokens(text: str, max_tokens: int) -> str:
    """
    Truncate text to fit within the max token count.
    """
    words = text.split()
    current_tokens = 0
    result = []
    
    for word in words:
        # Add 1 token for the word itself (simplified)
        if current_tokens + 1 <= max_tokens:
            result.append(word)
            current_tokens += 1
        else:
            break
    
    return " ".join(result)


def calculate_remaining_tokens(prompt_template: str, max_context: int = 4096) -> int:
    """
    Calculate how many tokens remain for dynamic content after accounting for the template.
    """
    template_tokens = count_tokens(prompt_template)
    return max_context - template_tokens