import os
from dotenv import load_dotenv

load_dotenv()

class Config:
    """Simple configuration for LLM evaluation"""
    
    # API Key
    GROQ_API_KEY = os.getenv('GROQ_API_KEY', 'your_api_key_here')
    
    # All available models from your image
    ALL_MODELS = {
        "allam-2-7b": "allam-2-7b",
        "compound-beta": "compound-beta",
        "compound-beta-mini": "compound-beta-mini",
        "deepseek-r1": "deepseek-r1-distill-llama-70b",
        "gemma2-9b": "gemma2-9b-it",
        "llama3.1-8b": "llama-3.1-8b-instant",
        "llama3.3-70b": "llama-3.3-70b-versatile",
        "llama3-70b": "llama3-70b-8192",
        "llama3-8b": "llama3-8b-8192",
        "llama4-maverick": "meta-llama/llama-4-maverick-17b-128e-instruct",
        "llama4-scout": "meta-llama/llama-4-scout-17b-16e-instruct",
        "llama-guard": "meta-llama/llama-guard-4-12b",
        "llama-prompt-guard-22m": "meta-llama/llama-prompt-guard-2-22m",
        "llama-prompt-guard-86m": "meta-llama/llama-prompt-guard-2-86m",
        "moonshot-kimi": "moonshot/kimi-k2-instruct",
        "gpt-oss-120b": "openai/gpt-oss-120b",
        "gpt-oss-20b": "openai/gpt-oss-20b",
        "qwen3-32b": "qwen/qwen3-32b",
    }
    
    # Default models to test (change this list)
    MODELS_TO_TEST = ["llama3-70b", "llama4-maverick", "compound-beta"] 
    # MODELS_TO_TEST = list(ALL_MODELS.keys())  # Test all models
    
    # Number of scenarios to run (3, 5, or 14)
    SCENARIO_COUNT = 14
    
    # Settings
    MAX_TOKENS = 1024
    TEMPERATURE = 0.3
    RATE_LIMIT_DELAY = 2.0
    RESULTS_DIR = "results"

def get_api_key():
    """Get API key"""
    api_key = Config.GROQ_API_KEY
    if api_key == 'your_api_key_here' or not api_key:
        print("⚠️  Set your API key in .env file!")
        return None
    return api_key