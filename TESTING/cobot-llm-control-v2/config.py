import os
from dotenv import load_dotenv

load_dotenv()

class Config:
    """Simple configuration for LLM evaluation"""
    
    # API Key
    GROQ_API_KEY = os.getenv('GROQ_API_KEY', 'your_api_key_here')
    
    # ✅ All available models from your Groq API (as of 15 October 2025)
    ALL_MODELS = {
        "gpt-oss-120b": "openai/gpt-oss-120b",
        "gpt-oss-20b": "openai/gpt-oss-20b",
        "gpt-oss-safeguard-20b": "openai/gpt-oss-safeguard-20b",

        "llama3.3-70b": "llama-3.3-70b-versatile",
        "llama3.1-8b": "llama-3.1-8b-instant",
        "llama4-maverick": "meta-llama/llama-4-maverick-17b-128e-instruct",
        "llama4-scout": "meta-llama/llama-4-scout-17b-16e-instruct",

        "qwen3-32b": "qwen/qwen3-32b",

        "kimi-k2": "moonshotai/kimi-k2-instruct",
        "kimi-k2-0905": "moonshotai/kimi-k2-instruct-0905",

        "llama-guard-4-12b": "meta-llama/llama-guard-4-12b",
        "llama-prompt-guard-2-22m": "meta-llama/llama-prompt-guard-2-22m",
        "llama-prompt-guard-2-86m": "meta-llama/llama-prompt-guard-2-86m",

        "compound": "groq/compound",
        "compound-mini": "groq/compound-mini",

        "allam-2-7b": "allam-2-7b",

        "whisper-large-v3": "whisper-large-v3",
        "whisper-large-v3-turbo": "whisper-large-v3-turbo",

        "playai-tts": "playai-tts",
        "playai-tts-arabic": "playai-tts-arabic",
    }
    
    # ✅ Default models to test
    MODELS_TO_TEST = [
        
        "llama3.3-70b",
        "llama4-maverick",
        "compound",
        "gpt-oss-120b",
        "kimi-k2-0905",
        
    ]
    
    # Number of scenarios to run
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
