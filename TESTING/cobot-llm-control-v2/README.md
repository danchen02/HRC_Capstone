# LLM Evaluation for Cobot Control

Simple reference for testing LLMs with robot commands.

## Setup
```bash
# Create environment
python -m venv cobot-env
cobot-env\Scripts\activate  # Windows
pip install -r requirements.txt

# Add API key to .env
GROQ_API_KEY=your_key_here
```

## Quick Commands
```bash
# Check config
python run_evaluation.py --config

# Test current config
python run_evaluation.py

# Test specific models
python run_evaluation.py --models llama3-70b llama4-maverick

# Quick test (3 scenarios)
python run_evaluation.py --scenarios 3

# Full test (14 scenarios)  
python run_evaluation.py --scenarios 14

# List available models
python run_evaluation.py --list
```

## Configuration

Edit `config.py`:

```python
# Change models to test
MODELS_TO_TEST = ["llama3-70b", "llama4-maverick", "llama4-scout"]

# Change scenario count
SCENARIO_COUNT = 5  # 3, 5, or 14
```

## Available Models
All 18 models from Groq including:
- `llama3-70b`, `llama4-maverick`, `llama4-scout`
- `compound-beta`, `deepseek-r1`, `gemma2-9b`
- `moonshot-kimi`, `qwen3-32b`, `gpt-oss-120b`
- And more...

## Results
- CSV: `results/results.csv`
- JSON: `results/detailed_results.json`