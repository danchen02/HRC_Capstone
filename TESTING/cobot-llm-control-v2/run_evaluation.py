#!/usr/bin/env python3
"""
Simple LLM evaluation runner
"""

import sys
from config import Config
from llm_evaluator import LLMEvaluator

def show_available_models():
    """Show all available models"""
    print("🤖 AVAILABLE MODELS:")
    for name in Config.ALL_MODELS.keys():
        print(f"  • {name}")

def show_config():
    """Show current configuration"""
    print("🔧 CURRENT CONFIGURATION")
    print("=" * 40)
    print(f"Models to test: {Config.MODELS_TO_TEST}")
    print(f"Scenario count: {Config.SCENARIO_COUNT}")
    estimated_time = len(Config.MODELS_TO_TEST) * Config.SCENARIO_COUNT * 2
    print(f"Estimated time: ~{estimated_time} seconds")
    print("=" * 40)

def run_evaluation(models=None, scenarios=None):
    """Run evaluation"""
    try:
        evaluator = LLMEvaluator()
        return evaluator.run_evaluation(models, scenarios)
    except Exception as e:
        print(f"❌ Error: {e}")
        return None

def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Simple LLM Evaluation')
    parser.add_argument('--models', nargs='+', help='Models to test (e.g., llama3-70b llama4-maverick)')
    parser.add_argument('--scenarios', type=int, help='Number of scenarios (3, 5, or 14)')
    parser.add_argument('--config', action='store_true', help='Show configuration')
    parser.add_argument('--list', action='store_true', help='List available models')
    
    args = parser.parse_args()
    
    if args.config:
        show_config()
        return
    
    if args.list:
        show_available_models()
        return
    
    # Run evaluation
    if args.models or args.scenarios:
        run_evaluation(args.models, args.scenarios)
    elif len(sys.argv) == 1:
        # Interactive mode
        print("🚀 LLM EVALUATION FOR COBOT CONTROL")
        show_config()
        print("\nOptions:")
        print("1. Run with current config")
        print("2. Test specific models")
        print("3. Quick test (3 scenarios)")
        print("4. Full test (14 scenarios)")
        print("5. Show available models")
        
        choice = input("\nEnter choice (1-5): ").strip()
        
        if choice == "1":
            run_evaluation()
        elif choice == "2":
            show_available_models()
            models_input = input("\nEnter model names (space-separated): ").strip()
            if models_input:
                models = models_input.split()
                run_evaluation(models)
        elif choice == "3":
            run_evaluation(scenarios=3)
        elif choice == "4":
            run_evaluation(scenarios=14)
        elif choice == "5":
            show_available_models()
        else:
            print("Invalid choice. Running current config...")
            run_evaluation()
    else:
        run_evaluation()

if __name__ == "__main__":
    main()