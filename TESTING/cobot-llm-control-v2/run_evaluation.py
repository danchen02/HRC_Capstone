#!/usr/bin/env python3
"""
Simple LLM evaluation runner
"""
import sys
from config import Config
from llm_evaluator import SimpleLLMEvaluator

def show_available_models():
    """Show all available models"""
    print("\n🤖 AVAILABLE MODELS:")
    print("=" * 50)
    for name, model_id in Config.ALL_MODELS.items():
        print(f"  • {name:<20} → {model_id}")
    print("=" * 50)

def show_config():
    """Show current configuration"""
    print("\n🔧 CURRENT CONFIGURATION")
    print("=" * 50)
    print(f"Models to test: {', '.join(Config.MODELS_TO_TEST)}")
    print(f"Scenarios: {Config.SCENARIO_COUNT if hasattr(Config, 'SCENARIO_COUNT') else 9}")
    print(f"Temperature: {Config.TEMPERATURE}")
    print(f"Max tokens: {Config.MAX_TOKENS}")
    print(f"Rate limit delay: {Config.RATE_LIMIT_DELAY}s")
    
    estimated_time = len(Config.MODELS_TO_TEST) * 9 * (Config.RATE_LIMIT_DELAY + 1)
    print(f"Estimated time: ~{estimated_time//60} minutes")
    print("=" * 50)

def run_evaluation(models=None, debug=False):
    """Run evaluation"""
    try:
        print("\n🚀 Starting evaluation...")
        evaluator = SimpleLLMEvaluator()
        return evaluator.run_evaluation(models_to_test=models, debug=debug)
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return None

def quick_test(model_name=None):
    """Quick test with just 3 scenarios"""
    print("\n⚡ QUICK TEST MODE (3 scenarios)")
    
    if model_name:
        models = [model_name]
    else:
        models = [Config.MODELS_TO_TEST[0]]  # Just first model
    
    print(f"Testing: {models[0]}")
    
    try:
        evaluator = SimpleLLMEvaluator()
        
        # Override scenarios to just 3
        original_scenarios = evaluator.get_test_scenarios
        evaluator.get_test_scenarios = lambda: original_scenarios()[:3]
        
        return evaluator.run_evaluation(models_to_test=models, debug=False)
    except Exception as e:
        print(f"❌ Error: {e}")
        return None

def interactive_mode():
    """Interactive mode for easy testing"""
    print("\n" + "=" * 60)
    print("🔬 LLM EVALUATION FOR COBOT CONTROL")
    print("=" * 60)
    
    show_config()
    
    print("\n📋 OPTIONS:")
    print("  1. Run with current config")
    print("  2. Quick test (3 scenarios, first model)")
    print("  3. Test specific models")
    print("  4. Debug mode (shows full LLM responses)")
    print("  5. Show available models")
    print("  6. Exit")
    
    choice = input("\nEnter choice (1-6): ").strip()
    
    if choice == "1":
        print("\n✅ Running full evaluation...")
        run_evaluation()
        
    elif choice == "2":
        quick_test()
        
    elif choice == "3":
        show_available_models()
        models_input = input("\nEnter model names (space-separated): ").strip()
        if models_input:
            models = models_input.split()
            # Validate models
            invalid = [m for m in models if m not in Config.ALL_MODELS]
            if invalid:
                print(f"⚠️  Invalid models: {invalid}")
                valid_models = [m for m in models if m in Config.ALL_MODELS]
                if valid_models:
                    print(f"✅ Testing valid models: {valid_models}")
                    run_evaluation(models=valid_models)
            else:
                run_evaluation(models=models)
        else:
            print("❌ No models specified")
            
    elif choice == "4":
        print("\n🐛 Debug mode enabled - will show full responses")
        confirm = input("Continue? (y/n): ").strip().lower()
        if confirm == 'y':
            run_evaluation(debug=True)
            
    elif choice == "5":
        show_available_models()
        input("\nPress Enter to continue...")
        interactive_mode()  # Return to menu
        
    elif choice == "6":
        print("\n👋 Goodbye!")
        return
        
    else:
        print("❌ Invalid choice. Running current config...")
        run_evaluation()

def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='LLM Evaluation for Robot Control (Section 5.1)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python run_evaluation.py                    # Interactive mode
  python run_evaluation.py --config           # Show configuration
  python run_evaluation.py --list             # List available models
  python run_evaluation.py --models llama3.3-70b compound
  python run_evaluation.py --quick            # Quick test (3 scenarios)
  python run_evaluation.py --debug            # Debug mode (shows responses)
        """
    )
    
    parser.add_argument('--models', nargs='+', 
                       help='Models to test (e.g., llama3.3-70b compound)')
    parser.add_argument('--config', action='store_true', 
                       help='Show current configuration')
    parser.add_argument('--list', action='store_true', 
                       help='List all available models')
    parser.add_argument('--quick', action='store_true',
                       help='Quick test (3 scenarios only)')
    parser.add_argument('--debug', action='store_true',
                       help='Debug mode (show full LLM responses)')
    
    args = parser.parse_args()
    
    # Handle flags
    if args.config:
        show_config()
        return
        
    if args.list:
        show_available_models()
        return
    
    # Run modes
    if args.quick:
        quick_test()
    elif args.models:
        # Validate models
        invalid = [m for m in args.models if m not in Config.ALL_MODELS]
        if invalid:
            print(f"⚠️  Invalid models: {invalid}")
            show_available_models()
            valid_models = [m for m in args.models if m in Config.ALL_MODELS]
            if not valid_models:
                print("❌ No valid models to test")
                return
            args.models = valid_models
        
        run_evaluation(models=args.models, debug=args.debug)
    elif args.debug:
        run_evaluation(debug=True)
    elif len(sys.argv) == 1:
        # No arguments - interactive mode
        interactive_mode()
    else:
        # Default run
        run_evaluation()

if __name__ == "__main__":
    main()