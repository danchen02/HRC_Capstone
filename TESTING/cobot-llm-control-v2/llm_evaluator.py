#!/usr/bin/env python3
"""
Simple LLM Evaluator for Section 5.1
Tests models on robot control tasks using actual system structure
"""

import time
import yaml
import json
import pandas as pd
import re
from groq import Groq
from config import Config, get_api_key

class SimpleLLMEvaluator:
    """Simple evaluator matching your actual system"""
    
    def __init__(self, objects_file="config/objects.yaml"):
        api_key = get_api_key()
        if not api_key:
            raise ValueError("No API key found in .env")
        
        self.client = Groq(api_key=api_key)
        self.objects_file = objects_file
        self.objects_data = {}
        self.results = []
        
        # Load objects like the real system does
        self.load_objects()

        # Robot control settings
        self.max_tokens = 512
        self.temperature = 0.3
        
    def load_objects(self):
        """Load objects from YAML like llm_manager.py does"""
        try:
            with open(self.objects_file, 'r') as f:
                self.objects_data = yaml.safe_load(f)
            print(f"✅ Loaded {len(self.objects_data.get('objects', {}))} objects from {self.objects_file}")
        except Exception as e:
            print(f"⚠️  Could not load objects: {e}")
            self.objects_data = {"objects": {}}
    
    
    def get_objects_context(self) -> str:
        """Create context string about available objects with IDs"""
        if not self.objects_data.get('objects'):
            return "No objects currently detected in workspace."
        
        context = "Current objects in workspace:\n"
        for obj_id, obj_info in self.objects_data['objects'].items():
            name = obj_info.get('name', obj_id)
            desc = obj_info.get('description', 'no description')
            pos = obj_info.get('position', {})
            graspable = obj_info.get('properties', {}).get('graspable', True)
            
            # Include the object ID for precise targeting
            context += f"- {obj_id} ({name}): {desc}"
            if pos:
                context += f" at ({pos.get('x', 0):.2f}, {pos.get('y', 0):.2f}, {pos.get('z', 0):.2f})"
            if not graspable:
                context += " (not graspable)"
            context += "\n"
        
        return context
    
    def create_system_prompt(self):
        """Create system prompt matching your actual llm_manager.py"""
        objects_context = self.get_objects_context()
        
        return f"""You are a UR3 robot arm in a collaborative workspace.

AVAILABLE ACTIONS (leave blank if command is ambiguous):
- MOVE(x, y, z): Move end-effector to coordinates
- PICK(object_ID): Pick up specified object, dont move to object. 
- PLACE(x, y, z): Place held object at coordinates, if location not fully specified, place on floor
- HOME(): Move robot to home position (safe starting position)
- SCAN(): Scan workspace to update object detection
- WAIT(seconds): Wait for specified time
- GRIPPER(open/close): Control gripper (or GRIPPER(0.05) for specific width) 

CURRENT WORKSPACE (contains object information):
{objects_context}

RESPONSE FORMAT:
Understanding: [what you think the user wants]
Actions: [specific actions to take, e.g., MOVE(0.3, 0.2, 0.1) OR leave empty if just answering a question]
Feedback: [any questions or status updates for the user OR direct answers to general questions (dont include object IDs in response e.g. hammer_001)]

IMPORTANT:
- For general knowledge questions (non-robot related), provide the answer directly in the Feedback section and leave Actions empty
- If unclear or vague ALWAYS ask for clarification directly in the Feedback section and leave Actions empty
- Use exact object IDs from the workspace list: PICK(hammer_001), PICK(cube_003), PICK(cube_006)
- Object IDs ensure you pick the correct item when multiple similar objects exist
- Use exact object names from the workspace list
- Coordinates should be within robot reach (-0.5 to 0.5 for x,y, 0.01 to 1.0 for z)
- if asked to PICK() or PLACE() object, don't MOVE() to location"""
    
    def get_test_scenarios(self):
        """Simple test scenarios for Section 5.1"""
        return [
            # Clear commands
            ("clear", "Pick up the red cube", ["PICK"], False),
            ("clear", "Go to home position", ["HOME"], False),
            ("clear", "Move to position 1.2, 0.6, 0.1", [], False),
            
            # Ambiguous - should ask for clarification
            ("ambiguous", "Pick up that thing over there", [], True),
            ("ambiguous", "Move it to the other side", [], True),
            ("ambiguous", "Pick up the cube", [], True),
            
            # Multi-step
            ("multi-step", "Pick up the blue cube and place it at 0.3, 0.4, 0.1", ["PICK", "PLACE"], False),
            ("multi-step", "Scan the workspace then pick up the hammer", ["SCAN", "PICK"], False),
            
            # Contextual
            ("contextual", "What tools are in the workspace?", [], False),
            ("contextual", "Open the gripper", ["GRIPPER"], False),
        ]
    
    def extract_actions(self, response_text):
        """Extract actions ONLY from Actions section (not Understanding/Feedback)"""
        actions = []
        keywords = ["MOVE", "PICK", "PLACE", "HOME", "SCAN", "WAIT", "GRIPPER"]
        
        # Extract only the Actions section
        actions_section = ""
        lines = response_text.split('\n')
        in_actions = False
        
        for line in lines:
            if line.strip().startswith('Actions:'):
                in_actions = True
                # Get content after "Actions:" on same line
                actions_section = line.split('Actions:', 1)[1] if ':' in line else ""
                continue
            elif in_actions:
                # Stop when we hit next section
                if line.strip().startswith('Understanding:') or line.strip().startswith('Feedback:'):
                    break
                actions_section += " " + line
        
        # Now search only in actions section
        for keyword in keywords:
            if re.search(rf'{keyword}\s*(?:\([^)]*\))?', actions_section.upper()):
                actions.append(keyword)
        
        return list(set(actions))
    
    def check_format(self, response_text):
        """Check if response has required format"""
        return all(section in response_text for section in 
                  ["Understanding:", "Actions:", "Feedback:"])
    
    def test_model(self, model_name, scenario_type, command, expected_actions, should_clarify):
        """Test one scenario"""
        try:
            model_id = Config.ALL_MODELS.get(model_name)
            if not model_id:
                return None
            
            # Time the response
            start = time.time()
            
            response = self.client.chat.completions.create(
                messages=[
                    {"role": "system", "content": self.create_system_prompt()},
                    {"role": "user", "content": command}
                ],
                model=model_id,
                temperature=Config.TEMPERATURE,
                max_tokens=Config.MAX_TOKENS,
            )
            
            response_time = time.time() - start
            response_text = response.choices[0].message.content
            
            # Evaluate metrics
            actions_found = self.extract_actions(response_text)
            has_format = self.check_format(response_text)
            
            # Check accuracy
            if should_clarify:
                # Ambiguous command → should NOT output actions
                accurate = len(actions_found) == 0
            else:
                # Clear command → must match expected actions EXACTLY
                accurate = set(actions_found) == set(expected_actions)
            
            return {
                "model": model_name,
                "type": scenario_type,
                "command": command,
                "response_time": response_time,
                "accurate": accurate,
                "has_format": has_format,
                "actions": actions_found,
                "response": response_text,
                "error": None
            }
            
        except Exception as e:
            return {
                "model": model_name,
                "type": scenario_type,
                "command": command,
                "response_time": 999,
                "accurate": False,
                "has_format": False,
                "actions": [],
                "response": "",
                "error": str(e)
            }
    
    def run_evaluation(self, models_to_test=None, debug=False):
        """Run evaluation on all models"""
        if models_to_test is None:
            models_to_test = Config.MODELS_TO_TEST
        
        scenarios = self.get_test_scenarios()
        
        print("=" * 70)
        print("🔬 LLM EVALUATION FOR SECTION 5.1")
        print("=" * 70)
        print(f"Testing {len(models_to_test)} models on {len(scenarios)} scenarios")
        print(f"Using objects from: {self.objects_file}")
        if debug:
            print("🐛 DEBUG MODE: Will show full responses")
        print()
        
        for model in models_to_test:
            print(f"\n{'='*70}")
            print(f"Testing: {model}")
            print('='*70)
            
            for scenario_type, command, expected_actions, should_clarify in scenarios:
                print(f"\n  [{scenario_type}] \"{command[:50]}...\"" if len(command) > 50 else f"\n  [{scenario_type}] \"{command}\"")
                
                result = self.test_model(model, scenario_type, command, expected_actions, should_clarify)
                
                if result:
                    self.results.append(result)
                    
                    if result['error']:
                        print(f"     ❌ ERROR: {result['error']}")
                    else:
                        acc_icon = "✅" if result['accurate'] else "❌"
                        fmt_icon = "✅" if result['has_format'] else "❌"
                        print(f"     {acc_icon} Accurate | {fmt_icon} Format | ⏱️  {result['response_time']:.2f}s")
                        print(f"     Actions found: {result['actions']}")
                        
                        # Debug mode shows full response
                        if debug:
                            print(f"\n     --- Full Response ---")
                            for line in result['response'].split('\n'):
                                print(f"     {line}")
                            print(f"     --------------------\n")
                
                time.sleep(Config.RATE_LIMIT_DELAY)
        
        print("\n" + "=" * 70)
        print("✅ EVALUATION COMPLETE")
        print("=" * 70 + "\n")
        
        return self.create_summary()
    
    def create_summary(self):
        """Create summary for report"""
        if not self.results:
            return None
        
        df = pd.DataFrame(self.results)
        
        print("📊 SUMMARY - Section 5.1 Metrics")
        print("-" * 70)
        print(f"{'Model':<25} {'Avg Time':<12} {'Accuracy':<12} {'Format':<12}")
        print("-" * 70)
        
        stats = {}
        for model in df['model'].unique():
            model_df = df[df['model'] == model]
            success_df = model_df[model_df['error'].isna()]
            
            if len(success_df) > 0:
                avg_time = success_df['response_time'].mean()
                accuracy = (success_df['accurate'].sum() / len(success_df)) * 100
                format_pct = (success_df['has_format'].sum() / len(success_df)) * 100
                
                stats[model] = {
                    'avg_time': avg_time,
                    'accuracy': accuracy,
                    'format': format_pct
                }
                
                print(f"{model:<25} {avg_time:>5.2f}s      {accuracy:>5.1f}%       {format_pct:>5.1f}%")
        
        print("-" * 70)
        
        # Best model
        if stats:
            best = max(stats.items(), key=lambda x: (x[1]['accuracy'], -x[1]['avg_time']))
            print(f"\n🏆 Best: {best[0]}")
            print(f"   Accuracy: {best[1]['accuracy']:.1f}%")
            print(f"   Speed: {best[1]['avg_time']:.2f}s")
            print(f"   Format: {best[1]['format']:.1f}%")
        
        # Save results
        self.save_results(df, stats)
        
        return df, stats
    
    def save_results(self, df, stats):
        """Save results"""
        import os
        os.makedirs(Config.RESULTS_DIR, exist_ok=True)
        
        # CSV
        csv_path = f"{Config.RESULTS_DIR}/evaluation_results.csv"
        df.to_csv(csv_path, index=False)
        
        # Stats JSON
        stats_path = f"{Config.RESULTS_DIR}/model_stats.json"
        with open(stats_path, 'w') as f:
            json.dump(stats, f, indent=2)
        
        # Detailed JSON
        json_path = f"{Config.RESULTS_DIR}/detailed_results.json"
        with open(json_path, 'w') as f:
            json.dump(self.results, f, indent=2)
        
        print(f"\n💾 Saved to {Config.RESULTS_DIR}/")
        print(f"   - evaluation_results.csv")
        print(f"   - model_stats.json")
        print(f"   - detailed_results.json")


def main():
    """Run evaluation"""
    import sys
    
    # Check for debug flag
    debug = "--debug" in sys.argv or "-d" in sys.argv
    
    evaluator = SimpleLLMEvaluator()
    df, stats = evaluator.run_evaluation(debug=debug)


if __name__ == "__main__":
    main()