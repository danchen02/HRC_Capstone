import time
import json
import pandas as pd
from groq import Groq
from config import Config, get_api_key

class LLMEvaluator:
    """Simple LLM evaluator for cobot control"""
    
    def __init__(self):
        api_key = get_api_key()
        if not api_key:
            raise ValueError("No API key found")
        
        self.client = Groq(api_key=api_key)
        self.results = []
        
        # Robot control prompt
        self.system_prompt = """You are controlling a UR3 robot arm. 

Available actions:
- MOVE(x, y, z): Move to position
- PICK(object): Pick up object  
- PLACE(object, x, y, z): Place object
- SCAN(): Scan workspace
- WAIT(seconds): Wait
- QUERY(question): Ask for clarification

Format:
Understanding: [what you think the user wants]
Actions: [list of actions]
Feedback: [any questions or status]"""

    def get_test_scenarios(self, count=14):
        """Get test scenarios"""
        scenarios = [
            ("clear_001", "Pick up the red block", ["SCAN", "PICK"]),
            ("clear_002", "Move to position x=0.3, y=0.2, z=0.1", ["MOVE"]),
            ("clear_003", "Scan the workspace", ["SCAN"]),
            ("ambiguous_001", "Pick up that thing over there", ["QUERY", "SCAN"]),
            ("ambiguous_002", "Move it to the other side", ["QUERY"]),
            ("multi_001", "Pick up blue cube and place on table", ["SCAN", "PICK", "PLACE"]),
            ("multi_002", "Scan then pick up hammer and move to 0.4, 0.3, 0.2", ["SCAN", "PICK", "PLACE"]),
            ("multi_003", "Move to home, scan, then pick up red objects", ["MOVE", "SCAN", "PICK"]),
            ("error_001", "Pick up the purple elephant", ["SCAN", "QUERY"]),
            ("error_002", "Move to position x=10, y=10, z=10", ["QUERY"]),
            ("error_003", "Make me a sandwich", ["QUERY"]),
            ("contextual_001", "Hand me the wrench", ["SCAN", "PICK", "MOVE"]),
            ("contextual_002", "The blue part is in the way", ["SCAN", "PICK", "PLACE", "QUERY"]),
            ("contextual_003", "Put the object somewhere safe", ["QUERY", "SCAN"]),
        ]
        return scenarios[:count]
    
    def test_model(self, model_name, command, expected_actions):
        """Test single model on single command"""
        try:
            start_time = time.time()
            
            # Get model ID
            model_id = Config.ALL_MODELS.get(model_name)
            if not model_id:
                return {"error": f"Model {model_name} not found"}
            
            # API call
            response = self.client.chat.completions.create(
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": command}
                ],
                model=model_id,
                temperature=Config.TEMPERATURE,
                max_tokens=Config.MAX_TOKENS,
            )
            
            response_text = response.choices[0].message.content
            response_time = time.time() - start_time
            
            # Extract actions
            actions_found = []
            action_keywords = ["MOVE", "PICK", "PLACE", "SCAN", "WAIT", "QUERY"]
            for keyword in action_keywords:
                if keyword in response_text.upper():
                    actions_found.append(keyword)
            
            # Calculate success score
            expected_upper = [a.upper() for a in expected_actions]
            matches = sum(1 for action in expected_upper if action in actions_found)
            success_score = matches / len(expected_actions) if expected_actions else 1.0
            
            # Bonus for good structure
            if "Understanding:" in response_text or "Actions:" in response_text:
                success_score += 0.1
            success_score = min(1.0, success_score)
            
            return {
                "model": model_name,
                "response": response_text,
                "response_time": response_time,
                "actions_found": actions_found,
                "success_score": success_score,
                "error": None
            }
            
        except Exception as e:
            return {
                "model": model_name,
                "response": f"ERROR: {str(e)}",
                "response_time": 999.0,
                "actions_found": [],
                "success_score": 0.0,
                "error": str(e)
            }
    
    def run_evaluation(self, models_to_test=None, scenario_count=None):
        """Run evaluation"""
        if models_to_test is None:
            models_to_test = Config.MODELS_TO_TEST
        
        if scenario_count is None:
            scenario_count = Config.SCENARIO_COUNT
            
        scenarios = self.get_test_scenarios(scenario_count)
        
        print(f"🚀 Testing {len(models_to_test)} models on {len(scenarios)} scenarios")
        print(f"Models: {models_to_test}")
        
        all_results = []
        total_tests = len(models_to_test) * len(scenarios)
        current_test = 0
        
        for model_name in models_to_test:
            if model_name not in Config.ALL_MODELS:
                print(f"❌ Model {model_name} not found")
                continue
                
            print(f"\n🤖 Testing {model_name}...")
            
            for scenario_id, command, expected_actions in scenarios:
                current_test += 1
                progress = (current_test / total_tests) * 100
                print(f"   [{progress:5.1f}%] {scenario_id}", end=" ")
                
                result = self.test_model(model_name, command, expected_actions)
                result["scenario_id"] = scenario_id
                result["command"] = command
                all_results.append(result)
                
                if result["error"]:
                    print("❌ ERROR")
                else:
                    print(f"✅ {result['success_score']:.2f} ({result['response_time']:.2f}s)")
                
                time.sleep(Config.RATE_LIMIT_DELAY)
        
        self.results = all_results
        return self.create_summary()
    
    def create_summary(self):
        """Create summary DataFrame"""
        data = []
        for result in self.results:
            data.append({
                'Model': result['model'],
                'Scenario_ID': result['scenario_id'],
                'Success_Score': result['success_score'],
                'Response_Time': result['response_time'],
                'Actions_Found': len(result['actions_found']),
                'Has_Error': result['error'] is not None
            })
        
        df = pd.DataFrame(data)
        
        # Print summary
        print(f"\n{'='*50}")
        print("📊 EVALUATION SUMMARY")
        print(f"{'='*50}")
        
        for model in df['Model'].unique():
            model_data = df[df['Model'] == model]
            successful_tests = model_data[~model_data['Has_Error']]
            
            if len(successful_tests) > 0:
                avg_time = successful_tests['Response_Time'].mean()
                avg_success = successful_tests['Success_Score'].mean()
                success_rate = len(successful_tests) / len(model_data) * 100
                
                print(f"\n🤖 {model.upper()}:")
                print(f"   Success Rate: {success_rate:5.1f}% ({len(successful_tests)}/{len(model_data)})")
                print(f"   Avg Response Time: {avg_time:5.2f}s")
                print(f"   Avg Success Score: {avg_success:5.2f}")
            else:
                print(f"\n❌ {model.upper()}: All tests failed")
        
        self.save_results(df)
        return df
    
    def save_results(self, df):
        """Save results"""
        import os
        os.makedirs(Config.RESULTS_DIR, exist_ok=True)
        
        # Save CSV
        csv_path = f"{Config.RESULTS_DIR}/results.csv"
        df.to_csv(csv_path, index=False)
        print(f"\n📊 Results saved to: {csv_path}")
        
        # Save detailed JSON
        json_path = f"{Config.RESULTS_DIR}/detailed_results.json"
        with open(json_path, 'w') as f:
            json.dump(self.results, f, indent=2)
        print(f"📝 Details saved to: {json_path}")