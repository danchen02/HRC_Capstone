#!/usr/bin/env python3
"""
Barebones Main - User prompt to robot execution
"""
import rclpy

# Simple imports since main.py is now in src/
from robot_control.motion_planner import MotionPlanner, PlanningResult
from llm_framework.llm_manager import LLMManager
from llm_framework.api_bridge import APIBridge
from llm_framework.action_library import ActionLibrary


def main():
    """Simple user input to robot execution"""
    
    print("🤖 Barebones LLM-Robot System")
    print("=" * 40)
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create components
        motion_planner = MotionPlanner()
        action_library = ActionLibrary(motion_planner)
        api_bridge = APIBridge(action_library)
        llm_manager = LLMManager(motion_planner=motion_planner)
        
        print("✅ All systems ready!")
        print("\nType 'quit' to exit")
        
        # Simple loop
        while True:
            # Get user input
            user_input = input("\n💬 Enter command: ").strip()
            
            if user_input.lower() in ['quit', 'exit', 'q']:
                break
            
            if not user_input:
                continue
            
            print(f"\n🗣️  Processing: '{user_input}'")
            
            # Step 1: LLM processes command
            llm_response = llm_manager.process_command(user_input)
            
            if not llm_response.success:
                print(f"❌ LLM Error: {llm_response.feedback}")
                continue
            
            print(f"🧠 LLM Understanding: {llm_response.understanding}")
            print(f"🎯 Actions: {llm_response.actions}")
            
            # Step 2: Execute actions on real robot
            print("🤖 Executing on robot...")
            action_results = api_bridge.execute_action_list(llm_response.actions)
            
            # Step 3: Show results
            for i, result in enumerate(action_results, 1):
                if result["result"].value == "success":
                    print(f"✅ Action {i}: {result['message']}")
                else:
                    print(f"❌ Action {i}: {result['message']}")
                    break  # Stop on first failure
    
    except KeyboardInterrupt:
        print("\n👋 Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        # Cleanup
        motion_planner.destroy_node()
        rclpy.shutdown()
        print("🏁 System shutdown")


if __name__ == "__main__":
    main()