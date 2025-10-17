#!/usr/bin/env python3
"""
Main - User prompt to robot execution
"""
import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading

# Simple imports since main.py is now in src/
from robot_control.motion_planner import MotionPlanner, PlanningResult
from llm_framework.llm_manager import LLMManager
from llm_framework.api_bridge import APIBridge
from llm_framework.action_library import ActionLibrary
from user_interface.gui_manager import RobotGUI

def main():
    """Simple user input to robot execution"""
    print("🤖 LLM-Robot System")
    print("=" * 40)
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create components
        motion_planner = MotionPlanner(group_name="ur_onrobot_manipulator")
        action_library = ActionLibrary(motion_planner)
        api_bridge = APIBridge(action_library)
        llm_manager = LLMManager(motion_planner=motion_planner)
        
        # CRITICAL: Spin motion_planner in background thread to receive callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(motion_planner)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        print("✅ All systems ready!")
        print("\nType 'quit' to exit")
        
        gui = RobotGUI(llm_manager, api_bridge)
        gui.run()

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