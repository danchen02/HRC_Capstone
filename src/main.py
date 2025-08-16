#!/usr/bin/env python3
"""
Simple Main - Just send hardcoded coordinates to motion planner
"""

import sys
import os
import rclpy

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from robot_control.motion_planner import MotionPlanner, PlanningResult


def main():
    """Send hardcoded coordinates to motion planner"""
    
    # Initialize ROS2
    rclpy.init()
    
    # Create motion planner
    planner = MotionPlanner()
    
    # Hardcoded coordinates
    x, y, z = 0.3, 0.2, 0.25
    
    print(f"Moving robot to coordinates: ({x}, {y}, {z})")
    
    # Execute movement
    result = planner.move_to_coordinates(x, y, z)
    
    # Print result
    if result == PlanningResult.SUCCESS:
        print("✅ Movement successful!")
    else:
        print(f"❌ Movement failed: {result}")
    
    # Cleanup
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()