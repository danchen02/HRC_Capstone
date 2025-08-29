#!/usr/bin/env python3
"""
Action Library Module for LLM-Cobot Project
Handles robot action execution and object management
"""

import time
import yaml
import os
from typing import Dict, List, Optional
from enum import Enum

# Import your motion planner
from robot_control.motion_planner import MotionPlanner, PlanningResult


class ActionResult(Enum):
    """Result of action execution"""
    SUCCESS = "success"
    FAILED = "failed"
    OBJECT_NOT_FOUND = "object_not_found"
    MOTION_FAILED = "motion_failed"
    INVALID_PARAMETERS = "invalid_parameters"


class ActionLibrary:
    """
    Action library that handles robot action execution.
    Manages objects and coordinates with MotionPlanner.
    """
    
    def __init__(self, motion_planner: MotionPlanner, objects_file: str = "config/objects.yaml"):
        self.motion_planner = motion_planner
        self.objects_file = objects_file
        self.objects_data = {}
        self.load_objects()
        
        print("📚 Action Library initialized")
    
    def load_objects(self):
        """Load object database from YAML file"""
        try:
            if os.path.exists(self.objects_file):
                with open(self.objects_file, 'r') as f:
                    self.objects_data = yaml.safe_load(f)
                print(f"📂 Loaded {len(self.objects_data.get('objects', {}))} objects")
            else:
                print(f"⚠️  Object database not found: {self.objects_file}")
                self.objects_data = {"objects": {}}
        except Exception as e:
            print(f"❌ Error loading objects: {e}")
            self.objects_data = {"objects": {}}
    
    def execute_move(self, x: float, y: float, z: float) -> Dict[str, any]:
        """
        Execute MOVE action
        
        Args:
            x, y, z: Target coordinates
            
        Returns:
            Dict with result info
        """
        try:
            result = self.motion_planner.move_to_coordinates(x, y, z)
            
            if result == PlanningResult.SUCCESS:
                return {
                    "result": ActionResult.SUCCESS,
                    "message": f"Moved to ({x:.3f}, {y:.3f}, {z:.3f})"
                }
            else:
                return {
                    "result": ActionResult.MOTION_FAILED,
                    "message": f"Move failed: {result}"
                }
                
        except Exception as e:
            return {
                "result": ActionResult.FAILED,
                "message": f"Move error: {str(e)}"
            }
    
    def execute_pick(self, object_name: str) -> Dict[str, any]:
        """
        Execute PICK action
        
        Args:
            object_name: Name of object to pick
            
        Returns:
            Dict with result info
        """
        try:
            # Reload objects to get latest positions  
            self.load_objects()
            
            # Find object in database
            object_info = self._find_object(object_name)
            if not object_info:
                return {
                    "result": ActionResult.OBJECT_NOT_FOUND,
                    "message": f"Object '{object_name}' not found in workspace"
                }
            
            # Execute pick sequence
            pos = object_info["position"]
            pick_waypoints = self.motion_planner.plan_pick_sequence(
                pos["x"], pos["y"], pos["z"]
            )
            
            result = self.motion_planner.execute_waypoint_sequence(pick_waypoints)
            
            if result == PlanningResult.SUCCESS:
                return {
                    "result": ActionResult.SUCCESS,
                    "message": f"Successfully picked up {object_name}"
                }
            else:
                return {
                    "result": ActionResult.MOTION_FAILED,
                    "message": f"Pick failed: {result}"
                }
                
        except Exception as e:
            return {
                "result": ActionResult.FAILED,
                "message": f"Pick error: {str(e)}"
            }
    
    def execute_place(self, x: float, y: float, z: float) -> Dict[str, any]:
        """
        Execute PLACE action
        
        Args:
            x, y, z: Target coordinates
            
        Returns:
            Dict with result info
        """
        try:
            # Create place sequence (approach, place, retreat)
            place_waypoints = [
                (x, y, z + 0.1),  # Approach point
                (x, y, z),        # Place point  
                (x, y, z + 0.1)   # Retreat point
            ]
            
            result = self.motion_planner.execute_waypoint_sequence(place_waypoints)
            
            if result == PlanningResult.SUCCESS:
                return {
                    "result": ActionResult.SUCCESS,
                    "message": f"Placed object at ({x:.3f}, {y:.3f}, {z:.3f})"
                }
            else:
                return {
                    "result": ActionResult.MOTION_FAILED,
                    "message": f"Place failed: {result}"
                }
                
        except Exception as e:
            return {
                "result": ActionResult.FAILED,
                "message": f"Place error: {str(e)}"
            }
    
    def execute_scan(self) -> Dict[str, any]:
        """
        Execute SCAN action - placeholder for Liam's perception system
        
        Returns:
            Dict with result info
        """
        print("🔍 SCAN called - waiting for Liam's perception system integration")
        
        # TODO: Integrate with Liam's object detection system
        # This should trigger camera capture and object detection
        # For now, just reload existing objects
        self.load_objects()
        
        return {
            "result": ActionResult.SUCCESS,
            "message": "Scan placeholder executed - awaiting perception integration"
        }
    
    def execute_wait(self, seconds: float) -> Dict[str, any]:
        """
        Execute WAIT action
        
        Args:
            seconds: Time to wait
            
        Returns:
            Dict with result info
        """
        try:
            # Safety limit
            if seconds > 60:
                seconds = 60
            
            time.sleep(seconds)
            
            return {
                "result": ActionResult.SUCCESS,
                "message": f"Waited for {seconds} seconds"
            }
            
        except Exception as e:
            return {
                "result": ActionResult.FAILED,
                "message": f"Wait error: {str(e)}"
            }
    
    def execute_query(self, question: str) -> Dict[str, any]:
        """
        Execute QUERY action
        
        Args:
            question: Question to ask user
            
        Returns:
            Dict with result info
        """
        return {
            "result": ActionResult.SUCCESS,
            "message": f"User input needed: {question}"
        }
    
    def execute_home(self) -> Dict[str, any]:
        """
        Execute HOME action - move to home position using specific joint angles
        
        Returns:
            Dict with result info
        """
        try:
            import math
            
            # Convert degrees to radians: [0, -90, 0, -90, 0, 0]
            home_joint_angles = [
                math.radians(0),    # shoulder_pan_joint
                math.radians(-90),  # shoulder_lift_joint  
                math.radians(0),    # elbow_joint
                math.radians(-90),  # wrist_1_joint
                math.radians(0),    # wrist_2_joint
                math.radians(0)     # wrist_3_joint
            ]
            
            result = self.motion_planner.move_to_joint_angles(home_joint_angles)
            
            if result == PlanningResult.SUCCESS:
                return {
                    "result": ActionResult.SUCCESS,
                    "message": "Moved to home position (0°, -90°, 0°, -90°, 0°, 0°)"
                }
            else:
                return {
                    "result": ActionResult.MOTION_FAILED,
                    "message": f"Home move failed: {result}"
                }
                
        except Exception as e:
            return {
                "result": ActionResult.FAILED,
                "message": f"Home error: {str(e)}"
            }
    
    def _find_object(self, object_name: str) -> Optional[Dict]:
        """Find object in database by name"""
        objects = self.objects_data.get("objects", {})
        
        for obj_id, obj_info in objects.items():
            if obj_info.get("name", "").lower() == object_name.lower():
                return obj_info
        
        return None
    
    def get_available_objects(self) -> List[str]:
        """Get list of available object names"""
        objects = self.objects_data.get("objects", {})
        return [obj_info.get("name", obj_id) for obj_id, obj_info in objects.items()]
    
    def get_object_info(self, object_name: str) -> Optional[Dict]:
        """Get detailed info about an object"""
        return self._find_object(object_name)


# Test the Action Library
def main():
    """Test the Action Library"""
    import rclpy
    
    # Initialize ROS2 and motion planner
    rclpy.init()
    motion_planner = MotionPlanner()
    
    # Create action library
    action_library = ActionLibrary(motion_planner)
    
    print("📚 Testing Action Library")
    print("=" * 40)
    
    try:
        # Test MOVE
        result = action_library.execute_move(0.3, 0.2, 0.25)
        print(f"MOVE result: {result['result'].value} - {result['message']}")
        
        # Test SCAN
        result = action_library.execute_scan()
        print(f"SCAN result: {result['result'].value} - {result['message']}")
        
        # Test PICK (if hammer exists)
        result = action_library.execute_pick("hammer")
        print(f"PICK result: {result['result'].value} - {result['message']}")
        
        # Test WAIT
        result = action_library.execute_wait(1.0)
        print(f"WAIT result: {result['result'].value} - {result['message']}")
        
    except KeyboardInterrupt:
        print("\n👋 Test interrupted")
    finally:
        motion_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()