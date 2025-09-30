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
        """Execute PICK action with inline waypoint logic"""
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
            
            pos = object_info["position"]
            obj_x, obj_y, obj_z = pos["x"], pos["y"], pos["z"]
            approach_height = 0.05  # 5cm above object
            
            # STEP 1: Open gripper
            print("🤏 Opening gripper...")
            gripper_result = self.motion_planner.control_gripper(self.motion_planner.gripper_open_width)
            if gripper_result != PlanningResult.SUCCESS:
                return {
                    "result": ActionResult.MOTION_FAILED,
                    "message": "Failed to open gripper before pick"
                }
            
            # STEP 2: Move to approach position (above object)
            print(f"🎯 Approaching {object_name}...")
            approach_result = self.motion_planner.move_to_coordinates(
                obj_x, obj_y, obj_z + approach_height
            )
            if approach_result != PlanningResult.SUCCESS:
                return {
                    "result": ActionResult.MOTION_FAILED,
                    "message": "Failed to approach object"
                }
            
            # STEP 3: Move down to grasp position (at object level)
            print(f"⬇️ Moving to grasp {object_name}...")
            grasp_move_result = self.motion_planner.move_to_coordinates(
                obj_x, obj_y, obj_z
            )
            if grasp_move_result != PlanningResult.SUCCESS:
                return {
                    "result": ActionResult.MOTION_FAILED,
                    "message": "Failed to move to grasp position"
                }
            
            # STEP 4: Close gripper to grasp object
            print("🤏 Closing gripper to grasp object...")
            grasp_result = self.motion_planner.control_gripper(self.motion_planner.gripper_closed_width)
            if grasp_result != PlanningResult.SUCCESS:
                return {
                    "result": ActionResult.MOTION_FAILED,
                    "message": "Failed to close gripper on object"
                }
            
            # STEP 5: Lift object back up
            print(f"⬆️ Lifting {object_name}...")
            lift_result = self.motion_planner.move_to_coordinates(
                obj_x, obj_y, obj_z + approach_height
            )
            if lift_result != PlanningResult.SUCCESS:
                return {
                    "result": ActionResult.MOTION_FAILED,
                    "message": "Failed to lift object after grasping"
                }
            
            return {
                "result": ActionResult.SUCCESS,
                "message": f"Successfully picked up {object_name}"
            }
            
        except Exception as e:
            return {
                "result": ActionResult.FAILED,
                "message": f"Pick error: {str(e)}"
            }
    
    def execute_place(self, x: float, y: float, z: float) -> Dict[str, any]:
        """Execute PLACE action with gripper control"""
        try:
            # 1. Move to place location (approach, place, retreat)
            print(f"📦 Moving to place location ({x:.3f}, {y:.3f}, {z:.3f})...")
            place_waypoints = [
                (x, y, z + 0.05),  # Approach point
                (x, y, z),        # Place point  
            ]
            
            result = self.motion_planner.execute_waypoint_sequence(place_waypoints)
            if result != PlanningResult.SUCCESS:
                return {
                    "result": ActionResult.MOTION_FAILED,
                    "message": f"Place motion failed: {result}"
                }
            
            # 2. Open gripper to release object
            print("🤏 Opening gripper to release...")
            release_result = self.motion_planner.open_gripper()
            if release_result != PlanningResult.SUCCESS:
                return {
                    "result": ActionResult.MOTION_FAILED,
                    "message": "Failed to open gripper to release object"
                }
            
            # 3. Retreat from place location
            print("⬆️ Retreating from place location...")
            retreat_result = self.motion_planner.move_to_coordinates(x, y, z + 0.05)
            if retreat_result != PlanningResult.SUCCESS:
                # Not critical if retreat fails
                print("⚠️ Retreat motion had issues, but object was placed")
            
            print(f"✅ Successfully placed object at ({x:.3f}, {y:.3f}, {z:.3f})")
            return {
                "result": ActionResult.SUCCESS,
                "message": f"Placed object at ({x:.3f}, {y:.3f}, {z:.3f})"
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
    
    def _find_object(self, object_identifier: str) -> Optional[Dict]:
        """Find object in database by exact ID or name"""
        objects = self.objects_data.get("objects", {})
        
        # First try exact object ID match (e.g., "hammer_001")
        if object_identifier in objects:
            return objects[object_identifier]
        
        # Fallback to name matching for backward compatibility
        for obj_id, obj_info in objects.items():
            if obj_info.get("name", "").lower() == object_identifier.lower():
                return obj_info
        
        return None
    
    def get_available_objects(self) -> List[str]:
        """Get list of available object names"""
        objects = self.objects_data.get("objects", {})
        return [obj_info.get("name", obj_id) for obj_id, obj_info in objects.items()]
    
    def get_object_info(self, object_name: str) -> Optional[Dict]:
        """Get detailed info about an object"""
        return self._find_object(object_name)
        
    def execute_gripper(self, target_width: float) -> Dict[str, any]:
        """Execute GRIPPER action with specified width"""
        try:
            # Determine action description for logging
            if target_width >= 0.080:
                action_desc = "opened"
            elif target_width <= 0.010:
                action_desc = "closed"
            else:
                action_desc = f"set to {target_width:.3f}m width"
            
            result = self.motion_planner.control_gripper(target_width)
            
            if result == PlanningResult.SUCCESS:
                return {
                    "result": ActionResult.SUCCESS,
                    "message": f"Gripper {action_desc} successfully"
                }
            else:
                return {
                    "result": ActionResult.MOTION_FAILED,
                    "message": f"Failed to control gripper: {result}"
                }
        except Exception as e:
            return {
                "result": ActionResult.FAILED,
                "message": f"Gripper control error: {str(e)}"
            }



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