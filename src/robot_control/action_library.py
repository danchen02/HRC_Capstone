#!/usr/bin/env python3
"""
Action Library Module for LLM-Cobot Project
Defines robot actions and integrates with MotionPlanner for execution
"""

import time
import yaml
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import rclpy
from rclpy.node import Node

# Import your motion planner
from motion_planner import MotionPlanner, PlanningResult


class ActionResult(Enum):
    """Enumeration for action execution results"""
    SUCCESS = "success"
    FAILED = "failed"
    INVALID_PARAMETERS = "invalid_parameters"
    OBJECT_NOT_FOUND = "object_not_found"
    PLANNING_FAILED = "planning_failed"
    USER_INPUT_REQUIRED = "user_input_required"


@dataclass
class ActionResponse:
    """Response from action execution"""
    result: ActionResult
    message: str
    data: Optional[Dict[str, Any]] = None


@dataclass
class RobotAction:
    """Definition of a robot action"""
    name: str
    description: str
    parameters: List[str]
    required_params: List[str]
    example: str


class ActionLibrary(Node):
    """
    Action library that manages robot actions and coordinates with MotionPlanner.
    Provides high-level action interface for LLM system.
    """
    
    def __init__(self, config_file: Optional[str] = None):
        super().__init__('action_library')
        
        # Initialize motion planner
        self.motion_planner = MotionPlanner()
        
        # Action definitions
        self.actions = self._define_actions()
        
        # Load configuration if provided
        if config_file:
            self._load_config(config_file)
        
        # Default workspace and object configurations
        self.workspace_bounds = {
            'x_min': -0.8, 'x_max': 0.8,
            'y_min': -0.8, 'y_max': 0.8,
            'z_min': 0.1, 'z_max': 1.0
        }
        
        # Object database (will be updated by perception system)
        self.known_objects = {}
        
        # Scan pattern configuration
        self.scan_pattern = [
            (0.3, 0.3, 0.4),
            (0.3, -0.3, 0.4),
            (-0.3, -0.3, 0.4),
            (-0.3, 0.3, 0.4),
            (0.0, 0.0, 0.5)  # Center overhead view
        ]
        
        self.get_logger().info("Action Library initialized")
    
    def _define_actions(self) -> Dict[str, RobotAction]:
        """Define all available robot actions"""
        return {
            'MOVE': RobotAction(
                name='MOVE',
                description='Move robot end-effector to specified coordinates',
                parameters=['x', 'y', 'z', 'orientation'],
                required_params=['x', 'y', 'z'],
                example='MOVE(0.3, 0.2, 0.15)'
            ),
            'PICK': RobotAction(
                name='PICK',
                description='Pick up a specified object',
                parameters=['object_name', 'approach_height'],
                required_params=['object_name'],
                example='PICK("hammer")'
            ),
            'PLACE': RobotAction(
                name='PLACE',
                description='Place held object at specified location',
                parameters=['object_name', 'x', 'y', 'z'],
                required_params=['x', 'y', 'z'],
                example='PLACE("hammer", 0.2, 0.2, 0.1)'
            ),
            'SCAN': RobotAction(
                name='SCAN',
                description='Scan the workspace to identify objects',
                parameters=['pattern_type', 'height'],
                required_params=[],
                example='SCAN()'
            ),
            'WAIT': RobotAction(
                name='WAIT',
                description='Wait for specified number of seconds',
                parameters=['seconds'],
                required_params=['seconds'],
                example='WAIT(2.0)'
            ),
            'QUERY': RobotAction(
                name='QUERY',
                description='Ask user for clarification or additional information',
                parameters=['question', 'options'],
                required_params=['question'],
                example='QUERY("Which tool do you want me to pick up?")'
            )
        }
    
    def execute_action(self, action_name: str, **kwargs) -> ActionResponse:
        """
        Execute a robot action with given parameters.
        
        Args:
            action_name: Name of action to execute
            **kwargs: Action parameters
            
        Returns:
            ActionResponse with execution result
        """
        action_name = action_name.upper()
        
        if action_name not in self.actions:
            return ActionResponse(
                ActionResult.INVALID_PARAMETERS,
                f"Unknown action: {action_name}. Available: {list(self.actions.keys())}"
            )
        
        self.get_logger().info(f"Executing action: {action_name} with params: {kwargs}")
        
        try:
            # Route to specific action handler
            if action_name == 'MOVE':
                return self._execute_move(**kwargs)
            elif action_name == 'PICK':
                return self._execute_pick(**kwargs)
            elif action_name == 'PLACE':
                return self._execute_place(**kwargs)
            elif action_name == 'SCAN':
                return self._execute_scan(**kwargs)
            elif action_name == 'WAIT':
                return self._execute_wait(**kwargs)
            elif action_name == 'QUERY':
                return self._execute_query(**kwargs)
            else:
                return ActionResponse(
                    ActionResult.FAILED,
                    f"Action handler not implemented: {action_name}"
                )
                
        except Exception as e:
            self.get_logger().error(f"Error executing {action_name}: {str(e)}")
            return ActionResponse(
                ActionResult.FAILED,
                f"Execution error: {str(e)}"
            )
    
    def _execute_move(self, x: float, y: float, z: float, 
                     orientation: Optional[Tuple[float, float, float, float]] = None) -> ActionResponse:
        """Execute MOVE action"""
        # Validate parameters
        if not self._validate_position(x, y, z):
            return ActionResponse(
                ActionResult.INVALID_PARAMETERS,
                f"Position ({x}, {y}, {z}) is outside workspace bounds"
            )
        
        # Execute movement
        result = self.motion_planner.move_to_coordinates(x, y, z, orientation)
        
        if result == PlanningResult.SUCCESS:
            return ActionResponse(
                ActionResult.SUCCESS,
                f"Successfully moved to position ({x:.3f}, {y:.3f}, {z:.3f})"
            )
        else:
            return ActionResponse(
                ActionResult.PLANNING_FAILED,
                f"Movement failed: {result}"
            )
    
    def _execute_pick(self, object_name: str, approach_height: float = 0.1) -> ActionResponse:
        """Execute PICK action"""
        # Check if object is known
        if object_name not in self.known_objects:
            return ActionResponse(
                ActionResult.OBJECT_NOT_FOUND,
                f"Object '{object_name}' not found in workspace. Known objects: {list(self.known_objects.keys())}"
            )
        
        obj_pos = self.known_objects[object_name]
        
        # Generate pick sequence
        pick_waypoints = self.motion_planner.plan_pick_sequence(
            obj_pos['x'], obj_pos['y'], obj_pos['z'], approach_height
        )
        
        # Execute pick sequence
        result = self.motion_planner.execute_waypoint_sequence(pick_waypoints)
        
        if result == PlanningResult.SUCCESS:
            # TODO: Add gripper control here
            self.get_logger().info(f"Would activate gripper to grasp {object_name}")
            return ActionResponse(
                ActionResult.SUCCESS,
                f"Successfully picked up {object_name}",
                data={'object': object_name, 'position': obj_pos}
            )
        else:
            return ActionResponse(
                ActionResult.PLANNING_FAILED,
                f"Pick operation failed: {result}"
            )
    
    def _execute_place(self, x: float, y: float, z: float, 
                      object_name: Optional[str] = None) -> ActionResponse:
        """Execute PLACE action"""
        # Validate position
        if not self._validate_position(x, y, z):
            return ActionResponse(
                ActionResult.INVALID_PARAMETERS,
                f"Place position ({x}, {y}, {z}) is outside workspace bounds"
            )
        
        # Generate place sequence (approach, place, retreat)
        place_waypoints = [
            (x, y, z + 0.1),  # Approach point
            (x, y, z),        # Place point
            (x, y, z + 0.1)   # Retreat point
        ]
        
        # Execute place sequence
        result = self.motion_planner.execute_waypoint_sequence(place_waypoints)
        
        if result == PlanningResult.SUCCESS:
            # TODO: Add gripper release here
            self.get_logger().info(f"Would release gripper to place object")
            return ActionResponse(
                ActionResult.SUCCESS,
                f"Successfully placed object at ({x:.3f}, {y:.3f}, {z:.3f})"
            )
        else:
            return ActionResponse(
                ActionResult.PLANNING_FAILED,
                f"Place operation failed: {result}"
            )
    
    def _execute_scan(self, pattern_type: str = "default", height: float = 0.4) -> ActionResponse:
        """Execute SCAN action"""
        scan_waypoints = []
        
        if pattern_type == "default":
            scan_waypoints = [(x, y, height) for x, y, _ in self.scan_pattern]
        elif pattern_type == "linear":
            # Simple linear scan
            scan_waypoints = [(0.3, y, height) for y in [-0.3, -0.1, 0.1, 0.3]]
        else:
            # Use default if unknown pattern
            scan_waypoints = [(x, y, height) for x, y, _ in self.scan_pattern]
        
        # Execute scan sequence
        result = self.motion_planner.execute_waypoint_sequence(scan_waypoints)
        
        if result == PlanningResult.SUCCESS:
            # TODO: Trigger perception system update here
            self.get_logger().info("Would trigger object detection at each waypoint")
            return ActionResponse(
                ActionResult.SUCCESS,
                f"Workspace scan completed with {len(scan_waypoints)} viewpoints"
            )
        else:
            return ActionResponse(
                ActionResult.PLANNING_FAILED,
                f"Scan operation failed: {result}"
            )
    
    def _execute_wait(self, seconds: float) -> ActionResponse:
        """Execute WAIT action"""
        if seconds < 0 or seconds > 60:  # Safety limit
            return ActionResponse(
                ActionResult.INVALID_PARAMETERS,
                f"Wait time must be between 0 and 60 seconds, got {seconds}"
            )
        
        self.get_logger().info(f"Waiting for {seconds} seconds...")
        time.sleep(seconds)
        
        return ActionResponse(
            ActionResult.SUCCESS,
            f"Waited for {seconds} seconds"
        )
    
    def _execute_query(self, question: str, options: Optional[List[str]] = None) -> ActionResponse:
        """Execute QUERY action"""
        # This action returns a request for user input
        # The LLM system should handle presenting this to the user
        
        query_data = {
            'question': question,
            'options': options,
            'timestamp': time.time()
        }
        
        return ActionResponse(
            ActionResult.USER_INPUT_REQUIRED,
            f"User input required: {question}",
            data=query_data
        )
    
    def _validate_position(self, x: float, y: float, z: float) -> bool:
        """Validate if position is within workspace bounds"""
        return (self.workspace_bounds['x_min'] <= x <= self.workspace_bounds['x_max'] and
                self.workspace_bounds['y_min'] <= y <= self.workspace_bounds['y_max'] and
                self.workspace_bounds['z_min'] <= z <= self.workspace_bounds['z_max'])
    
    def update_object_database(self, objects: Dict[str, Dict[str, float]]):
        """
        Update known objects from perception system.
        
        Args:
            objects: Dict with object_name -> {'x': x, 'y': y, 'z': z}
        """
        self.known_objects = objects
        self.get_logger().info(f"Updated object database with {len(objects)} objects: {list(objects.keys())}")
    
    def get_available_actions(self) -> Dict[str, RobotAction]:
        """Return dictionary of all available actions"""
        return self.actions
    
    def get_action_help(self, action_name: Optional[str] = None) -> str:
        """Get help text for actions"""
        if action_name:
            action_name = action_name.upper()
            if action_name in self.actions:
                action = self.actions[action_name]
                return f"{action.name}: {action.description}\nExample: {action.example}\nParameters: {action.parameters}"
            else:
                return f"Unknown action: {action_name}"
        else:
            help_text = "Available Actions:\n"
            for action in self.actions.values():
                help_text += f"  {action.example} - {action.description}\n"
            return help_text
    
    def _load_config(self, config_file: str):
        """Load configuration from YAML file"""
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                
            if 'workspace_bounds' in config:
                self.workspace_bounds.update(config['workspace_bounds'])
                
            if 'scan_pattern' in config:
                self.scan_pattern = config['scan_pattern']
                
            self.get_logger().info(f"Loaded configuration from {config_file}")
            
        except Exception as e:
            self.get_logger().warning(f"Could not load config file {config_file}: {e}")


# Example usage and testing
def main():
    """Example usage of the ActionLibrary"""
    rclpy.init()
    
    # Create action library
    action_lib = ActionLibrary()
    
    try:
        # Test object database update
        test_objects = {
            'hammer': {'x': 0.3, 'y': 0.2, 'z': 0.1},
            'screwdriver': {'x': 0.2, 'y': 0.3, 'z': 0.1}
        }
        action_lib.update_object_database(test_objects)
        
        # Test various actions
        print("\n=== Testing Action Library ===")
        
        # Test MOVE
        result = action_lib.execute_action('MOVE', x=0.3, y=0.3, z=0.2)
        print(f"MOVE result: {result.result.value} - {result.message}")
        
        # Test SCAN
        result = action_lib.execute_action('SCAN')
        print(f"SCAN result: {result.result.value} - {result.message}")
        
        # Test PICK
        result = action_lib.execute_action('PICK', object_name='hammer')
        print(f"PICK result: {result.result.value} - {result.message}")
        
        # Test PLACE
        result = action_lib.execute_action('PLACE', x=0.2, y=0.2, z=0.1)
        print(f"PLACE result: {result.result.value} - {result.message}")
        
        # Test WAIT
        result = action_lib.execute_action('WAIT', seconds=1.0)
        print(f"WAIT result: {result.result.value} - {result.message}")
        
        # Test QUERY
        result = action_lib.execute_action('QUERY', question="Which tool should I pick up?", 
                                         options=['hammer', 'screwdriver'])
        print(f"QUERY result: {result.result.value} - {result.message}")
        
        # Test invalid action
        result = action_lib.execute_action('INVALID_ACTION')
        print(f"INVALID result: {result.result.value} - {result.message}")
        
        # Show help
        print(f"\n=== Action Help ===\n{action_lib.get_action_help()}")
        
    except KeyboardInterrupt:
        action_lib.get_logger().info("Action library interrupted by user")
    finally:
        action_lib.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()