#!/usr/bin/env python3
"""
Motion Planner Module for LLM-Cobot Project
Handles MoveIt planning and execution for robot movements
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    PlanningOptions, 
    Constraints, 
    PositionConstraint, 
    OrientationConstraint, 
    JointConstraint
)
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
import math
from typing import List, Tuple, Optional, Dict
from enum import Enum
import tf2_ros

class PlanningResult(Enum):
    """Enumeration for planning results"""
    SUCCESS = 1
    PLANNING_FAILED = 2
    EXECUTION_FAILED = 3
    GOAL_NOT_ACCEPTED = 4
    INVALID_COORDINATES = 5

class MotionPlanner(Node):
    """
    Motion planner class that interfaces with MoveIt for robot trajectory planning and execution.
    Receives coordinates from action_library and handles all MoveIt communication.
    """
    
    def __init__(self, group_name: str = "ur_manipulator", planner_id: str = "RRTStarkConfigDefault"):
        super().__init__('motion_planner')
        
        # Configuration
        self.group_name = group_name
        self.planner_id = planner_id
        self.end_effector_link = "tool0"
        self.base_frame = "base_link"
        
        # MoveIt action client
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info("Waiting for MoveIt action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Connected to MoveIt action server")
        
        # Default home position (joint angles in radians)
        self.home_joint_angles = [0.0, math.radians(-90), 0.0, math.radians(-90), 0.0, 0.0]
        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        
        # Planning constraints and tolerances
        self.position_tolerance = 0.01  # 1cm tolerance
        self.orientation_tolerance = 0.1  # radians
        self.joint_tolerance = 0.01  # radians

        # TF listener for finding end-effector pos
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
    def get_current_position(self):
        """Get current end-effector position"""
        try:
            transform = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
            pos = transform.transform.translation
            return f"({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})"
        except Exception:
            return "unknown"
    
    def move_to_coordinates(self, x: float, y: float, z: float, 
                          orientation: Optional[Tuple[float, float, float, float]] = None) -> PlanningResult:
        """
        Move robot end-effector to specified coordinates.
        
        Args:
            x, y, z: Target position coordinates in meters
            orientation: Optional quaternion (x, y, z, w). If None, uses downward-facing default
            
        Returns:
            PlanningResult: Success or failure status
        """
        try:
            # Validate coordinates
            if not self._validate_coordinates(x, y, z):
                self.get_logger().error(f"Invalid coordinates: ({x}, {y}, {z})")
                return PlanningResult.INVALID_COORDINATES
            
            # Use default downward-facing orientation if none provided
            if orientation is None:
                orientation = (1.0, 0.0, 0.0, 0.0)  # Pointing downward
            
            self.get_logger().info(f"Planning move to coordinates: ({x:.3f}, {y:.3f}, {z:.3f})")
            
            # Create and execute goal
            goal_msg = self._create_cartesian_goal(x, y, z, orientation)
            return self._execute_goal(goal_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in move_to_coordinates: {str(e)}")
            return PlanningResult.PLANNING_FAILED
    
    def move_to_home_position(self) -> PlanningResult:
        """
        Move robot to predefined home position using joint space planning.
        
        Returns:
            PlanningResult: Success or failure status
        """
        try:
            self.get_logger().info("Planning move to home position")
            goal_msg = self._create_joint_goal(self.home_joint_angles)
            return self._execute_goal(goal_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in move_to_home_position: {str(e)}")
            return PlanningResult.PLANNING_FAILED
    
    def move_to_joint_angles(self, joint_angles: List[float]) -> PlanningResult:
        """
        Move robot to specified joint angles.
        
        Args:
            joint_angles: List of 6 joint angles in radians
            
        Returns:
            PlanningResult: Success or failure status
        """
        try:
            if len(joint_angles) != 6:
                self.get_logger().error(f"Expected 6 joint angles, got {len(joint_angles)}")
                return PlanningResult.INVALID_COORDINATES
                
            self.get_logger().info(f"Planning move to joint angles: {[math.degrees(a) for a in joint_angles]}")
            goal_msg = self._create_joint_goal(joint_angles)
            return self._execute_goal(goal_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in move_to_joint_angles: {str(e)}")
            return PlanningResult.PLANNING_FAILED
    
    def plan_pick_sequence(self, object_x: float, object_y: float, object_z: float, 
                          approach_height: float = 0.1) -> List[Tuple[float, float, float]]:
        """
        Plan a pick sequence: approach -> grasp -> lift.
        
        Args:
            object_x, object_y, object_z: Object position
            approach_height: Height above object for approach (meters)
            
        Returns:
            List of (x, y, z) waypoints for pick sequence
        """
        waypoints = [
            (object_x, object_y, object_z + approach_height),  # Approach point
            (object_x, object_y, object_z),                    # Grasp point
            (object_x, object_y, object_z + approach_height)   # Lift point
        ]
        
        self.get_logger().info(f"Generated pick sequence with {len(waypoints)} waypoints")
        return waypoints
    
    def execute_waypoint_sequence(self, waypoints: List[Tuple[float, float, float]]) -> PlanningResult:
        """
        Execute a sequence of waypoints.
        
        Args:
            waypoints: List of (x, y, z) coordinates
            
        Returns:
            PlanningResult: Overall execution result
        """
        for i, (x, y, z) in enumerate(waypoints):
            self.get_logger().info(f"Executing waypoint {i+1}/{len(waypoints)}: ({x:.3f}, {y:.3f}, {z:.3f})")
            
            result = self.move_to_coordinates(x, y, z)
            if result != PlanningResult.SUCCESS:
                self.get_logger().error(f"Failed at waypoint {i+1}")
                return result
                
        self.get_logger().info("Successfully executed all waypoints")
        return PlanningResult.SUCCESS
    
    def _create_cartesian_goal(self, x: float, y: float, z: float, 
                             orientation: Tuple[float, float, float, float]) -> MoveGroup.Goal:
        """Create MoveIt goal for Cartesian space planning."""
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = self.group_name
        goal_msg.request.planner_id = self.planner_id
        
        # Set target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.base_frame
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position = Point(x=x, y=y, z=z)
        target_pose.pose.orientation = Quaternion(x=orientation[0], y=orientation[1], 
                                                z=orientation[2], w=orientation[3])
        
        # Create constraints
        constraint = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = self.end_effector_link
        
        constraint_region = SolidPrimitive()
        constraint_region.type = SolidPrimitive.SPHERE
        constraint_region.dimensions = [self.position_tolerance]
        
        pos_constraint.constraint_region.primitives = [constraint_region]
        pos_constraint.constraint_region.primitive_poses = [target_pose.pose]
        
        # Orientation constraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header = target_pose.header
        orient_constraint.link_name = self.end_effector_link
        orient_constraint.orientation = target_pose.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = self.orientation_tolerance
        orient_constraint.absolute_y_axis_tolerance = self.orientation_tolerance
        orient_constraint.absolute_z_axis_tolerance = self.orientation_tolerance
        orient_constraint.weight = 1.0
        
        constraint.position_constraints = [pos_constraint]
        constraint.orientation_constraints = [orient_constraint]
        goal_msg.request.goal_constraints = [constraint]
        
        # Planning options
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False  # Plan and execute
        
        return goal_msg
    
    def _create_joint_goal(self, joint_angles: List[float]) -> MoveGroup.Goal:
        """Create MoveIt goal for joint space planning."""
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = self.group_name
        goal_msg.request.planner_id = self.planner_id
        
        # Create joint constraints
        constraint = Constraints()
        
        for joint_name, target_angle in zip(self.joint_names, joint_angles):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = target_angle
            joint_constraint.tolerance_above = self.joint_tolerance
            joint_constraint.tolerance_below = self.joint_tolerance
            joint_constraint.weight = 1.0
            constraint.joint_constraints.append(joint_constraint)
        
        goal_msg.request.goal_constraints = [constraint]
        
        # Planning options
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False  # Plan and execute
        
        return goal_msg
    
    def _execute_goal(self, goal_msg: MoveGroup.Goal) -> PlanningResult:
        """Execute a MoveIt goal and return the result."""
        try:
            # Send goal
            future = self._action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Goal not accepted by MoveIt")
                return PlanningResult.GOAL_NOT_ACCEPTED
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            result = result_future.result()
            
            # Check success (MoveIt error code 1 = SUCCESS)
            if result.result.error_code.val == 1:
                self.get_logger().info("Motion planning and execution successful")
                return PlanningResult.SUCCESS
            else:
                self.get_logger().error(f"Motion planning failed with error code: {result.result.error_code.val}")
                return PlanningResult.EXECUTION_FAILED
                
        except Exception as e:
            self.get_logger().error(f"Error executing goal: {str(e)}")
            return PlanningResult.EXECUTION_FAILED
    
    def _validate_coordinates(self, x: float, y: float, z: float) -> bool:
        """
        Validate if coordinates are within robot workspace.
        Basic validation - you can expand this based on your robot's specs.
        """
        # Basic range checks for UR robot (adjust based on your robot)
        if not (-0.8 <= x <= 0.8):
            return False
        if not (-0.8 <= y <= 0.8):
            return False
        if not (0.0 <= z <= 1.0):  # Prevent going below table or too high
            return False
        return True
    
    def set_planning_parameters(self, position_tolerance: float = None, 
                              orientation_tolerance: float = None,
                              joint_tolerance: float = None):
        """Update planning tolerances."""
        if position_tolerance is not None:
            self.position_tolerance = position_tolerance
        if orientation_tolerance is not None:
            self.orientation_tolerance = orientation_tolerance
        if joint_tolerance is not None:
            self.joint_tolerance = joint_tolerance
            
        self.get_logger().info(f"Updated tolerances - Pos: {self.position_tolerance}, "
                             f"Orient: {self.orientation_tolerance}, Joint: {self.joint_tolerance}")


# Example usage and testing
def main():
    """Example usage of the MotionPlanner class."""
    rclpy.init()
    
    # Create motion planner
    planner = MotionPlanner()
    
    try:
        # Test coordinate movement
        result = planner.move_to_coordinates(0.3, 0.3, 0.2)
        print(f"Move to coordinates result: {result}")
        
        if result == PlanningResult.SUCCESS:
            # Test home position
            home_result = planner.move_to_home_position()
            print(f"Move to home result: {home_result}")
            
            # Test pick sequence
            pick_waypoints = planner.plan_pick_sequence(0.2, 0.2, 0.15)
            sequence_result = planner.execute_waypoint_sequence(pick_waypoints)
            print(f"Pick sequence result: {sequence_result}")
            
    except KeyboardInterrupt:
        planner.get_logger().info("Motion planner interrupted by user")
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()