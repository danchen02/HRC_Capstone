#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
import math

class SimpleTester(Node):
    def __init__(self):
        super().__init__('simple_tester')
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self._action_client.wait_for_server()
        
    def move_to_position(self, x, y, z):
        # Create goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = "ur_manipulator"

        goal_msg.request.planner_id = "RRTStarkConfigDefault"
        
        # Set target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position = Point(x=x, y=y, z=z)
        target_pose.pose.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        
        # Position constraint
        constraint = Constraints()
        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = "tool0"
        
        constraint_region = SolidPrimitive()
        constraint_region.type = SolidPrimitive.SPHERE
        constraint_region.dimensions = [0.01]
        
        pos_constraint.constraint_region.primitives = [constraint_region]
        pos_constraint.constraint_region.primitive_poses = [target_pose.pose]
        
        # Orientation constraint - keep it facing downward
        orient_constraint = OrientationConstraint()
        orient_constraint.header = target_pose.header
        orient_constraint.link_name = "tool0"
        orient_constraint.orientation = target_pose.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.1
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0
        
        constraint.position_constraints = [pos_constraint]
        constraint.orientation_constraints = [orient_constraint]
        goal_msg.request.goal_constraints = [constraint]
        
        # Planning options
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        
        # Send and wait
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        return result.result.error_code.val == 1

    def move_to_home_position(self):
        # Create goal for joint space planning
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = "ur_manipulator"

        goal_msg.request.planner_id = "RRTStarkConfigDefault"
        
        # Set joint constraints for home position [0, -90, 0, -90, 0, 0] degrees
        home_joint_angles = [0.0, math.radians(-90), 0.0, math.radians(-90), 0.0, 0.0]
        joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        
        constraint = Constraints()
        
        for i, (joint_name, target_angle) in enumerate(zip(joint_names, home_joint_angles)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = target_angle
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraint.joint_constraints.append(joint_constraint)
        
        goal_msg.request.goal_constraints = [constraint]
        
        # Planning options
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        
        # Send and wait
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        return result.result.error_code.val == 1

def main():
    rclpy.init()
    tester = SimpleTester()
    
    # Move to (0.2, 0.2, 0.3)
    success = tester.move_to_position(0.3, 0.3, 0.2)
    print("Move to coordinate position: Success!" if success else "Move to coordinate position: Failed!")
    
    # Move to home position
    if success:
        home_success = tester.move_to_home_position()
        print("Move to home position: Success!" if home_success else "Move to home position: Failed!")
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()