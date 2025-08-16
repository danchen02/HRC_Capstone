#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from shape_msgs.msg import SolidPrimitive

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

def main():
    rclpy.init()
    tester = SimpleTester()
    
    # Move to (0.2, 0.2, 0.3)
    success = tester.move_to_position(0.2, 0.2, 0.3)
    print("Success!" if success else "Failed!")
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()