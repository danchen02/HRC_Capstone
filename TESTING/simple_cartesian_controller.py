#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

class SimpleCartesianController(Node):
    
    def __init__(self):
        super().__init__('simple_cartesian_controller')
        
        # MoveIt action client
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.move_client.wait_for_server()
        
        self.get_logger().info("Simple Cartesian Controller ready!")
    
    def move_to_position(self, x, y, z):
        """
        Move robot to cartesian position (x, y, z)
        Returns True if successful, False if failed
        """
        
        # Create pose goal
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.header.stamp = self.get_clock().now().to_msg()
        
        pose_goal.pose.position.x = float(x)
        pose_goal.pose.position.y = float(y) 
        pose_goal.pose.position.z = float(z)
        
        # Default orientation (pointing down)
        pose_goal.pose.orientation.x = 0.0
        pose_goal.pose.orientation.y = 1.0
        pose_goal.pose.orientation.z = 0.0
        pose_goal.pose.orientation.w = 0.0
        
        # Create motion plan request
        req = MotionPlanRequest()
        req.group_name = "ur_manipulator"
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1
        
        # Add position constraint
        constraints = Constraints()
        pos_constraint = PositionConstraint()
        pos_constraint.header = pose_goal.header
        pos_constraint.link_name = "tool0"
        
        # Constraint region (small sphere around target)
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [0.01]  # 1cm tolerance
        
        pos_constraint.constraint_region.primitives = [region]
        pos_constraint.constraint_region.primitive_poses = [pose_goal.pose]
        pos_constraint.weight = 1.0
        
        # Add orientation constraint  
        ori_constraint = OrientationConstraint()
        ori_constraint.header = pose_goal.header
        ori_constraint.link_name = "tool0"
        ori_constraint.orientation = pose_goal.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0
        
        constraints.position_constraints = [pos_constraint]
        constraints.orientation_constraints = [ori_constraint]
        req.goal_constraints = [constraints]
        
        # Create goal and send
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False  # Execute the motion
        
        self.get_logger().info(f"Moving to position: [{x:.3f}, {y:.3f}, {z:.3f}]")
        
        # Send goal and wait for result
        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return False
        
        # Wait for execution result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info("Motion successful!")
            return True
        else:
            self.get_logger().error(f"Motion failed with error code: {result.error_code.val}")
            return False

    def interactive_loop(self):
        """
        Interactive loop for manual coordinate input
        """
        print("\n" + "="*60)
        print("UR3 INTERACTIVE CARTESIAN CONTROLLER")
        print("="*60)
        print("Commands:")
        print("  - Enter coordinates as: x y z (in meters)")
        print("  - 'quit' or 'exit' - exit program")
        print("  - Example: 0.3 0.2 0.5")
        print("  - Coordinate system: base_link frame")
        print("  - Orientation: fixed downward-pointing")
        print("="*60)
        
        while rclpy.ok():
            try:
                user_input = input("\nEnter coordinates (x y z): ").strip()
                
                # Check for exit commands
                if user_input.lower() in ['quit', 'exit', 'q']:
                    self.get_logger().info("Exiting...")
                    break
                
                # Parse coordinates
                coords = user_input.split()
                if len(coords) != 3:
                    print("❌ Please provide exactly 3 coordinates (x y z)")
                    print("   Example: 0.3 0.2 0.5")
                    continue
                
                try:
                    x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
                except ValueError:
                    print("❌ Invalid coordinates. Please use numbers only.")
                    print("   Example: 0.3 0.2 0.5")
                    continue
                
                # Validate reasonable coordinate ranges (basic safety check)
                if abs(x) > 1.0 or abs(y) > 1.0 or z < 0.0 or z > 1.5:
                    print(f"⚠️  Warning: Coordinates [{x:.3f}, {y:.3f}, {z:.3f}] seem outside normal UR3 workspace")
                    confirm = input("   Continue anyway? (y/n): ").strip().lower()
                    if confirm not in ['y', 'yes']:
                        print("   Motion cancelled")
                        continue
                
                print(f"🤖 Commanding robot to move to: [{x:.3f}, {y:.3f}, {z:.3f}]")
                
                # Execute the movement
                success = self.move_to_position(x, y, z)
                
                if success:
                    print("✅ Motion completed successfully!")
                else:
                    print("❌ Motion failed! Check robot status and try again.")
                    
            except KeyboardInterrupt:
                self.get_logger().info("\nExiting due to keyboard interrupt...")
                break
            except Exception as e:
                print(f"❌ Error: {str(e)}")
                print("   Please try again with valid coordinates")

def main():
    rclpy.init()
    
    try:
        controller = SimpleCartesianController()
        
        # Give the action client time to connect
        import time
        time.sleep(1)
        
        # Start interactive loop
        controller.interactive_loop()
        
    except Exception as e:
        print(f"Failed to initialize controller: {e}")
        return 1
    finally:
        try:
            controller.destroy_node()
        except:
            pass
        rclpy.shutdown()
    
    return 0

if __name__ == '__main__':
    main()