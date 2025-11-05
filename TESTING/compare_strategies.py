#!/usr/bin/env python3
"""
Strategy Comparison Tool for Out-of-Reach Goals
Test each implementation independently and measure results
"""

import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.dirname(current_dir)
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)

import rclpy
from robot_control.motion_planner import MotionPlanner, PlanningResult
from robot_control.reachability_map import ReachabilityMap
import math
import time


class StrategyTester(MotionPlanner):
    """
    Test different strategies for handling out-of-reach goals
    """
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        # Initialize reachability map (optional, loads if exists)
        self.reach_map = ReachabilityMap(self, resolution=0.05)
        map_file = "reachability_map_ur3e_rg2.pkl"
        self.map_loaded = self.reach_map.load_map(map_file)
        
        if not self.map_loaded:
            self.get_logger().warn(
                "Reachability map not loaded. Strategy 1 will be unavailable."
            )
    
    def get_distance_to_target(self, target_x, target_y, target_z):
        """
        Calculate distance from current position to target
        
        Returns:
            distance in meters, or None if can't get position
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                'gripper_tcp', 
                rclpy.time.Time()
            )
            curr_x = transform.transform.translation.x
            curr_y = transform.transform.translation.y
            curr_z = transform.transform.translation.z
            
            distance = math.sqrt(
                (target_x - curr_x)**2 + 
                (target_y - curr_y)**2 + 
                (target_z - curr_z)**2
            )
            
            return distance, (curr_x, curr_y, curr_z)
        except Exception as e:
            self.get_logger().error(f"Failed to get current position: {e}")
            return None, None
    
    # ========================================================================
    # STRATEGY 1: Reachability Map (Pre-computed closest point)
    # ========================================================================
    def strategy_1_reachability_map(self, x, y, z):
        """
        Strategy 1: Use pre-computed reachability map
        - Finds closest reachable point
        - Plans directly to that point
        """
        if not self.map_loaded:
            self.get_logger().error("Strategy 1 requires reachability map!")
            return PlanningResult.PLANNING_FAILED
        
        # Find closest reachable point
        closest, is_exact = self.reach_map.find_closest_reachable(x, y, z)
        
        if closest is None:
            self.get_logger().error("No reachable point found in map!")
            return PlanningResult.INVALID_COORDINATES
        
        adjusted_x, adjusted_y, adjusted_z = closest
        
        if not is_exact:
            adjustment = math.sqrt(
                (x - adjusted_x)**2 + 
                (y - adjusted_y)**2 + 
                (z - adjusted_z)**2
            )
            self.get_logger().info(
                f"Map adjusted target by {adjustment:.3f}m: "
                f"({x:.2f}, {y:.2f}, {z:.2f}) → "
                f"({adjusted_x:.2f}, {adjusted_y:.2f}, {adjusted_z:.2f})"
            )
        
        # Move to closest reachable point
        return self.move_to_coordinates(adjusted_x, adjusted_y, adjusted_z)
    
    # ========================================================================
    # STRATEGY 2: Cartesian Waypoints
    # ========================================================================
    def strategy_2_cartesian_waypoints(self, x, y, z, num_waypoints=10):
        """
        Strategy 2: Create waypoints along straight line
        - Moves incrementally toward target
        - Stops when planning fails
        """
        try:
            # Get current position
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'gripper_tcp', rclpy.time.Time()
            )
            curr_x = transform.transform.translation.x
            curr_y = transform.transform.translation.y
            curr_z = transform.transform.translation.z
            
            self.get_logger().info(
                f"Starting from: ({curr_x:.3f}, {curr_y:.3f}, {curr_z:.3f})"
            )
            
            # Create waypoints
            for i in range(1, num_waypoints + 1):
                progress = i / num_waypoints
                wp_x = curr_x + (x - curr_x) * progress
                wp_y = curr_y + (y - curr_y) * progress
                wp_z = curr_z + (z - curr_z) * progress
                
                self.get_logger().info(
                    f"Waypoint {i}/{num_waypoints}: "
                    f"({wp_x:.3f}, {wp_y:.3f}, {wp_z:.3f})"
                )
                
                result = self.move_to_coordinates(wp_x, wp_y, wp_z)
                
                if result != PlanningResult.SUCCESS:
                    self.get_logger().warn(
                        f"Stopped at waypoint {i}/{num_waypoints}"
                    )
                    # If we got at least halfway, consider partial success
                    if i >= num_waypoints // 2:
                        return PlanningResult.SUCCESS
                    return result
            
            return PlanningResult.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Waypoint strategy failed: {e}")
            return PlanningResult.PLANNING_FAILED
    
    # ========================================================================
    # STRATEGY 3: Binary Search Along Vector
    # ========================================================================
    def strategy_3_binary_search(self, x, y, z, tolerance=0.02):
        """
        Strategy 3: Binary search for maximum reachable distance
        - Tests points along vector from current to target
        - Finds furthest reachable point
        """
        try:
            # Get current position
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'gripper_tcp', rclpy.time.Time()
            )
            curr_x = transform.transform.translation.x
            curr_y = transform.transform.translation.y
            curr_z = transform.transform.translation.z
            
            # Calculate direction vector
            direction = [
                x - curr_x,
                y - curr_y,
                z - curr_z
            ]
            max_distance = math.sqrt(sum(d**2 for d in direction))
            
            # Normalize direction
            direction = [d / max_distance for d in direction]
            
            self.get_logger().info(
                f"Binary searching from 0.0m to {max_distance:.3f}m"
            )
            
            # Binary search for maximum reachable distance
            min_dist = 0.0
            max_dist = max_distance
            best_distance = 0.0
            
            iteration = 0
            while max_dist - min_dist > tolerance:
                iteration += 1
                test_dist = (min_dist + max_dist) / 2
                
                test_x = curr_x + direction[0] * test_dist
                test_y = curr_y + direction[1] * test_dist
                test_z = curr_z + direction[2] * test_dist
                
                self.get_logger().info(
                    f"Iteration {iteration}: Testing {test_dist:.3f}m "
                    f"at ({test_x:.3f}, {test_y:.3f}, {test_z:.3f})"
                )
                
                # Test if this point is reachable (quick IK check)
                result = self.move_to_coordinates(test_x, test_y, test_z)
                
                if result == PlanningResult.SUCCESS:
                    min_dist = test_dist
                    best_distance = test_dist
                    self.get_logger().info(f"  ✓ Reachable")
                else:
                    max_dist = test_dist
                    self.get_logger().info(f"  ✗ Unreachable")
            
            self.get_logger().info(
                f"Binary search complete. Best distance: {best_distance:.3f}m "
                f"({best_distance/max_distance*100:.1f}% of target)"
            )
            
            return PlanningResult.SUCCESS if best_distance > 0 else PlanningResult.PLANNING_FAILED
            
        except Exception as e:
            self.get_logger().error(f"Binary search failed: {e}")
            return PlanningResult.PLANNING_FAILED
    
    # ========================================================================
    # STRATEGY 4: Progressive Scaling (Simple Fallback)
    # ========================================================================
    def strategy_4_progressive_scaling(self, x, y, z):
        """
        Strategy 4: Try progressively closer points if target fails
        - Tests 100%, 90%, 80%, 70%, 60%, 50% of distance
        - Uses first successful distance
        """
        try:
            # Get current position
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'gripper_tcp', rclpy.time.Time()
            )
            curr_x = transform.transform.translation.x
            curr_y = transform.transform.translation.y
            curr_z = transform.transform.translation.z
            
            scale_factors = [1.0, 0.9, 0.8, 0.7, 0.6, 0.5]
            
            for scale in scale_factors:
                scaled_x = curr_x + (x - curr_x) * scale
                scaled_y = curr_y + (y - curr_y) * scale
                scaled_z = curr_z + (z - curr_z) * scale
                
                self.get_logger().info(
                    f"Trying {scale*100:.0f}% of distance: "
                    f"({scaled_x:.3f}, {scaled_y:.3f}, {scaled_z:.3f})"
                )
                
                result = self.move_to_coordinates(scaled_x, scaled_y, scaled_z)
                
                if result == PlanningResult.SUCCESS:
                    if scale < 1.0:
                        self.get_logger().info(
                            f"Reached {scale*100:.0f}% of target distance"
                        )
                    return result
            
            self.get_logger().error("All scaled attempts failed")
            return PlanningResult.PLANNING_FAILED
            
        except Exception as e:
            self.get_logger().error(f"Scaling strategy failed: {e}")
            return PlanningResult.PLANNING_FAILED
    
    # ========================================================================
    # STRATEGY 5: Direct Planning (Baseline - no handling)
    # ========================================================================
    def strategy_5_direct_planning(self, x, y, z):
        """
        Strategy 5: Direct planning (baseline)
        - Just try to plan directly to target
        - No error handling
        """
        self.get_logger().info("Attempting direct planning (no error handling)")
        return self.move_to_coordinates(x, y, z)
    
    # ========================================================================
    # STRATEGY 6: True Cartesian Path (MoveIt compute_cartesian_path)
    # ========================================================================
    def strategy_6_true_cartesian_path(self, x, y, z):
        """
        Strategy 6: Use MoveIt's compute_cartesian_path service
        - Plans true Cartesian straight-line path
        - Returns fraction of path achieved (0.0 to 1.0)
        - Automatically goes as far as possible
        """
        from moveit_msgs.srv import GetCartesianPath
        from moveit_msgs.msg import RobotState, Constraints
        from geometry_msgs.msg import Pose
        from trajectory_msgs.msg import JointTrajectory
        
        try:
            # Create service client
            cartesian_client = self.create_client(
                GetCartesianPath, 
                '/compute_cartesian_path'
            )
            
            # Wait for service
            self.get_logger().info("Waiting for compute_cartesian_path service...")
            if not cartesian_client.service_is_ready():
                import time
                start_time = time.time()
                while not cartesian_client.service_is_ready() and (time.time() - start_time) < 5.0:
                    time.sleep(0.1)
                
                if not cartesian_client.service_is_ready():
                    self.get_logger().error("compute_cartesian_path service not available!")
                    return PlanningResult.PLANNING_FAILED
            
            self.get_logger().info("Service ready!")
            
            # Build request
            request = GetCartesianPath.Request()
            request.header.frame_id = self.base_frame
            request.group_name = self.group_name
            
            # Get current state
            request.start_state = RobotState()
            # Empty start_state means use current state
            
            # Create target waypoint (just one - the goal)
            target_pose = Pose()
            target_pose.position.x = x
            target_pose.position.y = y
            target_pose.position.z = z
            # Downward facing gripper
            target_pose.orientation.x = 1.0
            target_pose.orientation.y = 0.0
            target_pose.orientation.z = 0.0
            target_pose.orientation.w = 0.0
            
            request.waypoints = [target_pose]
            
            # Cartesian path parameters
            request.max_step = 0.01  # 1cm resolution along path
            request.jump_threshold = 0.0  # Disable jump threshold
            request.avoid_collisions = True
            request.path_constraints = Constraints()  # Empty constraints
            
            # Link to plan for (end effector)
            request.link_name = self.end_effector_link
            
            self.get_logger().info(
                f"Computing Cartesian path to ({x:.3f}, {y:.3f}, {z:.3f})"
            )
            
            # Call service
            future = cartesian_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if not future.done():
                self.get_logger().error("Service call timed out")
                return PlanningResult.PLANNING_FAILED
            
            response = future.result()
            
            # Check result
            if response is None:
                self.get_logger().error("No response from service")
                return PlanningResult.PLANNING_FAILED
            
            fraction = response.fraction
            self.get_logger().info(
                f"✓ Cartesian path computed: {fraction*100:.1f}% of path achievable"
            )
            
            if fraction <= 0.0:
                self.get_logger().error("No part of path is reachable")
                return PlanningResult.PLANNING_FAILED
            
            if fraction < 1.0:
                self.get_logger().warn(
                    f"Only {fraction*100:.1f}% reachable - going as far as possible"
                )
            
            # Execute the computed trajectory
            if len(response.solution.joint_trajectory.points) == 0:
                self.get_logger().error("No trajectory points in solution")
                return PlanningResult.PLANNING_FAILED
            
            self.get_logger().info(
                f"Executing trajectory with {len(response.solution.joint_trajectory.points)} points"
            )
            
            # Execute using the trajectory controller
            result = self._execute_trajectory(response.solution.joint_trajectory)
            
            if result:
                self.get_logger().info(
                    f"Cartesian path executed successfully ({fraction*100:.1f}% achieved)"
                )
                return PlanningResult.SUCCESS
            else:
                self.get_logger().error("Trajectory execution failed")
                return PlanningResult.PLANNING_FAILED
            
        except Exception as e:
            self.get_logger().error(f"Cartesian path strategy failed: {e}")
            import traceback
            traceback.print_exc()
            return PlanningResult.PLANNING_FAILED
    
    def _execute_trajectory(self, trajectory):
        """
        Execute a joint trajectory directly
        
        Args:
            trajectory: JointTrajectory message
            
        Returns:
            bool: True if successful
        """
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        
        try:
            # The action server name should match the MoveIt execution controller
            # Try common controller names
            controller_names = [
                'scaled_joint_trajectory_controller',  # UR driver default
                f'{self.group_name}_controller',
                'joint_trajectory_controller',
            ]
            
            traj_client = None
            for controller_name in controller_names:
                action_name = f'/{controller_name}/follow_joint_trajectory'
                self.get_logger().info(f"Trying action server: {action_name}")
                
                temp_client = ActionClient(
                    self,
                    FollowJointTrajectory,
                    action_name
                )
                
                if temp_client.wait_for_server(timeout_sec=1.0):
                    self.get_logger().info(f"✓ Found action server: {action_name}")
                    traj_client = temp_client
                    break
                else:
                    temp_client.destroy()
            
            if traj_client is None:
                self.get_logger().error(
                    "No trajectory action server found. Tried: " + 
                    ", ".join(controller_names)
                )
                return False
            
            # Create goal
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = trajectory
            
            # Send goal
            self.get_logger().info("Sending trajectory to controller...")
            future = traj_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if not future.done():
                self.get_logger().error("Goal send timed out")
                return False
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Trajectory goal rejected")
                return False
            
            # Wait for result
            self.get_logger().info("Executing trajectory...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
            
            result = result_future.result()
            
            if result.result.error_code == 0:  # SUCCESS
                self.get_logger().info("Trajectory executed successfully!")
                return True
            else:
                self.get_logger().error(f"Trajectory execution error: {result.result.error_code}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Trajectory execution failed: {e}")
            import traceback
            traceback.print_exc()
            return False


def run_comparison_test(tester, target_x, target_y, target_z):
    """
    Run all strategies and compare results
    """
    print("\n" + "="*70)
    print(f"🎯 TARGET: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
    print("="*70)
    
    strategies = [
        (1, "Reachability Map", tester.strategy_1_reachability_map, False),
        (2, "Cartesian Waypoints (Manual)", tester.strategy_2_cartesian_waypoints, True),
        (3, "Binary Search", tester.strategy_3_binary_search, False),
        (4, "Progressive Scaling", tester.strategy_4_progressive_scaling, False),
        (5, "Direct Planning (Baseline)", tester.strategy_5_direct_planning, False),
        (6, "True Cartesian Path (MoveIt)", tester.strategy_6_true_cartesian_path, True),
    ]
    
    results = []
    
    for num, name, strategy_func, needs_intermediate in strategies:
        print(f"\n{'─'*70}")
        print(f"📊 STRATEGY {num}: {name}")
        print(f"{'─'*70}")
        
        # Move to home first
        print("Moving to home position...")
        home_result = tester.move_to_home_position()
        if home_result != PlanningResult.SUCCESS:
            print("⚠️  Home position move failed, starting from current position")
        time.sleep(1)
        
        # For Cartesian strategies, move to intermediate position first
        if needs_intermediate:
            # Use a known good staging position instead of calculating intermediate
            # This is a safe, easy-to-reach position in the middle of the workspace
            staging_x = 0.25
            staging_y = 0.0
            staging_z = 0.25
            
            print(f"📍 Moving to staging position first: "
                  f"({staging_x:.3f}, {staging_y:.3f}, {staging_z:.3f})")
            print(f"   (This gives Cartesian strategies a fair starting point)")
            
            staging_result = tester.move_to_coordinates(staging_x, staging_y, staging_z)
            
            if staging_result != PlanningResult.SUCCESS:
                print("⚠️  Staging position unreachable, starting from current position")
            else:
                print("✓ Staging position reached")
                
                # Check distance from staging to target
                dist_to_target = math.sqrt(
                    (target_x - staging_x)**2 + 
                    (target_y - staging_y)**2 + 
                    (target_z - staging_z)**2
                )
                print(f"   Distance from staging to target: {dist_to_target:.3f}m")
            
            time.sleep(1)
        
        # Try strategy
        start_time = time.time()
        result = strategy_func(target_x, target_y, target_z)
        elapsed_time = time.time() - start_time
        
        # Measure final distance to target
        distance, final_pos = tester.get_distance_to_target(target_x, target_y, target_z)
        
        if distance is not None:
            print(f"\n📏 RESULTS:")
            print(f"   Success: {'✅ YES' if result == PlanningResult.SUCCESS else '❌ NO'}")
            print(f"   Final position: ({final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f})")
            print(f"   Distance from target: {distance:.3f}m")
            print(f"   Time taken: {elapsed_time:.2f}s")
            
            results.append({
                'strategy': name,
                'success': result == PlanningResult.SUCCESS,
                'distance': distance,
                'time': elapsed_time,
                'final_pos': final_pos
            })
        else:
            print(f"\n📏 RESULTS:")
            print(f"   Success: {'✅ YES' if result == PlanningResult.SUCCESS else '❌ NO'}")
            print(f"   Distance: Unable to measure")
            print(f"   Time taken: {elapsed_time:.2f}s")
        
        time.sleep(2)
    
    # Print summary
    print("\n" + "="*70)
    print("📊 SUMMARY COMPARISON")
    print("="*70)
    print(f"{'Strategy':<25} {'Success':<10} {'Distance (m)':<15} {'Time (s)':<10}")
    print("─"*70)
    
    for r in results:
        success_mark = "✅" if r['success'] else "❌"
        print(f"{r['strategy']:<25} {success_mark:<10} {r['distance']:<15.3f} {r['time']:<10.2f}")
    
    # Find best strategy
    successful = [r for r in results if r['success']]
    if successful:
        best = min(successful, key=lambda x: x['distance'])
        print("\n🏆 BEST STRATEGY:")
        print(f"   {best['strategy']} - {best['distance']:.3f}m from target in {best['time']:.2f}s")


def interactive_mode(tester):
    """
    Interactive mode to test individual strategies
    """
    print("\n🎮 INTERACTIVE STRATEGY TESTER")
    print("="*70)
    
    strategies = {
        '1': ("Reachability Map", tester.strategy_1_reachability_map),
        '2': ("Cartesian Waypoints (Manual)", tester.strategy_2_cartesian_waypoints),
        '3': ("Binary Search", tester.strategy_3_binary_search),
        '4': ("Progressive Scaling", tester.strategy_4_progressive_scaling),
        '5': ("Direct Planning (Baseline)", tester.strategy_5_direct_planning),
        '6': ("True Cartesian Path (MoveIt)", tester.strategy_6_true_cartesian_path),
    }
    
    while True:
        print("\n" + "─"*70)
        print("Enter target coordinates (or 'compare' for full comparison):")
        
        try:
            user_input = input("x y z (or 'compare x y z' or 'quit'): ").strip()
            
            if user_input.lower() == 'quit':
                break
            
            parts = user_input.split()
            
            if parts[0].lower() == 'compare':
                if len(parts) == 4:
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    run_comparison_test(tester, x, y, z)
                else:
                    print("Usage: compare x y z")
                continue
            
            if len(parts) != 3:
                print("Usage: x y z  (e.g., 0.5 0.5 0.5)")
                continue
            
            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            
            # Select strategy
            print("\nSelect strategy:")
            for key, (name, _) in strategies.items():
                print(f"  {key}. {name}")
            
            strategy_choice = input("Choice (1-6): ").strip()
            
            if strategy_choice not in strategies:
                print("Invalid strategy")
                continue
            
            strategy_name, strategy_func = strategies[strategy_choice]
            
            print(f"\n🎯 Testing {strategy_name} with target ({x}, {y}, {z})")
            
            # Execute
            start_time = time.time()
            result = strategy_func(x, y, z)
            elapsed_time = time.time() - start_time
            
            # Measure distance
            distance, final_pos = tester.get_distance_to_target(x, y, z)
            
            print(f"\n📏 RESULTS:")
            print(f"   Success: {'✅ YES' if result == PlanningResult.SUCCESS else '❌ NO'}")
            if distance is not None:
                print(f"   Final position: ({final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f})")
                print(f"   Distance from target: {distance:.3f}m")
            print(f"   Time taken: {elapsed_time:.2f}s")
            
        except ValueError:
            print("Invalid input. Use numbers for coordinates.")
        except KeyboardInterrupt:
            print("\n\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")


def main():
    print("\n" + "="*70)
    print("🧪 STRATEGY COMPARISON TOOL")
    print("="*70)
    print("\nThis tool tests different strategies for handling unreachable goals.")
    print("\nPress Enter when robot system is ready...")
    input()
    
    rclpy.init()
    
    try:
        tester = StrategyTester(group_name="ur_onrobot_manipulator")
        
        print("\n✅ Strategy Tester initialized!")
        time.sleep(1)
        
        print("\nMode:")
        print("1. Interactive mode (test individual strategies)")
        print("2. Comparison mode (test all strategies on one target)")
        
        mode = input("\nSelect mode (1-2): ").strip()
        
        if mode == "1":
            interactive_mode(tester)
        elif mode == "2":
            x = float(input("Target X: "))
            y = float(input("Target Y: "))
            z = float(input("Target Z: "))
            run_comparison_test(tester, x, y, z)
        else:
            print("Invalid mode")
        
    except KeyboardInterrupt:
        print("\n\n👋 Test interrupted")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        tester.destroy_node()
        rclpy.shutdown()
        print("\n🏁 Test complete")


if __name__ == '__main__':
    main()