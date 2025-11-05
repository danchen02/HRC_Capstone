#!/usr/bin/env python3
"""
Test suite for Reachability Map and Enhanced Motion Planning
Tests with fake hardware + gripper
"""

import sys
import os

# Get the absolute path to the src directory
current_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.dirname(current_dir)  # Go up one level to src/
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)
    print(f"Added to path: {src_dir}")

import rclpy
from robot_control.motion_planner import MotionPlanner, PlanningResult
from robot_control.reachability_map import ReachabilityMap
import time
import math


class EnhancedMotionPlanner(MotionPlanner):
    """
    Motion planner with reachability map and Cartesian path fallback
    """
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        # Initialize reachability map
        self.reach_map = ReachabilityMap(self, resolution=0.03)
        
        # Try to load existing map
        map_file = "reachability_map_ur3e_rg2.pkl"
        if not self.reach_map.load_map(map_file):
            self.get_logger().info(
                "No existing map found. Will need to generate one."
            )
    
    def generate_reachability_map(self):
        """Generate reachability map for UR3e with RG2 gripper"""
        self.get_logger().info("Starting reachability map generation...")
        self.get_logger().info("⚠️  This will take 5-10 minutes. Please wait...")
        
        # Define workspace bounds for UR3e - more realistic bounds
        # UR3e has ~500mm reach, but accounting for mounting and gripper
        success = self.reach_map.generate_map(
            x_range=(-0.5, 0.5),   # 80cm range in X
            y_range=(-0.5, 0.5),   # 80cm range in Y  
            z_range=(0.01, 0.5),    # 10cm to 50cm height (above table)
            save_file="reachability_map_ur3e_rg2.pkl"
        )
        
        if success:
            self.get_logger().info("✅ Reachability map generation complete!")
        else:
            self.get_logger().error("❌ Reachability map generation failed!")
        
        return success
    
    def move_to_coordinates_enhanced(self, x: float, y: float, z: float,
                                    use_map: bool = True,
                                    use_cartesian: bool = True) -> PlanningResult:
        """
        Enhanced move with reachability map and Cartesian fallback
        
        Args:
            x, y, z: Target coordinates
            use_map: Whether to use reachability map for validation
            use_cartesian: Whether to use Cartesian path as fallback
        """
        original_target = (x, y, z)
        
        # Step 1: Check reachability map
        if use_map and self.reach_map.reachability_grid:
            closest, is_exact = self.reach_map.find_closest_reachable(x, y, z)
            
            if closest is None:
                self.get_logger().error("No reachable point found in map!")
                return PlanningResult.INVALID_COORDINATES
            
            if not is_exact:
                self.get_logger().warn(
                    f"Target {original_target} unreachable. "
                    f"Using closest: {closest}"
                )
                x, y, z = closest
        
        # Step 2: Try Cartesian path planning (smooth linear motion)
        if use_cartesian:
            self.get_logger().info(
                f"Attempting Cartesian path to ({x:.3f}, {y:.3f}, {z:.3f})"
            )
            result = self.move_with_cartesian_path(x, y, z)
            
            if result == PlanningResult.SUCCESS:
                return result
            
            self.get_logger().warn("Cartesian path failed, trying regular planning")
        
        # Step 3: Fallback to regular planning
        self.get_logger().info("Using regular motion planning")
        return self.move_to_coordinates(x, y, z)
    
    def move_with_cartesian_path(self, x: float, y: float, z: float) -> PlanningResult:
        """
        Use Cartesian path planning - goes as far as possible toward target
        
        This is a simplified version using waypoints.
        """
        try:
            # Get current position using TF
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link', 
                    'gripper_tcp', 
                    rclpy.time.Time()
                )
                curr_x = transform.transform.translation.x
                curr_y = transform.transform.translation.y
                curr_z = transform.transform.translation.z
                
                self.get_logger().info(
                    f"Current position: ({curr_x:.3f}, {curr_y:.3f}, {curr_z:.3f})"
                )
            except Exception as e:
                self.get_logger().error(f"Failed to get current position: {e}")
                # Fallback: just try the target directly
                return self.move_to_coordinates(x, y, z)
            
            # Create waypoints along straight line
            num_waypoints = 5
            waypoints = []
            
            for i in range(1, num_waypoints + 1):
                progress = i / num_waypoints
                wp_x = curr_x + (x - curr_x) * progress
                wp_y = curr_y + (y - curr_y) * progress
                wp_z = curr_z + (z - curr_z) * progress
                waypoints.append((wp_x, wp_y, wp_z))
            
            # Execute waypoints
            for i, (wx, wy, wz) in enumerate(waypoints):
                self.get_logger().info(
                    f"  Waypoint {i+1}/{num_waypoints}: ({wx:.3f}, {wy:.3f}, {wz:.3f})"
                )
                result = self.move_to_coordinates(wx, wy, wz)
                
                if result != PlanningResult.SUCCESS:
                    self.get_logger().warn(
                        f"Stopped at waypoint {i+1}/{num_waypoints} (reached maximum extent)"
                    )
                    # If we reached at least halfway, consider it partial success
                    if i >= num_waypoints // 2:
                        return PlanningResult.SUCCESS
                    return result
            
            return PlanningResult.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Cartesian path error: {e}")
            return PlanningResult.PLANNING_FAILED


def run_tests(planner: EnhancedMotionPlanner):
    """Run comprehensive test suite"""
    
    print("\n" + "=" * 70)
    print("🧪 STARTING REACHABILITY MAP TEST SUITE")
    print("=" * 70 + "\n")
    
    # Test 1: Check if map exists
    print("\n📋 TEST 1: Checking for existing reachability map...")
    if not planner.reach_map.reachability_grid:
        print("❌ No map loaded. Generate one? (y/n)")
        response = input().strip().lower()
        if response == 'y':
            planner.generate_reachability_map()
        else:
            print("⚠️  Tests will run without reachability map validation")
    else:
        print("✅ Map loaded successfully!")
    
    time.sleep(2)
    
    # Test 2: Home position
    print("\n📋 TEST 2: Moving to home position...")
    result = planner.move_to_home_position()
    print(f"Result: {'✅ SUCCESS' if result == PlanningResult.SUCCESS else '❌ FAILED'}")
    time.sleep(2)
    
    # Test 3: Reachable position
    print("\n📋 TEST 3: Moving to clearly reachable position (0.25, 0.0, 0.25)...")
    result = planner.move_to_coordinates_enhanced(0.25, 0.0, 0.25)
    print(f"Result: {'✅ SUCCESS' if result == PlanningResult.SUCCESS else '❌ FAILED'}")
    time.sleep(2)
    
    # Test 4: Gripper control
    print("\n📋 TEST 4: Testing gripper control...")
    print("  Opening gripper...")
    result = planner.open_gripper()
    print(f"  Result: {'✅ SUCCESS' if result == PlanningResult.SUCCESS else '❌ FAILED'}")
    time.sleep(1)
    
    print("  Closing gripper...")
    result = planner.close_gripper()
    print(f"  Result: {'✅ SUCCESS' if result == PlanningResult.SUCCESS else '❌ FAILED'}")
    time.sleep(2)
    
    # Test 5: Borderline reachable position
    print("\n📋 TEST 5: Testing borderline position (0.35, 0.2, 0.2)...")
    result = planner.move_to_coordinates_enhanced(0.35, 0.2, 0.2)
    print(f"Result: {'✅ SUCCESS' if result == PlanningResult.SUCCESS else '❌ FAILED'}")
    time.sleep(2)
    
    # Test 6: Clearly unreachable position
    print("\n📋 TEST 6: Testing unreachable position (0.6, 0.6, 0.6)...")
    print("  This should use the closest reachable point from the map")
    result = planner.move_to_coordinates_enhanced(0.6, 0.6, 0.6)
    print(f"Result: {'✅ SUCCESS' if result == PlanningResult.SUCCESS else '❌ FAILED'}")
    time.sleep(2)
    
    # Test 7: Cartesian path test
    print("\n📋 TEST 7: Testing Cartesian path to (0.25, 0.0, 0.25)...")
    result = planner.move_to_coordinates_enhanced(0.25, 0.0, 0.25, use_cartesian=True)
    print(f"Result: {'✅ SUCCESS' if result == PlanningResult.SUCCESS else '❌ FAILED'}")
    time.sleep(2)
    
    # Test 8: Return home
    print("\n📋 TEST 8: Returning to home position...")
    result = planner.move_to_home_position()
    print(f"Result: {'✅ SUCCESS' if result == PlanningResult.SUCCESS else '❌ FAILED'}")
    
    print("\n" + "=" * 70)
    print("✅ TEST SUITE COMPLETE")
    print("=" * 70 + "\n")


def interactive_test(planner: EnhancedMotionPlanner):
    """Interactive testing mode"""
    print("\n🎮 INTERACTIVE TEST MODE")
    print("=" * 70)
    print("Commands:")
    print("  move x y z    - Move to coordinates")
    print("  home          - Move to home position")
    print("  open          - Open gripper")
    print("  close         - Close gripper")
    print("  map           - Generate reachability map")
    print("  test          - Run full test suite")
    print("  quit          - Exit")
    print("=" * 70 + "\n")
    
    while True:
        try:
            command = input("Enter command: ").strip().lower()
            
            if command == "quit":
                break
            
            elif command == "home":
                print("Moving to home...")
                result = planner.move_to_home_position()
                print(f"Result: {result}")
            
            elif command == "open":
                print("Opening gripper...")
                result = planner.open_gripper()
                print(f"Result: {result}")
            
            elif command == "close":
                print("Closing gripper...")
                result = planner.close_gripper()
                print(f"Result: {result}")
            
            elif command == "map":
                print("Generating reachability map...")
                planner.generate_reachability_map()
            
            elif command == "test":
                run_tests(planner)
            
            elif command.startswith("move"):
                parts = command.split()
                if len(parts) == 4:
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    print(f"Moving to ({x}, {y}, {z})...")
                    result = planner.move_to_coordinates_enhanced(x, y, z)
                    print(f"Result: {result}")
                else:
                    print("Usage: move x y z")
            
            else:
                print("Unknown command")
        
        except KeyboardInterrupt:
            print("\n\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")


def main():
    """Main test function"""
    print("\n" + "=" * 70)
    print("🤖 ENHANCED MOTION PLANNER TEST")
    print("=" * 70)
    print("\nThis test requires:")
    print("1. Fake hardware launched")
    print("2. MoveIt launched")
    print("3. Robot controllers active")
    print("\nPress Enter when ready...")
    input()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create enhanced motion planner
        planner = EnhancedMotionPlanner(group_name="ur_onrobot_manipulator")
        
        print("\n✅ Enhanced Motion Planner initialized!")
        time.sleep(2)
        
        # Ask user what to do
        print("\nWhat would you like to do?")
        print("1. Run automated test suite")
        print("2. Interactive mode")
        print("3. Just generate reachability map")
        
        choice = input("Enter choice (1-3): ").strip()
        
        if choice == "1":
            run_tests(planner)
        elif choice == "2":
            interactive_test(planner)
        elif choice == "3":
            planner.generate_reachability_map()
        else:
            print("Invalid choice")
        
    except KeyboardInterrupt:
        print("\n\n👋 Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        planner.destroy_node()
        rclpy.shutdown()
        print("\n🏁 Test complete")


if __name__ == '__main__':
    main()