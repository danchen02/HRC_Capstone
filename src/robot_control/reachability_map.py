#!/usr/bin/env python3
"""
Reachability Map Module for LLM-Cobot Project
Pre-computes and stores reachable workspace for fast lookup
"""

import numpy as np
import pickle
import os
import math
from typing import Tuple, Optional, Dict
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from geometry_msgs.msg import PoseStamped, Quaternion
import rclpy


class ReachabilityMap:
    """
    Pre-computed map of reachable workspace positions.
    Generates a 3D grid and tests IK feasibility for each point.
    """
    
    def __init__(self, motion_planner, resolution=0.02):
        """
        Initialize reachability map
        
        Args:
            motion_planner: MotionPlanner instance for IK testing
            resolution: Grid resolution in meters (default 5cm)
        """
        self.motion_planner = motion_planner
        self.resolution = resolution
        self.reachability_grid = {}
        self.total_points = 0
        self.reachable_points = 0
        
        # IK service client
        self.ik_client = self.motion_planner.create_client(
            GetPositionIK, 
            '/compute_ik'
        )
        
        self.motion_planner.get_logger().info(
            f"ReachabilityMap initialized with {resolution}m resolution"
        )
    
    def generate_map(self, x_range: Tuple[float, float], 
                    y_range: Tuple[float, float], 
                    z_range: Tuple[float, float],
                    save_file: str = "reachability_map.pkl"):
        """
        Generate reachability map by testing IK for grid points
        
        Args:
            x_range: (min, max) in meters
            y_range: (min, max) in meters
            z_range: (min, max) in meters
            save_file: Filename to save the map
        """
        self.motion_planner.get_logger().info(
            "=" * 50 + "\n" +
            "🗺️  GENERATING REACHABILITY MAP\n" +
            f"   Resolution: {self.resolution}m\n" +
            f"   X range: {x_range}\n" +
            f"   Y range: {y_range}\n" +
            f"   Z range: {z_range}\n" +
            "=" * 50
        )
        
        # Wait for IK service
        self.motion_planner.get_logger().info("Waiting for IK service...")
        if not self.ik_client.service_is_ready():
            # Wait up to 10 seconds for service
            import time
            start_time = time.time()
            while not self.ik_client.service_is_ready() and (time.time() - start_time) < 10.0:
                time.sleep(0.1)
            
            if not self.ik_client.service_is_ready():
                self.motion_planner.get_logger().error(
                    "IK service not available! Cannot generate map."
                )
                return False
        
        self.motion_planner.get_logger().info("IK service ready!")
        
        self.total_points = 0
        self.reachable_points = 0
        
        # Create grid points
        x_points = np.arange(x_range[0], x_range[1], self.resolution)
        y_points = np.arange(y_range[0], y_range[1], self.resolution)
        z_points = np.arange(z_range[0], z_range[1], self.resolution)
        
        total_to_test = len(x_points) * len(y_points) * len(z_points)
        tested = 0
        
        self.motion_planner.get_logger().info(
            f"Testing {total_to_test} grid points..."
        )
        
        # Test each grid point
        for x in x_points:
            for y in y_points:
                for z in z_points:
                    self.total_points += 1
                    tested += 1
                    
                    # Show progress every 10%
                    if tested % max(1, total_to_test // 10) == 0:
                        progress = (tested / total_to_test) * 100
                        self.motion_planner.get_logger().info(
                            f"Progress: {progress:.0f}% "
                            f"({self.reachable_points}/{tested} reachable)"
                        )
                    
                    # Test IK feasibility
                    if self._test_ik_solution(x, y, z):
                        # Round to grid resolution for consistent keys
                        grid_key = (
                            round(x / self.resolution) * self.resolution,
                            round(y / self.resolution) * self.resolution,
                            round(z / self.resolution) * self.resolution
                        )
                        self.reachability_grid[grid_key] = True
                        self.reachable_points += 1
        
        coverage = (self.reachable_points / self.total_points * 100) if self.total_points > 0 else 0
        
        self.motion_planner.get_logger().info(
            "=" * 50 + "\n" +
            "✅ REACHABILITY MAP COMPLETE\n" +
            f"   Total points tested: {self.total_points}\n" +
            f"   Reachable points: {self.reachable_points}\n" +
            f"   Coverage: {coverage:.1f}%\n" +
            "=" * 50
        )
        
        # Save to file
        if save_file:
            self._save_map(save_file)
        
        return True
    
    def _test_ik_solution(self, x: float, y: float, z: float) -> bool:
        """
        Test if IK solution exists for given position
        
        Args:
            x, y, z: Target position
            
        Returns:
            True if reachable, False otherwise
        """
        try:
            # Create IK request
            request = GetPositionIK.Request()
            request.ik_request = PositionIKRequest()
            request.ik_request.group_name = self.motion_planner.group_name
            request.ik_request.avoid_collisions = True
            request.ik_request.timeout.sec = 0
            request.ik_request.timeout.nanosec = 100000000  # 100ms timeout
            
            # Set target pose (downward facing gripper)
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.motion_planner.base_frame
            target_pose.pose.position.x = x
            target_pose.pose.position.y = y
            target_pose.pose.position.z = z
            target_pose.pose.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
            
            request.ik_request.pose_stamped = target_pose
            
            # Call IK service
            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(
                self.motion_planner, 
                future, 
                timeout_sec=0.1
            )
            
            if future.result() is not None:
                response = future.result()
                # MoveIt error code 1 = SUCCESS
                return response.error_code.val == 1
            
            return False
            
        except Exception as e:
            # Silently fail for individual points
            return False
    
    def find_closest_reachable(self, target_x: float, target_y: float, 
                              target_z: float) -> Tuple[Optional[Tuple[float, float, float]], bool]:
        """
        Find closest reachable point to target
        
        Args:
            target_x, target_y, target_z: Target position
            
        Returns:
            ((x, y, z), is_exact): Closest point and whether it's exact match
        """
        if not self.reachability_grid:
            self.motion_planner.get_logger().warn(
                "Reachability map not loaded - cannot find closest point"
            )
            return None, False
        
        # Snap target to grid
        grid_x = round(target_x / self.resolution) * self.resolution
        grid_y = round(target_y / self.resolution) * self.resolution
        grid_z = round(target_z / self.resolution) * self.resolution
        
        # Check if target is exactly reachable
        if (grid_x, grid_y, grid_z) in self.reachability_grid:
            return (grid_x, grid_y, grid_z), True
        
        # Find nearest reachable point
        min_distance = float('inf')
        closest_point = None
        
        for (x, y, z) in self.reachability_grid.keys():
            dist = math.sqrt(
                (x - target_x)**2 + 
                (y - target_y)**2 + 
                (z - target_z)**2
            )
            if dist < min_distance:
                min_distance = dist
                closest_point = (x, y, z)
        
        if closest_point:
            self.motion_planner.get_logger().info(
                f"Target ({target_x:.2f}, {target_y:.2f}, {target_z:.2f}) "
                f"unreachable. Closest point: {closest_point} "
                f"(distance: {min_distance:.3f}m)"
            )
        
        return closest_point, False
    
    def is_reachable(self, x: float, y: float, z: float, tolerance: float = 0.03) -> tuple[bool, Optional[tuple[float, float, float]]]:
        """
        Check if a point is reachable within tolerance
        
        Args:
            x, y, z: Target coordinates
            tolerance: Maximum acceptable distance (default 3cm)
            
        Returns:
            (is_reachable, closest_point)
            - is_reachable: True if within tolerance of a reachable point
            - closest_point: Closest reachable point if target unreachable, None if reachable
        """
        if not self.reachability_grid:
            # No map loaded - can't validate
            self.motion_planner.get_logger().warn(
                "Reachability map not loaded - cannot validate coordinates"
            )
            return True, None  # Assume reachable if no map
        
        closest, is_exact = self.find_closest_reachable(x, y, z)
        
        if closest is None:
            # No reachable points found at all
            return False, None
        
        if is_exact:
            # Target is exactly on a reachable grid point
            return True, None
        
        # Calculate distance to closest reachable point
        distance = math.sqrt(
            (x - closest[0])**2 + 
            (y - closest[1])**2 + 
            (z - closest[2])**2
        )
        
        is_reachable = distance <= tolerance
        
        if not is_reachable:
            self.motion_planner.get_logger().info(
                f"Target ({x:.3f}, {y:.3f}, {z:.3f}) out of reach. "
                f"Closest point ({closest[0]:.3f}, {closest[1]:.3f}, {closest[2]:.3f}) "
                f"is {distance:.3f}m away (tolerance: {tolerance:.3f}m)"
            )
        
        return is_reachable, closest if not is_reachable else None    
    
    def _save_map(self, filename: str):
        """Save reachability map to file"""
        try:
            data = {
                'resolution': self.resolution,
                'grid': self.reachability_grid,
                'total_points': self.total_points,
                'reachable_points': self.reachable_points
            }
            
            with open(filename, 'wb') as f:
                pickle.dump(data, f)
            
            self.motion_planner.get_logger().info(
                f"💾 Saved reachability map to {filename}"
            )
            
        except Exception as e:
            self.motion_planner.get_logger().error(
                f"Failed to save map: {e}"
            )
    
    def load_map(self, filename: str) -> bool:
        """Load pre-computed reachability map"""
        try:
            if not os.path.exists(filename):
                self.motion_planner.get_logger().warn(
                    f"Map file {filename} not found"
                )
                return False
            
            with open(filename, 'rb') as f:
                data = pickle.load(f)
            
            self.resolution = data['resolution']
            self.reachability_grid = data['grid']
            self.total_points = data['total_points']
            self.reachable_points = data['reachable_points']
            
            coverage = (self.reachable_points / self.total_points * 100) if self.total_points > 0 else 0
            
            self.motion_planner.get_logger().info(
                f"📂 Loaded reachability map from {filename}\n" +
                f"   Resolution: {self.resolution}m\n" +
                f"   Reachable points: {self.reachable_points}/{self.total_points} ({coverage:.1f}%)"
            )
            
            return True
            
        except Exception as e:
            self.motion_planner.get_logger().error(
                f"Failed to load map: {e}"
            )
            return False
        

def main():
    """Generate reachability map for UR3e with RG2 gripper"""
    import sys
    import os
    
    # Add src directory to path if not already there
    current_dir = os.path.dirname(os.path.abspath(__file__))
    src_dir = os.path.dirname(current_dir)  # Go up one level to src/
    if src_dir not in sys.path:
        sys.path.insert(0, src_dir)
        print(f"Added to path: {src_dir}")
    
    print("\n" + "="*70)
    print("🗺️  REACHABILITY MAP GENERATOR")
    print("="*70)
    print("\nThis will generate a reachability map for the UR3e + RG2 gripper.")
    print("⚠️  This process takes 5-10 minutes depending on resolution.")
    print("\nRequirements:")
    print("  1. Robot system launched (fake or real)")
    print("  2. MoveIt launched")
    print("  3. Controllers active")
    print("\nPress Enter when ready, or Ctrl+C to cancel...")
    
    try:
        input()
    except KeyboardInterrupt:
        print("\n\nCancelled.")
        return
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Import here to avoid circular dependency
        from robot_control.motion_planner import MotionPlanner
        
        # Create motion planner (needed for IK service)
        print("\n📡 Connecting to robot system...")
        planner = MotionPlanner(group_name="ur_onrobot_manipulator")
        
        # Create reachability map
        print("🗺️  Initializing reachability map...")
        reach_map = ReachabilityMap(planner, resolution=0.02)  # 2cm resolution
        
        # Prompt for resolution
        print(f"\nCurrent resolution: {reach_map.resolution}m (2cm)")
        response = input("Change resolution? (y/n): ").strip().lower()
        
        if response == 'y':
            try:
                new_res = float(input("Enter resolution in meters (e.g., 0.03 for 3cm): "))
                reach_map.resolution = new_res
                print(f"✓ Resolution set to {new_res}m")
            except ValueError:
                print("Invalid input, keeping default 0.05m")
        
        # Define workspace bounds for UR3e
        # UR3e has ~500mm reach, accounting for mounting and gripper
        x_range = (-0.5, 0.5)   # 80cm range in X
        y_range = (-0.5, 0.5)   # 80cm range in Y  
        z_range = (0.01, 0.5)    # 10cm to 50cm height (above table)
        
        print(f"\nWorkspace bounds:")
        print(f"  X: {x_range[0]}m to {x_range[1]}m")
        print(f"  Y: {y_range[0]}m to {y_range[1]}m")
        print(f"  Z: {z_range[0]}m to {z_range[1]}m")
        
        response = input("\nProceed with generation? (y/n): ").strip().lower()
        
        if response != 'y':
            print("Cancelled.")
            return
        
        # Generate map
        print("\n🚀 Starting generation...\n")
        success = reach_map.generate_map(
            x_range=x_range,
            y_range=y_range,
            z_range=z_range,
            save_file="reachability_map_ur3e_rg2.pkl"
        )
        
        if success:
            print("\n" + "="*70)
            print("✅ REACHABILITY MAP GENERATED SUCCESSFULLY")
            print("="*70)
            print(f"\nMap saved to: reachability_map_ur3e_rg2.pkl")
            print(f"Resolution: {reach_map.resolution}m")
            print(f"Reachable points: {reach_map.reachable_points}/{reach_map.total_points}")
            coverage = (reach_map.reachable_points / reach_map.total_points * 100) if reach_map.total_points > 0 else 0
            print(f"Coverage: {coverage:.1f}%")
            print("\nYou can now use this map in your main system!")
        else:
            print("\n❌ Map generation failed.")
            sys.exit(1)
        
    except KeyboardInterrupt:
        print("\n\n👋 Generation interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        if 'planner' in locals():
            planner.destroy_node()
        rclpy.shutdown()
        print("\n🏁 Complete")


if __name__ == '__main__':
    main()