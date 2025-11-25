#!/usr/bin/env python3
"""
Reachability Map Visualization in RViz
Publishes markers showing reachable vs unreachable points
"""

import sys
import os
# Add project src directory to path
current_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.dirname(current_dir)
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)
    print(f"Added to path: {src_dir}")

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from robot_control.reachability_map import ReachabilityMap
from robot_control.motion_planner import MotionPlanner
import time


class ReachabilityVisualizer(Node):
    """
    Visualizes reachability map in RViz using markers
    """
    
    def __init__(self):
        super().__init__('reachability_visualizer')
        
        # Publisher for markers
        self.marker_pub = self.create_publisher(
            MarkerArray, 
            '/reachability_map_markers', 
            10
        )
        
        # Create motion planner (needed for reachability map)
        self.motion_planner = MotionPlanner(group_name="ur_onrobot_manipulator")
        
        # Initialize reachability map
        self.reach_map = ReachabilityMap(self.motion_planner, resolution=0.05)
        
        # Try to load existing map
        map_file = "reachability_map_ur3e_rg2.pkl"
        if not self.reach_map.load_map(map_file):
            self.get_logger().error(
                f"Failed to load map from {map_file}. "
                "Please generate a map first using test_reachability.py"
            )
            return
        
        self.get_logger().info("Reachability map loaded successfully!")
        self.get_logger().info(
            f"Total points: {self.reach_map.total_points}, "
            f"Reachable: {self.reach_map.reachable_points}"
        )
        
        # Timer to publish markers periodically
        self.create_timer(2.0, self.publish_markers)
        
        self.get_logger().info(
            "\n" + "="*60 + "\n"
            "📊 REACHABILITY MAP VISUALIZATION\n"
            "="*60 + "\n"
            "Open RViz and add:\n"
            "  1. MarkerArray display\n"
            "  2. Topic: /reachability_map_markers\n"
            "\n"
            "Legend:\n"
            "  🟢 GREEN spheres = Reachable points\n"
            "  🔴 RED spheres = Unreachable points\n"
            "  🟦 BLUE cube = Origin (0,0,0)\n"
            "="*60
        )
    
    def publish_markers(self):
        """Publish marker array showing reachability map"""
        marker_array = MarkerArray()
        
        # Create origin marker (blue cube at 0,0,0)
        origin_marker = Marker()
        origin_marker.header.frame_id = "base_link"
        origin_marker.header.stamp = self.get_clock().now().to_msg()
        origin_marker.ns = "origin"
        origin_marker.id = 0
        origin_marker.type = Marker.CUBE
        origin_marker.action = Marker.ADD
        origin_marker.pose.position.x = 0.0
        origin_marker.pose.position.y = 0.0
        origin_marker.pose.position.z = 0.0
        origin_marker.pose.orientation.w = 1.0
        origin_marker.scale.x = 0.05
        origin_marker.scale.y = 0.05
        origin_marker.scale.z = 0.05
        origin_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)
        origin_marker.lifetime.sec = 0  # Never delete
        marker_array.markers.append(origin_marker)
        
        # Add reachable points (green spheres)
        marker_id = 1
        for (x, y, z) in self.reach_map.reachability_grid.keys():
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "reachable"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = float(z)
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.02  # 2cm spheres
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            
            # Green for reachable
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.4)
            
            marker.lifetime.sec = 0  # Never delete
            
            marker_array.markers.append(marker)
            marker_id += 1
        
        # Optionally add unreachable points (red - but this would be many points)
        # For performance, we'll skip showing unreachable points by default
        
        self.get_logger().info(
            f"Publishing {len(marker_array.markers)} markers "
            f"({self.reach_map.reachable_points} reachable points + 1 origin)"
        )
        
        self.marker_pub.publish(marker_array)
    
    def publish_slice_markers(self, z_height=0.2):
        """
        Publish a 2D slice at specific height showing reachable (green) 
        and unreachable (red) points
        """
        marker_array = MarkerArray()
        marker_id = 0
        
        # Generate all grid points at this height
        x_min, x_max = -0.4, 0.4
        y_min, y_max = -0.4, 0.4
        resolution = self.reach_map.resolution
        
        import numpy as np
        x_points = np.arange(x_min, x_max, resolution)
        y_points = np.arange(y_min, y_max, resolution)
        
        for x in x_points:
            for y in y_points:
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = f"slice_z{z_height}"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                marker.pose.position.x = float(x)
                marker.pose.position.y = float(y)
                marker.pose.position.z = z_height
                marker.pose.orientation.w = 1.0
                
                marker.scale.x = resolution * 0.8
                marker.scale.y = resolution * 0.8
                marker.scale.z = 0.01  # Thin slice
                
                # Check if this point is reachable
                grid_key = (
                    round(x / resolution) * resolution,
                    round(y / resolution) * resolution,
                    round(z_height / resolution) * resolution
                )
                
                if grid_key in self.reach_map.reachability_grid:
                    # Green for reachable
                    marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7)
                else:
                    # Red for unreachable
                    marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3)
                
                marker.lifetime.sec = 0
                marker_array.markers.append(marker)
                marker_id += 1
        
        self.get_logger().info(f"Publishing slice at z={z_height}m with {len(marker_array.markers)} markers")
        self.marker_pub.publish(marker_array)


def main():
    """Main visualization function"""
    print("\n" + "="*70)
    print("🎨 REACHABILITY MAP RVIZ VISUALIZATION")
    print("="*70)
    print("\nThis will visualize the reachability map in RViz.")
    print("\nMake sure you have:")
    print("  1. Generated a reachability map (test_reachability.py option 3)")
    print("  2. Robot system launched")
    print("  3. RViz running")
    print("\nVisualization modes:")
    print("  1. 3D point cloud (all reachable points)")
    print("  2. 2D slice at specific height")
    print("  3. Both")
    print("\n" + "="*70)
    
    mode = input("\nSelect mode (1-3): ").strip()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        visualizer = ReachabilityVisualizer()
        
        if not visualizer.reach_map.reachability_grid:
            print("\n❌ No map loaded. Exiting...")
            return
        
        if mode == "1":
            print("\n🟢 Publishing 3D point cloud visualization...")
            print("   Open RViz and add MarkerArray: /reachability_map_markers")
            rclpy.spin(visualizer)
            
        elif mode == "2":
            z_height = float(input("Enter Z height for slice (e.g., 0.2): "))
            print(f"\n🔴🟢 Publishing 2D slice at z={z_height}m...")
            print("   Open RViz and add MarkerArray: /reachability_map_markers")
            
            # Publish slice once
            visualizer.publish_slice_markers(z_height)
            
            # Keep node alive
            rclpy.spin(visualizer)
            
        elif mode == "3":
            print("\n🟢 Publishing 3D point cloud + instructions for slices...")
            print("   Open RViz and add MarkerArray: /reachability_map_markers")
            
            # Start with 3D visualization
            rclpy.spin(visualizer)
        
        else:
            print("Invalid mode selected")
    
    except KeyboardInterrupt:
        print("\n\n👋 Visualization stopped")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()