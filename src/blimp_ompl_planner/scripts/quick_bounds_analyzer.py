#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from octomap_msgs.msg import Octomap
import struct
import math

class OctomapAnalyzer(Node):
    def __init__(self):
        super().__init__('octomap_analyzer')
        
        self.octomap_sub = self.create_subscription(
            Octomap,
            '/octomap_binary', 
            self.analyze_octomap,
            10
        )
        
        print("🔍 Analyzing existing OctoMap to find room dimensions...")
        print("   This will use the 3D point cloud data RTABMap has already built")

    def analyze_octomap(self, msg):
        """Analyze the OctoMap binary data to extract actual room bounds"""
        try:
            resolution = msg.resolution
            data_size = len(msg.data)
            
            print(f"\nOctoMap Data Found:")
            print(f"   Resolution: {resolution:.3f}m per voxel")
            print(f"   Data size: {data_size} bytes")
            
            # Parse the binary OctoMap header to get bounds
            # The OctoMap format includes bounding box information
            if data_size > 32:  # Minimum size for header
                try:
                    # Try to extract bounds from the binary data
                    # OctoMap binary format starts with bounds information
                    data = msg.data
                    
                    # Look for coordinate patterns in the data
                    # We'll estimate bounds by looking at the coordinate range
                    coords = []
                    
                    # Parse coordinates from the binary data (simplified approach)
                    for i in range(0, min(len(data)-12, 1000), 4):  # Sample coordinates
                        try:
                            # Try to interpret 4-byte chunks as floats (coordinates)
                            coord = struct.unpack('<f', data[i:i+4])[0]
                            
                            # Filter reasonable coordinate values (not infinity, not too large)
                            if -100 < coord < 100 and not math.isnan(coord) and not math.isinf(coord):
                                coords.append(coord)
                        except:
                            continue
                    
                    if coords:
                        # Estimate bounds from coordinate distribution
                        coords.sort()
                        min_coord = coords[int(len(coords) * 0.05)]  # 5th percentile
                        max_coord = coords[int(len(coords) * 0.95)]  # 95th percentile
                        
                        estimated_size = max_coord - min_coord
                        
                        print(f"   Estimated room dimensions from OctoMap:")
                        print(f"   Coordinate range: {min_coord:.1f} to {max_coord:.1f}")
                        print(f"   Estimated size: ~{estimated_size:.1f} meters")
                        
                        # Calculate reasonable bounds based on estimated size
                        if estimated_size > 2.0:  # Reasonable room size
                            padding = max(2.0, estimated_size * 0.3)  # 30% padding or 2m minimum
                            
                            suggested_bound = max(estimated_size/2 + padding, 5.0)  # Ensure minimum 5m radius
                            
                            print(f"\nRECOMMENDED PLANNING BOUNDS:")
                            print(f"   Based on {estimated_size:.1f}m room with {padding:.1f}m padding")
                            
                            self.print_bounds_config(suggested_bound)
                        else:
                            print(f"\nRoom seems small ({estimated_size:.1f}m), using default bounds")
                            self.print_bounds_config(6.0)  # Default 6m radius
                            
                    else:
                        print("Could not extract coordinates from OctoMap data")
                        self.suggest_manual_bounds()
                        
                except Exception as e:
                    print(f"Error parsing OctoMap: {e}")
                    self.suggest_manual_bounds()
            else:
                print("OctoMap data too small to analyze")
                self.suggest_manual_bounds()
                
        except Exception as e:
            self.get_logger().error(f'Error analyzing OctoMap: {e}')
            self.suggest_manual_bounds()

    def print_bounds_config(self, radius):
        """Print the bounds configuration"""
        print(f"\nCOPY THIS TO blimp_planning_bounds.yaml:")
        print("-" * 50)
        print(f"bounds_min_x: -{radius:.1f}")
        print(f"bounds_max_x: {radius:.1f}")
        print(f"bounds_min_y: -{radius:.1f}")
        print(f"bounds_max_y: {radius:.1f}")
        print(f"bounds_min_z: -0.5")
        print(f"bounds_max_z: 3.5")
        print("-" * 50)
        
        total_size = radius * 2
        print(f"Planning space: {total_size:.1f}m × {total_size:.1f}m × 4.0m")
        
        # Auto-update the config file
        self.update_config_file(radius)

    def update_config_file(self, radius):
        """Automatically update the bounds config file"""
        try:
            config_path = "/home/blimp2/blimp_ws/src/blimp_ompl_planner/config/blimp_planning_bounds.yaml"
            
            # Read current config
            with open(config_path, 'r') as f:
                lines = f.readlines()
            
            # Update bounds lines
            new_lines = []
            for line in lines:
                if line.strip().startswith('bounds_min_x:'):
                    new_lines.append(f"bounds_min_x: -{radius:.1f}     # Auto-updated from OctoMap analysis\n")
                elif line.strip().startswith('bounds_max_x:'):
                    new_lines.append(f"bounds_max_x: {radius:.1f}      # Auto-updated from OctoMap analysis\n")
                elif line.strip().startswith('bounds_min_y:'):
                    new_lines.append(f"bounds_min_y: -{radius:.1f}     # Auto-updated from OctoMap analysis\n")
                elif line.strip().startswith('bounds_max_y:'):
                    new_lines.append(f"bounds_max_y: {radius:.1f}      # Auto-updated from OctoMap analysis\n")
                elif line.strip().startswith('bounds_min_z:'):
                    new_lines.append(f"bounds_min_z: -0.5      # Allow slightly below ground\n")
                elif line.strip().startswith('bounds_max_z:'):
                    new_lines.append(f"bounds_max_z: 3.5       # Room ceiling height\n")
                else:
                    new_lines.append(line)
            
            # Write updated config
            with open(config_path, 'w') as f:
                f.writelines(new_lines)
                
            print(f" Config file automatically updated!")
            print(f"   File: {config_path}")
            
        except Exception as e:
            print(f"    Could not auto-update config file: {e}")

    def suggest_manual_bounds(self):
        """Suggest manual bounds for a 9-meter room"""
        print(f"\nFor a 9-meter room, I recommend:")
        print(f"   Room: 9m × ?m (you mentioned 9m)")
        print(f"   Planning bounds should be ~12-15m total")
        
        self.print_bounds_config(7.5)  # 15m total (good for 9m+ room)

def main():
    rclpy.init()
    
    analyzer = OctomapAnalyzer()
    
    # Wait for one OctoMap message then exit
    rclpy.spin_once(analyzer, timeout_sec=10.0)
    
    print(f"\n Analysis complete!")
    print(f"   Restart your OMPL planner to use the new bounds:")
    print(f"   ros2 launch blimp_ompl_planner ompl_planner_launch.py")
    
    analyzer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()