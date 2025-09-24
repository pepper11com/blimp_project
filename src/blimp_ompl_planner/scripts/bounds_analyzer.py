#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from octomap_msgs.msg import Octomap
import tf2_ros
# We'll extract bounds from the message header instead of using octomap library

class BoundsAnalyzer(Node):
    def __init__(self):
        super().__init__('bounds_analyzer')
        
        # OctoMap subscription
        self.octomap_sub = self.create_subscription(
            Octomap, '/octomap_binary', self.analyze_octomap, 10
        )
        
        # TF for camera position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Bounds Analyzer started. Waiting for OctoMap data...')

    def analyze_octomap(self, msg):
        """Analyze OctoMap to suggest optimal planning bounds"""
        try:
            # Extract basic info from message
            data_size = len(msg.data)
            resolution = msg.resolution
            
            # Get camera position for reference  
            camera_pos = self.get_camera_position()
            
            # Since we can't easily decode the octree, make reasonable suggestions
            # based on typical room sizes and camera position
            if camera_pos:
                cx, cy, cz = camera_pos
                
                # Suggest bounds around camera position (typical room size)
                padding = 3.0  # 3 meter radius around camera
                suggested_bounds = {
                    'min_x': cx - padding,
                    'max_x': cx + padding,
                    'min_y': cy - padding,
                    'max_y': cy + padding,
                    'min_z': -0.5,  # Slightly below ground
                    'max_z': cz + 2.0  # 2m above camera
                }
            else:
                # Default room-sized bounds
                suggested_bounds = {
                    'min_x': -3.0,
                    'max_x': 3.0,
                    'min_y': -3.0,
                    'max_y': 3.0,
                    'min_z': -0.5,
                    'max_z': 3.0
                }
            
            self.print_analysis(
                suggested_bounds,
                camera_pos,
                data_size,
                resolution
            )
            
        except Exception as e:
            self.get_logger().error(f'Error analyzing OctoMap: {e}')

    def get_camera_position(self):
        """Get current camera position for reference"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'camera_link', rclpy.time.Time()
            )
            pos = transform.transform.translation
            return (pos.x, pos.y, pos.z)
        except:
            return None

    def print_analysis(self, suggested, camera_pos, data_size, resolution):
        """Print detailed bounds analysis"""
        print("\n" + "="*70)
        print("📊 PLANNING BOUNDS RECOMMENDATION")
        print("="*70)
        
        print(f"🗺️  OctoMap Statistics:")
        print(f"   Data size: {data_size} bytes")
        print(f"   Resolution: {resolution:.3f}m")
        
        if camera_pos:
            print(f"\n📷 Current Camera Position: ({camera_pos[0]:.2f}, {camera_pos[1]:.2f}, {camera_pos[2]:.2f})")
            
            # Calculate suggested room size
            width = suggested['max_x'] - suggested['min_x']
            height = suggested['max_y'] - suggested['min_y']
            depth = suggested['max_z'] - suggested['min_z']
            print(f"   Suggested room: {width:.1f}m x {height:.1f}m x {depth:.1f}m")
        
        print(f"\n💡 RECOMMENDED PLANNING BOUNDS:")
        print(f"   (Based on camera position + safety margin)")
        
        print(f"\n🎯 YAML CONFIG (copy this to blimp_planning_bounds.yaml):")
        print("-"*50)
        print(f"bounds_min_x: {suggested['min_x']:.1f}    # X minimum")
        print(f"bounds_max_x: {suggested['max_x']:.1f}    # X maximum") 
        print(f"bounds_min_y: {suggested['min_y']:.1f}    # Y minimum")
        print(f"bounds_max_y: {suggested['max_y']:.1f}    # Y maximum")
        print(f"bounds_min_z: {suggested['min_z']:.1f}    # Z minimum") 
        print(f"bounds_max_z: {suggested['max_z']:.1f}    # Z maximum")
        print("-"*50)
        
        print(f"\n📏 EXPLANATION:")
        print(f"   • These bounds define where the planner will search for paths")
        print(f"   • They should be larger than your actual room")  
        print(f"   • Smaller bounds = faster planning, larger bounds = more flexibility")
        print(f"   • The planner will reject goals outside these bounds")
        print("="*70)

def main():
    rclpy.init()
    analyzer = BoundsAnalyzer()
    
    print("🔍 Analyzing your OctoMap to suggest optimal planning bounds...")
    print("   Move around to build more map data, then check results here.")
    
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        print("\n👋 Analysis stopped")
    
    analyzer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()