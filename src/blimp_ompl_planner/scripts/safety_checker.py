#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from octomap_msgs.msg import Octomap
import tf2_geometry_msgs
import math

class SafetyChecker(Node):
    def __init__(self):
        super().__init__('safety_checker')
        
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # OctoMap subscription
        self.octomap_sub = self.create_subscription(
            Octomap, '/octomap_binary', self.octomap_callback, 10
        )
        
        self.has_map = False
        self.get_logger().info('Safety Checker initialized. Monitoring camera position...')

    def octomap_callback(self, msg):
        """Track if we have map data"""
        self.has_map = True

    def check_current_safety(self):
        """Check if current camera position is safe for planning"""
        if not self.has_map:
            print("❌ No OctoMap data available yet!")
            return False
            
        try:
            # Get current camera pose
            transform = self.tf_buffer.lookup_transform(
                'map', 'camera_link', rclpy.time.Time()
            )
            
            pos = transform.transform.translation
            
            print(f"\n🎯 Current Camera Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
            
            # Check bounds
            bounds_ok = (-10 <= pos.x <= 10 and 
                        -10 <= pos.y <= 10 and 
                        -0.2 <= pos.z <= 8.0)
            
            if bounds_ok:
                print("✅ Position is within planning bounds")
                print(f"💡 Try this goal: python3 send_navigation_goal.py {pos.x + 1:.1f} {pos.y + 0.5:.1f} {pos.z + 0.3:.1f}")
            else:
                print("❌ Position is outside planning bounds!")
                print(f"   Bounds: X(-10,10), Y(-10,10), Z(-0.2,8.0)")
                
            return bounds_ok
            
        except Exception as e:
            print(f"❌ Could not get camera position: {e}")
            return False

def main():
    rclpy.init()
    checker = SafetyChecker()
    
    import time
    time.sleep(2)  # Wait for TF buffer
    
    try:
        while True:
            checker.check_current_safety()
            
            # Wait for user input
            input("\n Press ENTER to check again (Ctrl+C to quit)...")
            
    except KeyboardInterrupt:
        print("\n👋 Safety checker stopped")
    
    checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()