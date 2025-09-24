#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import math
import time

class SimplePoseChecker(Node):
    def __init__(self):
        super().__init__('simple_pose_checker')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
    def check_pose(self):
        """Simple pose check using tf2_echo equivalent method"""
        # Wait a bit for buffer to fill
        time.sleep(3)
        
        try:
            # Get the latest transform
            transform = self.tf_buffer.lookup_transform(
                'map', 'camera_link', 
                rclpy.time.Time(),  # Latest available
                rclpy.duration.Duration(seconds=1.0)
            )
            
            pos = transform.transform.translation
            rot = transform.transform.rotation
            
            print("\n" + "="*60)
            print("🎯 CURRENT CAMERA_LINK POSE IN MAP FRAME")
            print("="*60)
            print(f"📍 Position:")
            print(f"   X: {pos.x:.3f} m")
            print(f"   Y: {pos.y:.3f} m") 
            print(f"   Z: {pos.z:.3f} m")
            
            print(f"\n🎯 SUGGESTED TEST WAYPOINTS:")
            print(f"   Forward 1m:  X={pos.x + 1.0:.1f}, Y={pos.y:.1f}, Z={pos.z:.1f}")
            print(f"   Right 1m:    X={pos.x:.1f}, Y={pos.y - 1.0:.1f}, Z={pos.z:.1f}")  
            print(f"   Up 0.5m:     X={pos.x:.1f}, Y={pos.y:.1f}, Z={pos.z + 0.5:.1f}")
            print(f"   Diagonal:    X={pos.x + 0.5:.1f}, Y={pos.y + 0.5:.1f}, Z={pos.z + 0.3:.1f}")
            
            print(f"\n💡 EXAMPLE NAVIGATION COMMANDS:")
            print(f"   python3 send_navigation_goal.py {pos.x + 1.0:.1f} {pos.y:.1f} {pos.z:.1f}")
            print(f"   python3 send_navigation_goal.py {pos.x:.1f} {pos.y - 1.0:.1f} {pos.z:.1f}")  
            print(f"   python3 send_navigation_goal.py {pos.x:.1f} {pos.y:.1f} {pos.z + 0.5:.1f}")
            print("="*60)
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to get transform: {e}")
            return False

def main():
    rclpy.init()
    node = SimplePoseChecker()
    
    success = node.check_pose()
    
    node.destroy_node()
    rclpy.shutdown()
    
    return success

if __name__ == '__main__':
    main()