#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped
import time

class CollisionTester(Node):
    def __init__(self):
        super().__init__('collision_tester')
        
        # TF2 for camera position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Collision Tester initialized')

    def get_current_pose(self):
        """Get current camera pose"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'camera_link', rclpy.time.Time()
            )
            
            pos = transform.transform.translation
            return (pos.x, pos.y, pos.z)
        except:
            return None

    def test_collision_scenarios(self):
        """Test different collision scenarios"""
        current_pos = self.get_current_pose()
        
        if not current_pos:
            print("❌ Cannot get current position!")
            return
            
        x, y, z = current_pos
        
        print(f"\n🎯 COLLISION TESTING WITH 7.5m ROBOT RADIUS")
        print("="*60)
        print(f"📍 Current position: ({x:.2f}, {y:.2f}, {z:.2f})")
        print(f"🤖 Robot radius: 7.5m")
        print(f"🔍 The planner should avoid obstacles within 7.5m of these goals:")
        
        # Test goals at different distances
        test_goals = [
            (x + 1.0, y, z, "1m away - should be REJECTED (too close to robot)"),
            (x + 5.0, y, z, "5m away - should be REJECTED (within 7.5m radius)"),  
            (x + 8.0, y, z, "8m away - should be ACCEPTED (outside 7.5m radius)"),
            (x, y + 1.0, z + 1.0, "1m diagonal - should be REJECTED"),
            (x, y, z + 2.0, "2m up - should be REJECTED"),
        ]
        
        for gx, gy, gz, description in test_goals:
            distance = ((gx-x)**2 + (gy-y)**2 + (gz-z)**2)**0.5
            
            print(f"\n🎯 Test: {description}")
            print(f"   Goal: ({gx:.1f}, {gy:.1f}, {gz:.1f})")
            print(f"   Distance: {distance:.1f}m")
            print(f"   Expected: {'REJECT' if distance < 7.5 else 'ACCEPT'}")
            print(f"   Command: python3 send_navigation_goal.py {gx:.1f} {gy:.1f} {gz:.1f}")

def main():
    rclpy.init()
    tester = CollisionTester()
    
    # Wait for TF
    time.sleep(2)
    
    tester.test_collision_scenarios()
    
    print(f"\n💡 HOW TO TEST:")
    print(f"1. Run: ros2 launch blimp_ompl_planner ompl_planner_launch.py")
    print(f"2. Try the test commands above")  
    print(f"3. Goals within 7.5m should be REJECTED")
    print(f"4. Goals beyond 7.5m should be ACCEPTED")
    print(f"5. Look for debug messages about collision checking")
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()