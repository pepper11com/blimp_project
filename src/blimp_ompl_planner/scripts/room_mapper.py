#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import math
import time

class RoomMapper(Node):
    def __init__(self):
        super().__init__('room_mapper')
        
        # TF for camera position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Track camera positions
        self.positions = []
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.min_y = float('inf') 
        self.max_y = float('-inf')
        self.min_z = float('inf')
        self.max_z = float('-inf')
        
        # Timer to sample camera position
        self.timer = self.create_timer(0.5, self.sample_position)
        
        self.get_logger().info('Room Mapper started! Move around your room...')
        print("\n🗺️  ROOM MAPPING MODE")
        print("="*50)
        print("📷 Move your camera around the room edges")
        print("🔄 I'll track the maximum extents")
        print("⏱️  Let it run for 30+ seconds")
        print("❌ Press Ctrl+C when done")
        print("="*50)

    def sample_position(self):
        """Sample current camera position and track extents"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'camera_link', rclpy.time.Time()
            )
            
            pos = transform.transform.translation
            x, y, z = pos.x, pos.y, pos.z
            
            # Update extents
            self.min_x = min(self.min_x, x)
            self.max_x = max(self.max_x, x)
            self.min_y = min(self.min_y, y)
            self.max_y = max(self.max_y, y)
            self.min_z = min(self.min_z, z)
            self.max_z = max(self.max_z, z)
            
            # Store position
            self.positions.append((x, y, z))
            
            # Calculate current room dimensions
            width = self.max_x - self.min_x
            height = self.max_y - self.min_y
            depth = self.max_z - self.min_z
            
            # Print live update
            print(f"\r📍 Pos: ({x:.1f},{y:.1f},{z:.1f}) | Room: {width:.1f}×{height:.1f}×{depth:.1f}m | Samples: {len(self.positions)}", end='', flush=True)
            
        except Exception as e:
            print(f"\r❌ Cannot get camera position: {e}", end='', flush=True)

    def print_final_analysis(self):
        """Print final room analysis and bounds recommendation"""
        if len(self.positions) < 10:
            print(f"\n❌ Not enough data! Only {len(self.positions)} samples.")
            return
            
        width = self.max_x - self.min_x
        height = self.max_y - self.min_y  
        depth = self.max_z - self.min_z
        
        # Add safety padding (30% extra or minimum 2m)
        padding_x = max(width * 0.3, 2.0)
        padding_y = max(height * 0.3, 2.0)
        padding_z = max(depth * 0.3, 1.0)
        
        # Calculate suggested bounds
        suggested_min_x = self.min_x - padding_x
        suggested_max_x = self.max_x + padding_x
        suggested_min_y = self.min_y - padding_y
        suggested_max_y = self.max_y + padding_y
        suggested_min_z = max(self.min_z - 0.5, -1.0)  # Don't go too far underground
        suggested_max_z = self.max_z + padding_z
        
        print(f"\n\n🏠 ROOM ANALYSIS RESULTS")
        print("="*60)
        print(f"📊 Samples collected: {len(self.positions)}")
        print(f"📏 Camera traveled:")
        print(f"   X: {self.min_x:.2f} to {self.max_x:.2f} = {width:.2f}m")
        print(f"   Y: {self.min_y:.2f} to {self.max_y:.2f} = {height:.2f}m")
        print(f"   Z: {self.min_z:.2f} to {self.max_z:.2f} = {depth:.2f}m")
        
        print(f"\n🏡 ESTIMATED ROOM SIZE:")
        print(f"   {width:.1f}m × {height:.1f}m × {depth:.1f}m")
        
        print(f"\n💡 RECOMMENDED PLANNING BOUNDS:")
        print(f"   (Room + {padding_x:.1f}m padding)")
        
        print(f"\n🎯 COPY THIS TO blimp_planning_bounds.yaml:")
        print("-"*50)
        print(f"bounds_min_x: {suggested_min_x:.1f}")
        print(f"bounds_max_x: {suggested_max_x:.1f}")
        print(f"bounds_min_y: {suggested_min_y:.1f}")  
        print(f"bounds_max_y: {suggested_max_y:.1f}")
        print(f"bounds_min_z: {suggested_min_z:.1f}")
        print(f"bounds_max_z: {suggested_max_z:.1f}")
        print("-"*50)
        
        total_width = suggested_max_x - suggested_min_x
        total_height = suggested_max_y - suggested_min_y
        total_depth = suggested_max_z - suggested_min_z
        print(f"📦 Planning space: {total_width:.1f}×{total_height:.1f}×{total_depth:.1f}m")
        print("="*60)

def main():
    rclpy.init()
    mapper = RoomMapper()
    
    try:
        rclpy.spin(mapper)
    except KeyboardInterrupt:
        print(f"\n\n🛑 Mapping stopped by user")
        mapper.print_final_analysis()
    
    mapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()