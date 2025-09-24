#!/usr/bin/env python3

import rclpy
import rclpy.duration
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
import math

class PoseChecker(Node):
    def __init__(self):
        super().__init__('pose_checker')
        
        # Create TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Pose Checker initialized. Checking camera_link pose...')

    def get_current_pose(self):
        """Get current camera_link pose in map frame"""
        try:
            # Wait for transform to become available
            if self.tf_buffer.can_transform('map', 'camera_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0)):
                # Look up transform from map to camera_link
                transform = self.tf_buffer.lookup_transform(
                    'map',           # target frame
                    'camera_link',   # source frame
                    rclpy.time.Time() # get latest available
                )
                return transform
            else:
                self.get_logger().error('Transform map->camera_link not available after 5 seconds')
                return None
            
        except TransformException as ex:
            self.get_logger().error(f'Could not transform map to camera_link: {ex}')
            return None

    def quaternion_to_euler(self, quat):
        """Convert quaternion to euler angles (roll, pitch, yaw)"""
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

    def print_pose_info(self):
        """Print current pose and suggest nearby waypoints"""
        transform = self.get_current_pose()
        
        if transform is None:
            # Try to get available frames
            print("\n❌ Could not get map->camera_link transform!")
            print("\n🔍 DEBUGGING TF TREE:")
            print("Let's check what transforms are available...")
            
            # Try odom->camera_link
            try:
                if self.tf_buffer.can_transform('odom', 'camera_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0)):
                    odom_transform = self.tf_buffer.lookup_transform(
                        'odom', 'camera_link', rclpy.time.Time()
                    )
                    print("✅ odom->camera_link transform EXISTS")
                    self.print_transform_info(odom_transform, "odom")
                    return True
                else:
                    print("❌ odom->camera_link transform NOT available (timeout)")
            except Exception as e:
                print(f"❌ odom->camera_link transform ERROR: {e}")
            
            # Try base_link->camera_link  
            try:
                base_transform = self.tf_buffer.lookup_transform(
                    'base_link', 'camera_link', rclpy.time.Time()
                )
                print("✅ base_link->camera_link transform EXISTS")
                print("   (This means camera_link is isolated from odom/map)")
            except:
                print("❌ base_link->camera_link transform NOT available")
            
            print(f"\n💡 POSSIBLE SOLUTIONS:")
            print(f"   1. RTABMap might not be properly configured")
            print(f"   2. The blimp localization launch might need to be started")
            print(f"   3. Camera->odom transform might be missing")
            print(f"\n🔧 Try: ros2 launch blimp_core blimp_localization_slam_launch.py")
            
            return False
            
        pos = transform.transform.translation
        rot = transform.transform.rotation
        
        # Convert quaternion to euler angles
        roll, pitch, yaw = self.quaternion_to_euler(rot)
        
        print("\n" + "="*60)
        print("🎯 CURRENT CAMERA_LINK POSE IN MAP FRAME")
        print("="*60)
        print(f"📍 Position:")
        print(f"   X: {pos.x:.3f} m")
        print(f"   Y: {pos.y:.3f} m") 
        print(f"   Z: {pos.z:.3f} m")
        print(f"\n🧭 Orientation:")
        print(f"   Roll:  {math.degrees(roll):.1f}°")
        print(f"   Pitch: {math.degrees(pitch):.1f}°")
        print(f"   Yaw:   {math.degrees(yaw):.1f}°")
        print(f"\n🤖 Quaternion:")
        print(f"   x: {rot.x:.3f}")
        print(f"   y: {rot.y:.3f}")
        print(f"   z: {rot.z:.3f}")
        print(f"   w: {rot.w:.3f}")
        
        # Suggest some nearby waypoints
        print(f"\n🎯 SUGGESTED TEST WAYPOINTS:")
        print(f"   Forward 2m:  X={pos.x + 2.0:.1f}, Y={pos.y:.1f}, Z={pos.z:.1f}")
        print(f"   Right 2m:    X={pos.x:.1f}, Y={pos.y - 2.0:.1f}, Z={pos.z:.1f}")
        print(f"   Up 1m:       X={pos.x:.1f}, Y={pos.y:.1f}, Z={pos.z + 1.0:.1f}")
        print(f"   Diagonal:    X={pos.x + 1.5:.1f}, Y={pos.y + 1.5:.1f}, Z={pos.z + 0.5:.1f}")
        
        print(f"\n💡 EXAMPLE COMMANDS:")
        print(f"   python3 send_navigation_goal.py {pos.x + 2.0:.1f} {pos.y:.1f} {pos.z:.1f}")
        print(f"   python3 send_navigation_goal.py {pos.x:.1f} {pos.y - 2.0:.1f} {pos.z:.1f}")
        print(f"   python3 send_navigation_goal.py {pos.x:.1f} {pos.y:.1f} {pos.z + 1.0:.1f}")
        print("="*60)
        
        return True

    def print_transform_info(self, transform, frame_name):
        """Print transform information for any frame"""
        pos = transform.transform.translation
        rot = transform.transform.rotation
        
        # Convert quaternion to euler angles
        roll, pitch, yaw = self.quaternion_to_euler(rot)
        
        print(f"\n🎯 CURRENT CAMERA_LINK POSE IN {frame_name.upper()} FRAME")
        print("="*60)
        print(f"📍 Position:")
        print(f"   X: {pos.x:.3f} m")
        print(f"   Y: {pos.y:.3f} m") 
        print(f"   Z: {pos.z:.3f} m")
        print(f"\n🧭 Orientation:")
        print(f"   Roll:  {math.degrees(roll):.1f}°")
        print(f"   Pitch: {math.degrees(pitch):.1f}°")
        print(f"   Yaw:   {math.degrees(yaw):.1f}°")
        
        # Suggest some nearby waypoints (in the available frame)
        print(f"\n🎯 SUGGESTED TEST WAYPOINTS (in {frame_name} frame):")
        print(f"   Forward 2m:  X={pos.x + 2.0:.1f}, Y={pos.y:.1f}, Z={pos.z:.1f}")
        print(f"   Right 2m:    X={pos.x:.1f}, Y={pos.y - 2.0:.1f}, Z={pos.z:.1f}")
        print(f"   Up 1m:       X={pos.x:.1f}, Y={pos.y:.1f}, Z={pos.z + 1.0:.1f}")
        
        if frame_name != 'map':
            print(f"\n⚠️  NOTE: These coordinates are in '{frame_name}' frame, not 'map' frame!")
            print(f"   You'll need the full localization stack for map-based navigation.")
        
        return True

def main(args=None):
    rclpy.init(args=args)
    
    node = PoseChecker()
    
    # Give TF some time to populate
    import time
    print("⏳ Waiting for TF buffer to populate...")
    time.sleep(3.0)
    
    try:
        success = node.print_pose_info()
        if not success:
            print("\n❌ Could not get pose. Make sure:")
            print("   1. RTABMap is running")
            print("   2. Camera is connected and publishing")
            print("   3. TF transforms are being published")
            print("\n🔧 Try running:")
            print("   ros2 launch blimp_core blimp_localization_slam_launch.py")
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()