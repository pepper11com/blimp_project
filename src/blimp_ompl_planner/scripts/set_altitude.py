#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys

# ros2 run blimp_ompl_planner set_altitude.py 4.0  
# ros2 topic pub /set_altitude std_msgs/Float32 "data: 2.0"

def main():
    if len(sys.argv) != 2:
        print("Usage: set_altitude.py <height_in_meters>")
        print("Example: set_altitude.py 2.5")
        print("Range: -10.0 to 10.0 meters")
        return
    
    try:
        altitude = float(sys.argv[1])
        
        if altitude < -10.0 or altitude > 10.0:
            print("Altitude must be between -10.0 and 10.0 meters")
            return
            
        rclpy.init()
        node = rclpy.create_node('altitude_setter')
        
        pub = node.create_publisher(Float32, '/set_altitude', 10)
        
        # Wait a bit for publisher to be ready
        rclpy.spin_once(node, timeout_sec=0.1)
        
        msg = Float32()
        msg.data = altitude
        
        print(f" Setting altitude to {altitude:.1f}m...")
        pub.publish(msg)
        
        # Give time for message to be sent
        rclpy.spin_once(node, timeout_sec=0.1)
        
        print("Altitude setting sent")
        print(f"All /goal_pose messages will use Z={altitude:.1f}m")
        
        node.destroy_node()
        rclpy.shutdown()
        
    except ValueError:
        print("Invalid altitude value. Please enter a number.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()