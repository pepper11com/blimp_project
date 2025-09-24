#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        
        # Subscribe to planned paths
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )
        
        # Publisher for republishing the path (to keep it visible)
        self.path_pub = self.create_publisher(Path, '/planned_path_viz', 10)
        
        # Timer to keep republishing the last path
        self.timer = self.create_timer(1.0, self.republish_path)
        
        self.last_path = None
        
        self.get_logger().info('Path Visualizer started. Listening for planned paths...')

    def path_callback(self, msg):
        """Callback when a new path is received"""
        self.last_path = msg
        
        # Update timestamp to keep it fresh
        self.last_path.header.stamp = self.get_clock().now().to_msg()
        
        # Log path info
        num_points = len(msg.poses)
        if num_points > 0:
            start = msg.poses[0].pose.position
            goal = msg.poses[-1].pose.position
            
            path_length = self.calculate_path_length(msg)
            
            self.get_logger().info(f'📍 New path received!')
            self.get_logger().info(f'   Points: {num_points}')
            self.get_logger().info(f'   Start: ({start.x:.2f}, {start.y:.2f}, {start.z:.2f})')
            self.get_logger().info(f'   Goal:  ({goal.x:.2f}, {goal.y:.2f}, {goal.z:.2f})')
            self.get_logger().info(f'   Length: {path_length:.2f} meters')
            
        # Immediately republish
        self.republish_path()

    def calculate_path_length(self, path):
        """Calculate the total length of the path"""
        if len(path.poses) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(1, len(path.poses)):
            p1 = path.poses[i-1].pose.position
            p2 = path.poses[i].pose.position
            
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            dz = p2.z - p1.z
            
            total_length += math.sqrt(dx*dx + dy*dy + dz*dz)
        
        return total_length

    def republish_path(self):
        """Republish the last path to keep it visible in RViz"""
        if self.last_path is not None:
            # Update timestamp
            self.last_path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.last_path)

def main(args=None):
    rclpy.init(args=args)
    
    node = PathVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()