#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float32
import math

class Interactive3DGoalSelector(Node):
    """
    Interactive 3D Goal Selector using RViz Publish Point + altitude slider
    
    How to use:
    1. Use 'Publish Point' tool in RViz to click XY position
    2. Publish altitude on /goal_altitude topic (or use default 1.5m)
    3. Automatically sends 3D goal to planner
    """
    def __init__(self):
        super().__init__('interactive_3d_goal_selector')
        
        # Subscribe to clicked points from RViz
        self.point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10
        )
        
        # Subscribe to altitude commands
        self.altitude_sub = self.create_subscription(
            Float32,
            '/goal_altitude',
            self.altitude_callback,
            10
        )
        
        # Publisher for complete 3D goal
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        # Current altitude setting
        self.target_altitude = 1.5  # Default flying altitude
        
        self.get_logger().info('Interactive 3D Goal Selector ready')
        self.get_logger().info('Set "Publish Point" to set XY position')
        self.get_logger().info(f'Current altitude: {self.target_altitude:.1f}m')
        self.get_logger().info('Change altitude: ros2 topic pub /goal_altitude std_msgs/Float32 "data: 2.0"')

    def altitude_callback(self, msg):
        """Update target altitude"""
        self.target_altitude = max(0.5, min(5.0, msg.data))  # Clamp between 0.5-5m
        self.get_logger().info(f'Altitude updated to: {self.target_altitude:.1f}m')

    def point_callback(self, msg):
        """Handle clicked point from RViz"""
        x = msg.point.x
        y = msg.point.y
        z = self.target_altitude  # Use our altitude setting
        
        self.get_logger().info(f'Setting 3D goal: ({x:.2f}, {y:.2f}, {z:.1f})')
        
        # Create PoseStamped message
        goal_msg = PoseStamped()
        goal_msg.header = msg.header
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = z
        
        # Orientation (facing forward)
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        
        # Publish goal
        self.goal_pub.publish(goal_msg)
        self.get_logger().info('3D goal sent to navigation system!')

def main(args=None):
    rclpy.init(args=args)
    node = Interactive3DGoalSelector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()