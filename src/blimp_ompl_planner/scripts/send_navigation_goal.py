#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import sys

class NavigationGoalSender(Node):
    def __init__(self):
        super().__init__('navigation_goal_sender')
        
        # Create action client
        self.action_client = ActionClient(
            self, 
            NavigateToPose, 
            '/navigate_to_pose'
        )
        
        self.get_logger().info('Navigation Goal Sender initialized')

    def send_goal(self, x, y, z, yaw=0.0):
        """Send a navigation goal to the OMPL planner"""
        
        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return False
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        
        # Set pose (goal is in map frame, but robot frame is camera_link)
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        
        # Convert yaw to quaternion (simplified - only yaw rotation)
        import math
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.get_logger().info(f'Sending goal: ({x:.1f}, {y:.1f}, {z:.1f})')
        
        # Send goal
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False
        
        self.get_logger().info('Goal accepted! Waiting for result...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result:
            self.get_logger().info('Navigation succeeded!')
            return True
        else:
            self.get_logger().error('Navigation failed!')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 4:
        print("Usage: python3 send_navigation_goal.py <x> <y> <z> [yaw]")
        print("Example: python3 send_navigation_goal.py 2.0 1.0 1.5 0.0")
        return
    
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    yaw = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    
    node = NavigationGoalSender()
    success = node.send_goal(x, y, z, yaw)
    
    node.destroy_node()
    rclpy.shutdown()
    
    return 0 if success else 1

if __name__ == '__main__':
    main()