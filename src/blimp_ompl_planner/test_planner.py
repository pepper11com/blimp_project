#!/usr/bin/env python3

"""
Simple test script to verify the OMPL planner functionality.
This script publishes a test OctoMap and sends a planning goal.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
import time

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from octomap_msgs.msg import Octomap
from std_msgs.msg import Header
import octomap_msgs

class OMPLPlannerTester(Node):
    def __init__(self):
        super().__init__('ompl_planner_tester')
        
        # Create action client
        self.action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        
        # Create OctoMap publisher for testing
        self.octomap_pub = self.create_publisher(
            Octomap, 'octomap', 10)
        
        self.get_logger().info('OMPL Planner Tester initialized')
        
    def create_simple_octomap(self):
        """Create a simple test OctoMap with some obstacles"""
        # This is a simplified example - in practice you'd use the octomap library
        octomap_msg = Octomap()
        octomap_msg.header = Header()
        octomap_msg.header.stamp = self.get_clock().now().to_msg()
        octomap_msg.header.frame_id = "map"
        octomap_msg.binary = True
        octomap_msg.id = "OcTree"
        octomap_msg.resolution = 0.1
        
        # For testing purposes, create an empty map
        # In practice, you would serialize a real OctoMap here
        octomap_msg.data = b''  # Empty data for now
        
        return octomap_msg
    
    def send_planning_goal(self, x, y, z, yaw=0.0):
        """Send a planning goal to the planner"""
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available')
            return False
            
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = float(z)
        
        # Convert yaw to quaternion
        goal_msg.pose.pose.orientation = Quaternion()
        goal_msg.pose.pose.orientation.z = np.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = np.cos(yaw / 2.0)
        
        self.get_logger().info(f'Sending planning goal: ({x}, {y}, {z})')
        
        # Send goal
        future = self.action_client.send_goal_async(goal_msg)
        
        return future
    
    def test_planner(self):
        """Run a complete test of the planner"""
        self.get_logger().info('Starting planner test...')
        
        # First, publish a test OctoMap
        self.get_logger().info('Publishing test OctoMap...')
        octomap_msg = self.create_simple_octomap()
        
        # Publish the map multiple times to ensure it's received
        for _ in range(5):
            self.octomap_pub.publish(octomap_msg)
            time.sleep(0.1)
        
        self.get_logger().info('OctoMap published, waiting 2 seconds...')
        time.sleep(2.0)
        
        # Send a planning goal
        future = self.send_planning_goal(5.0, 3.0, 2.0)
        
        if future:
            self.get_logger().info('Goal sent, waiting for result...')
            return True
        else:
            self.get_logger().error('Failed to send goal')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    tester = OMPLPlannerTester()
    
    try:
        # Run the test
        if tester.test_planner():
            tester.get_logger().info('Test completed. Check the planner node logs for results.')
        else:
            tester.get_logger().error('Test failed')
            
        # Keep the node alive for a bit to see results
        time.sleep(5.0)
        
    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted')
    
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()