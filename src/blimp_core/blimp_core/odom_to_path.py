#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math
from collections import deque

class OdomToPathNode(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        
        # Create a Path message to store the trajectory
        self.path = Path()
        
        # Use a deque for efficient appends and pops from both ends
        self.poses_deque = deque()
        self.total_length = 0.0
        self.max_length = 10.0  # meters

        # Create a subscriber to the odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # Subscribe to the odometry topic from stereo_odometry
            self.odom_callback,
            10)
            
        # Create a publisher for the path topic
        self.publisher = self.create_publisher(Path, '/path', 10)
        
        self.get_logger().info('Odometry to Path node has been started.')

    def odom_callback(self, msg: Odometry):
        # The path frame_id should be the same as the odometry frame_id
        self.path.header = msg.header
        
        # Create a PoseStamped message from the Odometry message
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        
        # Add new pose and update length
        if self.poses_deque:
            last_pose = self.poses_deque[-1]
            self.total_length += self.calculate_distance(last_pose, pose)
        
        self.poses_deque.append(pose)
        
        # Prune path if it exceeds max length
        while self.total_length > self.max_length and len(self.poses_deque) > 1:
            first_pose = self.poses_deque[0]
            second_pose = self.poses_deque[1]
            self.total_length -= self.calculate_distance(first_pose, second_pose)
            self.poses_deque.popleft()

        # Update path message from deque
        self.path.poses = list(self.poses_deque)
        
        # Publish the updated path
        self.publisher.publish(self.path)

    def calculate_distance(self, pose1, pose2):
        p1 = pose1.pose.position
        p2 = pose2.pose.position
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
