#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

# ros2 run blimp_ompl_planner set_altitude.py 4.0  
# ros2 topic pub /set_altitude std_msgs/Float32 "data: 2.0"

class RVizGoalHandler(Node):
    """
    Converts goal poses into NavigateToPose actions for OMPL planner
    Supports separate altitude setting via /set_altitude topic
    """
    def __init__(self):
        super().__init__('rviz_goal_handler')
        
        # Subscribe to goal poses (from Foxglove, RViz, etc.)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Subscribe to altitude setting
        self.altitude_sub = self.create_subscription(
            Float32,
            '/set_altitude',
            self.altitude_callback,
            10
        )
        
        # Action client for OMPL planner
        self.action_client = ActionClient(
            self,
            NavigateToPose, 
            '/navigate_to_pose'
        )
        
        # Current target altitude (persistent)
        self.target_altitude = 1.5  # Default flying altitude
        
        self.get_logger().info('Goal Handler started!')
        self.get_logger().info(f'Default altitude: {self.target_altitude}m')
        self.get_logger().info('Send goals to /goal_pose topic (X,Y from goal, Z from altitude setting)')
        self.get_logger().info('Set altitude: ros2 topic pub /set_altitude std_msgs/Float32 "data: 2.0"')

    def altitude_callback(self, msg):
        """Update target altitude setting"""
        new_altitude = max(-10.0, min(10.0, msg.data))  # Clamp between -10.0-10.0m
        self.target_altitude = new_altitude
        self.get_logger().info(f'Altitude set to: {self.target_altitude:.1f}m (will be used for next goals)')

    def goal_callback(self, msg):
        """Handle goal pose - use X,Y from message, Z from altitude setting"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        z_original = msg.pose.position.z
        
        # Always use our persistent altitude setting
        z = self.target_altitude
        
        self.get_logger().info(f'Received goal: X={x:.2f}, Y={y:.2f}, Z_orig={z_original:.2f}')
        self.get_logger().info(f' Using altitude setting: {z:.1f}m')
        
        # Create modified pose with our altitude
        modified_pose = PoseStamped()
        modified_pose.header = msg.header
        modified_pose.header.stamp = self.get_clock().now().to_msg()
        modified_pose.pose.position.x = x
        modified_pose.pose.position.y = y
        modified_pose.pose.position.z = z
        modified_pose.pose.orientation = msg.pose.orientation  # Keep original orientation
        
        # Send to OMPL planner
        self.send_navigation_goal(modified_pose)

    def send_navigation_goal(self, pose_stamped):
        """Send goal to OMPL planner"""
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('OMPL planner not available!')
            return
        
        # Create NavigateToPose goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        
        # Update timestamp
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info(' Sending goal to OMPL planner...')
        
        # Send goal asynchronously
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle response from planner"""
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info(' Goal accepted by planner!')
            
            # Get result
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().error(' Goal rejected by planner!')

    def goal_result_callback(self, future):
        """Handle final result"""
        result = future.result().result
        if result:
            self.get_logger().info(' Navigation completed successfully!')
        else:
            self.get_logger().error(' Navigation failed!')

def main(args=None):
    rclpy.init(args=args)
    node = RVizGoalHandler()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()