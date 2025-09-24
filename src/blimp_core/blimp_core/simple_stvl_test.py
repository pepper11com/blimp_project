#!/usr/bin/env python3
"""
Simple STVL test without Nav2 costmap - just to verify STVL functionality
This creates a minimal costmap node to test STVL plugin directly
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav2_costmap_2d.costmap_2d import Costmap2D
from nav2_costmap_2d.costmap_2d_ros import Costmap2DROS
import threading
import time

class SimpleSTVLTest(Node):
    def __init__(self):
        super().__init__('simple_stvl_test')
        
        self.get_logger().info("Starting Simple STVL Test Node")
        
        # Set parameters for a minimal costmap with STVL
        self.declare_parameters(namespace='', parameters=[
            ('global_frame', 'map'),
            ('robot_base_frame', 'base_link'),
            ('width', 5),
            ('height', 5), 
            ('resolution', 0.2),
            ('rolling_window', True),
            ('track_unknown_space', False),
            ('plugins', ['stvl_layer']),
            
            # STVL plugin parameters
            ('stvl_layer.plugin', 'spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer'),
            ('stvl_layer.enabled', True),
            ('stvl_layer.voxel_decay', 3.0),
            ('stvl_layer.decay_model', 0),
            ('stvl_layer.voxel_size', 0.1),
            ('stvl_layer.mark_threshold', 0),
            ('stvl_layer.obstacle_range', 3.0),
            ('stvl_layer.max_obstacle_height', 2.0),
            ('stvl_layer.track_unknown_space', False),
            ('stvl_layer.publish_voxel_map', True),
            ('stvl_layer.mapping_mode', False),
            ('stvl_layer.map_save_duration', 60),
            ('stvl_layer.observation_sources', 'test_cloud'),
            ('stvl_layer.test_cloud.enabled', True),
            ('stvl_layer.test_cloud.data_type', 'PointCloud2'),
            ('stvl_layer.test_cloud.topic', '/cloud_obstacles'),
            ('stvl_layer.test_cloud.marking', True),
            ('stvl_layer.test_cloud.clearing', False),
            ('stvl_layer.test_cloud.min_obstacle_height', 0.1),
            ('stvl_layer.test_cloud.max_obstacle_height', 2.0),
            ('stvl_layer.test_cloud.expected_update_rate', 5.0),
            ('stvl_layer.test_cloud.filter', 'voxel'),
            ('stvl_layer.test_cloud.voxel_min_points', 1),
        ])
        
        try:
            # Create a simple costmap instance
            self.costmap = Costmap2DROS(self)
            self.get_logger().info("STVL Test: Costmap created successfully!")
            
            # Start a timer to report status
            self.timer = self.create_timer(5.0, self.report_status)
            
        except Exception as e:
            self.get_logger().error(f"STVL Test failed: {e}")
    
    def report_status(self):
        self.get_logger().info("STVL Test is running...")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SimpleSTVLTest()
        
        # Use MultiThreadedExecutor for lifecycle management
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            node.destroy_node()
    except Exception as e:
        print(f"Failed to start STVL test: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()