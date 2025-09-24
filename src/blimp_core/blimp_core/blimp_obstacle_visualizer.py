#!/usr/bin/env python3
"""
Real-time Obstacle Visualizer for Blimp
Creates live obstacle visualization from stereo depth without Nav2 dependencies
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import struct
import time

class BlimpObstacleVisualizer(Node):
    def __init__(self):
        super().__init__('blimp_obstacle_visualizer')
        
        self.get_logger().info("🚁 Starting Blimp Real-time Obstacle Visualizer")
        
        # QoS for reliable subscription
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers for visualization
        self.obstacle_pub = self.create_publisher(
            MarkerArray, '/blimp/obstacles', qos)
        self.processed_cloud_pub = self.create_publisher(
            PointCloud2, '/blimp/obstacle_cloud', qos)
        
        # Subscribe to RTABMap's local point clouds
        self.create_subscription(
            PointCloud2, '/odom_last_frame', 
            self.process_point_cloud, qos)
        
        # Parameters
        self.declare_parameters(namespace='', parameters=[
            ('obstacle_height_min', 0.1),    # Min obstacle height (m)
            ('obstacle_height_max', 2.5),    # Max obstacle height (m) 
            ('obstacle_range', 4.0),         # Max detection range (m)
            ('voxel_size', 0.15),           # Voxel grid resolution (m)
            ('decay_time', 3.0),            # How long obstacles persist (s)
            ('update_rate', 5.0),           # Visualization update rate (Hz)
        ])
        
        # Obstacle storage with timestamps
        self.obstacles = {}  # {(x,y,z): timestamp}
        self.last_cleanup = time.time()
        
        # Timer for regular updates
        update_period = 1.0 / self.get_parameter('update_rate').value
        self.timer = self.create_timer(update_period, self.update_visualization)
        
        self.get_logger().info("✅ Obstacle visualizer ready!")
    
    def process_point_cloud(self, msg):
        """Process incoming point cloud to extract obstacles"""
        try:
            # Get parameters
            min_height = self.get_parameter('obstacle_height_min').value
            max_height = self.get_parameter('obstacle_height_max').value
            max_range = self.get_parameter('obstacle_range').value
            voxel_size = self.get_parameter('voxel_size').value
            
            # Parse point cloud data
            points = self.parse_pointcloud2(msg)
            if points is None or len(points) == 0:
                return
                
            current_time = time.time()
            
            # Filter points by height and range
            valid_points = []
            for x, y, z in points:
                # Check height bounds
                if min_height <= z <= max_height:
                    # Check range from robot (assuming robot at origin)
                    distance = np.sqrt(x*x + y*y)
                    if distance <= max_range:
                        valid_points.append((x, y, z))
            
            # Voxelize points to reduce noise
            voxelized = self.voxelize_points(valid_points, voxel_size)
            
            # Add to obstacle storage
            for point in voxelized:
                self.obstacles[point] = current_time
            
            # Publish processed cloud for debugging
            if valid_points:
                processed_msg = self.create_pointcloud2(valid_points, msg.header)
                self.processed_cloud_pub.publish(processed_msg)
                
        except Exception as e:
            self.get_logger().warn(f"Error processing point cloud: {e}")
    
    def parse_pointcloud2(self, msg):
        """Parse PointCloud2 message into list of (x,y,z) points"""
        try:
            # Find x, y, z field offsets
            x_offset = y_offset = z_offset = None
            for field in msg.fields:
                if field.name == 'x':
                    x_offset = field.offset
                elif field.name == 'y':
                    y_offset = field.offset
                elif field.name == 'z':
                    z_offset = field.offset
            
            if None in [x_offset, y_offset, z_offset]:
                return None
            
            points = []
            point_step = msg.point_step
            
            for i in range(0, len(msg.data), point_step):
                if i + 12 <= len(msg.data):  # Ensure we have enough data
                    x = struct.unpack_from('f', msg.data, i + x_offset)[0]
                    y = struct.unpack_from('f', msg.data, i + y_offset)[0] 
                    z = struct.unpack_from('f', msg.data, i + z_offset)[0]
                    
                    # Filter out invalid points
                    if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                        points.append((x, y, z))
                        
            return points
            
        except Exception as e:
            self.get_logger().debug(f"Parse error: {e}")
            return None
    
    def voxelize_points(self, points, voxel_size):
        """Reduce point density by voxelization"""
        voxel_dict = {}
        
        for x, y, z in points:
            # Convert to voxel coordinates
            vx = int(x / voxel_size)
            vy = int(y / voxel_size) 
            vz = int(z / voxel_size)
            
            # Store center of voxel
            center_x = (vx + 0.5) * voxel_size
            center_y = (vy + 0.5) * voxel_size
            center_z = (vz + 0.5) * voxel_size
            
            voxel_dict[(vx, vy, vz)] = (center_x, center_y, center_z)
        
        return list(voxel_dict.values())
    
    def create_pointcloud2(self, points, header):
        """Create PointCloud2 message from points"""
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False
        
        # Define fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        
        # Pack data
        data = []
        for x, y, z in points:
            data.extend(struct.pack('fff', x, y, z))
        
        msg.data = bytes(data)
        return msg
    
    def update_visualization(self):
        """Update obstacle visualization markers"""
        current_time = time.time()
        decay_time = self.get_parameter('decay_time').value
        
        # Clean up old obstacles
        if current_time - self.last_cleanup > 1.0:  # Cleanup every second
            expired = []
            for point, timestamp in self.obstacles.items():
                if current_time - timestamp > decay_time:
                    expired.append(point)
            
            for point in expired:
                del self.obstacles[point]
            
            self.last_cleanup = current_time
        
        # Create marker array
        marker_array = MarkerArray()
        
        if self.obstacles:
            # Create cube list marker for efficiency
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'blimp_obstacles'
            marker.id = 0
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            
            # Marker properties
            marker.scale.x = 0.1  # Cube size
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 0.8  # Semi-transparent
            marker.color.r = 1.0  # Red obstacles
            marker.color.g = 0.2
            marker.color.b = 0.0
            
            # Add all obstacle points
            for (x, y, z), timestamp in self.obstacles.items():
                # Fade based on age
                age = current_time - timestamp
                alpha = max(0.2, 1.0 - (age / decay_time))
                
                point = Point()
                point.x, point.y, point.z = x, y, z
                marker.points.append(point)
                
                # Color based on age (newer = brighter red)
                color = marker.color
                color.a = alpha
                marker.colors.append(color)
            
            marker_array.markers.append(marker)
        
        # Always publish (even empty) to clear old markers
        self.obstacle_pub.publish(marker_array)
        
        if self.obstacles:
            self.get_logger().debug(f"Visualizing {len(self.obstacles)} obstacles")

def main(args=None):
    rclpy.init(args=args)
    
    node = BlimpObstacleVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()