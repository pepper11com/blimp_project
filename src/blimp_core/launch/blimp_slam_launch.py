# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node, SetParameter
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():

#     # Get the path to your YAML configuration file
#     config_file_path = PathJoinSubstitution([
#         get_package_share_directory('blimp_core'), 'config', 'rtabmap_mono_lidar.yaml'
#     ])

#     # Shared remappings for stereo topics
#     stereo_remappings = [
#           ('imu', '/imu/data'),
#           ('left/image_rect', '/camera/infra1/image_rect_raw'),
#           ('left/camera_info', '/camera/infra1/camera_info'),
#           ('right/image_rect', '/camera/infra2/image_rect_raw'),
#           ('right/camera_info', '/camera/infra2/camera_info')]
          
#     # SLAM node also needs the LiDAR scan and odometry
#     slam_remappings = stereo_remappings + [('scan', '/scan'), ('odom', '/odom')]

#     return LaunchDescription([
#         # Declare launch arguments
#         DeclareLaunchArgument('use_sim_time', default_value='true'),
#         DeclareLaunchArgument('unite_imu_method', default_value='2'),

#         # Static transforms
#         Node(package='tf2_ros', executable='static_transform_publisher', name='static_lidar_tf',
#              output='screen', arguments=['-0.15', '0', '0.1', '0', '0', '1', '0', 'base_link', 'lidar_link']),
#         Node(package='tf2_ros', executable='static_transform_publisher', name='static_camera_tf',
#              output='screen', arguments=['0.15', '0', '0.12', '0', '0', '0', '1', 'base_link', 'camera_link']),

#         # Camera driver
#         SetParameter(name='depth_module.emitter_enabled', value=0),
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([os.path.join(
#                 get_package_share_directory('realsense2_camera'), 'launch'),
#                 '/rs_launch.py']),
#             launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'), 'unite_imu_method': LaunchConfiguration('unite_imu_method'), 'enable_infra1': 'true', 'enable_infra2': 'true', 'enable_gyro': 'true', 'enable_accel': 'true', 'enable_sync': 'true'}.items(),
#         ),

#         # IMU filter
       

#         # Stereo Odometry Node
#         Node(
#             package='rtabmap_odom', executable='stereo_odometry', output='screen',
#             name='stereo_odometry',
#             parameters=[config_file_path],
#             remappings=stereo_remappings),

#         # RTAB-Map SLAM Node
#         Node(
#             package='rtabmap_slam', executable='rtabmap', output='screen',
#             name='rtabmap',
#             parameters=[config_file_path],
#             remappings=slam_remappings,
#             arguments=['-d']),
#     ])
    
    
# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch rtabmap_examples realsense_d435i_stereo.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    parameters=[{
        'frame_id':'camera_link',
        'subscribe_stereo':True,
        'subscribe_odom_info':True,
        'wait_imu_to_init':True,
        
        # === SLAM PERFORMANCE PARAMETERS ===
        'Rtabmap/DetectionRate': '6.0',
        'Mem/ImagePreDecimation': '2',

        # Publish less metadata to lighten transport/visualization
        'Rtabmap/PublishStats': "False",
        'Rtabmap/PublishLikelihood': "False",
        'Rtabmap/PublishPdf': "False",

        'Grid/RangeMax': '0.0',        # infinite range max for depth image projection
        'Stereo/MinDisparity': '0.05',

        # === POINT CLOUD DENSITY PARAMETERS ===
        # These have the biggest impact on the size of cloud_map
        'Grid/CellSize': '0.06',             # Voxel grid filter size (m), LARGER = LESS DENSE!!
        'Grid/NoiseFilteringRadius': '0.06', # Radius for noise filtering
        'Grid/NoiseFilteringMinNeighbors': '4', # Min neighbors for noise filtering

        # Further reduce outgoing map size without hurting SLAM accuracy
        'Grid/DepthDecimation': '4',        # Increase decimation before cloud creation
        'GridGlobal/MaxNodes': '150',       # Limit nodes assembled in cloud_map/grid_map
    }]

    remappings=[
          ('imu', '/imu/data'),
          ('left/image_rect', '/camera/infra1/image_rect_raw'),
          ('left/camera_info', '/camera/infra1/camera_info'),
          ('right/image_rect', '/camera/infra2/image_rect_raw'),
          ('right/camera_info', '/camera/infra2/camera_info')]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'unite_imu_method', default_value='2',
            description='0-None, 1-copy, 2-linear_interpolation. Use unite_imu_method:="1" if imu topics stop being published.'),
        # Add controls for visualization throttling
        DeclareLaunchArgument('use_viz_throttle', default_value='true'),
        DeclareLaunchArgument('viz_throttle_rate_cloud', default_value='1.0'),
        DeclareLaunchArgument('viz_throttle_rate_grid', default_value='2.0'),

        # Hack to disable IR emitter
        SetParameter(name='depth_module.emitter_enabled', value=0),

        # Launch camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
                launch_arguments={'camera_namespace': '',
                                  'enable_gyro': 'true',
                                  'enable_accel': 'true',
                                  'unite_imu_method': LaunchConfiguration('unite_imu_method'),
                                  'enable_infra1': 'true',
                                  'enable_infra2': 'true',
                                  'enable_sync': 'true',
                                  'enable_depth': 'false'
                                  }.items(),
        ),

        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),
        
        # Throttle only what Foxglove subscribes to (subscribe to /viz/* in Foxglove)
        # Requires: sudo apt install ros-$ROS_DISTRO-topic-tools
        Node(
            package='topic_tools', executable='throttle', name='throttle_cloud_map',
            condition=IfCondition(LaunchConfiguration('use_viz_throttle')),
            arguments=['messages', '/rtabmap/cloud_map', LaunchConfiguration('viz_throttle_rate_cloud'), '/viz/cloud_map'],
            output='screen'),
        Node(
            package='topic_tools', executable='throttle', name='throttle_grid_map',
            condition=IfCondition(LaunchConfiguration('use_viz_throttle')),
            arguments=['messages', '/rtabmap/grid_map', LaunchConfiguration('viz_throttle_rate_grid'), '/viz/grid_map'],
            output='screen'),
                
        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/camera/imu')]),
    ])