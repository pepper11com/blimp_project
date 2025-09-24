import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    
    # RTAB-Map parameters - HYBRID MODE: Localization + Live Obstacles
    rtabmap_params = {
        'frame_id': 'camera_link',
        'subscribe_stereo': True,
        'subscribe_odom_info': True,
        'wait_imu_to_init': True,

        # === HYBRID LOCALIZATION + LIVE OBSTACLES ===
        'Mem/IncrementalMemory': 'True',               # Enable live updates!
        'Mem/InitWMWithAllNodes': 'True',              # Load existing map
        'Mem/LocalizationDataSaved': 'False',          # Don't save localization poses
        'Mem/ReduceGraph': 'True',                     # Limit map growth
        'Mem/RecentWmRatio': '0.1',                    # Keep only 10% recent nodes active
        'Mem/STMSize': '10',                           # Small short-term memory
        'RGBD/MaxLocalRetrieved': '5',                 # Limit local retrievals
        'database_path': LaunchConfiguration('database_path'),
        'RGBD/StartAtOrigin': 'true',

        'Grid/RangeMax': '0.0',
        'Stereo/MinDisparity': '0.05',
        'delete_db_on_start': False,

        # === REAL-TIME PERFORMANCE ===
        'Rtabmap/DetectionRate': '6.0',
        'Mem/ImagePreDecimation': '2',
        'Grid/3D': 'true',
        
        # === LIVE OBSTACLE GENERATION ===
        'Grid/RayTracing': 'true',                     # Generate live obstacles
        'Grid/3DGroundTruth': 'false',
        'Grid/MaxGroundAngle': '45',
        'Grid/NormalsSegmentation': 'false',
        
        'Grid/FootprintRadius': '0.3',
        'Grid/FootprintLength': '0.5',
        'Grid/FootprintWidth': '0.25',

        # Reduce publishing overhead
        'Rtabmap/PublishStats': "False",
        'Rtabmap/PublishLikelihood': "False",
        'Rtabmap/PublishPdf': "False",

        # === LIVE POINT CLOUD SETTINGS ===
        'Grid/CellSize': '0.15',                       # Good balance for live data
        'Grid/NoiseFilteringRadius': '0.05',          # Less filtering for responsiveness
        'Grid/NoiseFilteringMinNeighbors': '2',       
        'Grid/DepthDecimation': '1',                   # Less decimation for live obstacles
        'Grid/PreVoxelFiltering': 'true',
        
        'map_publish_to_tf': False,
    }

    def _check_db(context, *args, **kwargs):
        path = LaunchConfiguration('database_path').perform(context)
        if not os.path.isfile(path):
            raise RuntimeError(f'Database file not found: {path}. Set database_path to an existing rtabmap.db')
        return []

    remappings=[
        ('imu', '/imu/data'),
        ('left/image_rect', '/camera/infra1/image_rect_raw'),
        ('left/camera_info', '/camera/infra1/camera_info'),
        ('right/image_rect', '/camera/infra2/image_rect_raw'),
        ('right/camera_info', '/camera/infra2/camera_info')
    ]

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'unite_imu_method', default_value='2',
            description='0-None, 1-copy, 2-linear_interpolation.'),
        DeclareLaunchArgument(
            'database_path', default_value='/home/blimp2/.ros/rtabmap.db',
            description='Path to RTAB-Map database to localize against.'),
        DeclareLaunchArgument(
            'enable_stvl', default_value='true',
            description='Enable STVL for real-time obstacle visualization.'),
            
        # Fail fast if DB is missing
        OpaqueFunction(function=_check_db),

        # Viz throttling
        DeclareLaunchArgument('use_viz_throttle', default_value='true'),
        DeclareLaunchArgument('viz_throttle_rate_cloud', default_value='1.0'),
        DeclareLaunchArgument('viz_throttle_rate_grid', default_value='2.0'),

        # Hack to disable IR emitter
        SetParameter(name='depth_module.emitter_enabled', value=0),

        # Camera driver - Stereo only 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
            launch_arguments={
                'camera_namespace': '',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': LaunchConfiguration('unite_imu_method'),
                'enable_infra1': 'true',
                'enable_infra2': 'true',
                'enable_sync': 'true',
                'enable_color': 'false',
                'depth_fps': '15',
                'infra_fps': '15'
            }.items(),
        ),

        # Odometry
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=[rtabmap_params],
            remappings=remappings),

        # RTAB-Map in HYBRID mode (localization + live obstacles)
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_params],
            remappings=remappings
        ),

        # === LIGHTWEIGHT STVL FOR LIVE OBSTACLES ===
        Node(
            package='nav2_costmap_2d', 
            executable='nav2_costmap_2d',
            name='stvl_costmap',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_stvl')),
            parameters=[{
                # === BASIC COSTMAP SETTINGS ===
                'update_frequency': 8.0,       # Higher frequency for live data
                'publish_frequency': 4.0,      
                'global_frame': 'map',
                'robot_base_frame': 'camera_link',
                'rolling_window': True,
                
                # === SMALL COSTMAP FOR LIVE VISUALIZATION ===
                'width': 8,                    # 8m x 8m window
                'height': 8,
                'resolution': 0.1,             # 10cm resolution
                'track_unknown_space': False,
                
                # === STVL PLUGIN ===
                'plugins': ['stvl_layer'],
                
                'stvl_layer.plugin': 'spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer',
                'stvl_layer.enabled': True,
                
                # === TEMPORAL - Short for responsiveness ===
                'stvl_layer.voxel_decay': 2.0,           # Very short 2-second decay
                'stvl_layer.decay_model': 0,             
                'stvl_layer.observation_persistence': 0.0,
                
                # === SPATIAL SETTINGS ===
                'stvl_layer.voxel_size': 0.08,           # Fine 8cm voxels for detail
                'stvl_layer.mark_threshold': 0,
                'stvl_layer.obstacle_range': 4.0,        
                'stvl_layer.max_obstacle_height': 2.5,   
                'stvl_layer.origin_z': 0.0,
                
                # === PERFORMANCE ===
                'stvl_layer.track_unknown_space': False,
                'stvl_layer.publish_voxel_map': True,    # Main output!
                'stvl_layer.update_footprint_enabled': False,
                'stvl_layer.combination_method': 1,
                'stvl_layer.transform_tolerance': 0.2,
                'stvl_layer.mapping_mode': False,
                'stvl_layer.map_save_duration': 60.0,      # Fixed: integer type
                
                # === LIVE OBSTACLE SOURCE ===
                'stvl_layer.observation_sources': 'live_obstacles',
                
                # Use RTABMap's LIVE obstacle cloud 
                'stvl_layer.live_obstacles.enabled': True,
                'stvl_layer.live_obstacles.data_type': 'PointCloud2',
                'stvl_layer.live_obstacles.topic': '/cloud_obstacles',  # Should be live now!
                'stvl_layer.live_obstacles.marking': True,
                'stvl_layer.live_obstacles.clearing': False,
                'stvl_layer.live_obstacles.min_obstacle_height': 0.1,
                'stvl_layer.live_obstacles.max_obstacle_height': 2.5,
                'stvl_layer.live_obstacles.min_z': 0.1,
                'stvl_layer.live_obstacles.max_z': 4.0,
                'stvl_layer.live_obstacles.expected_update_rate': 6.0,
                'stvl_layer.live_obstacles.clear_after_reading': True,
                'stvl_layer.live_obstacles.filter': 'voxel',
                'stvl_layer.live_obstacles.voxel_min_points': 1,        # Accept single points
                'stvl_layer.live_obstacles.vertical_fov_angle': 0.98,
                'stvl_layer.live_obstacles.horizontal_fov_angle': 1.51,
                'stvl_layer.live_obstacles.decay_acceleration': 4.0,    # Fast decay
                'stvl_layer.live_obstacles.model_type': 0,
            }],
            remappings=[
                ('~/costmap_raw', '/stvl_costmap_raw'),
                ('~/costmap', '/stvl_costmap'),
                ('~/voxel_grid', '/stvl_voxel_grid'),  # Main STVL output!
            ]
        ),

        # === LIFECYCLE MANAGER FOR STVL ===
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='stvl_lifecycle_manager',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_stvl')),
            parameters=[{
                'autostart': True,
                'node_names': ['stvl_costmap']
            }]
        ),

        # Odom to Path 
        Node(
            package='blimp_core',
            executable='odom_to_path',
            output='screen'),

        # Throttle RTABMap topics
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

        # Throttle STVL visualization
        Node(
            package='topic_tools', executable='throttle', name='throttle_stvl',
            condition=IfCondition(LaunchConfiguration('enable_stvl')),
            arguments=['messages', '/stvl_voxel_grid', '3.0', '/viz/stvl_obstacles'],
            output='screen'),

        # IMU filter
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 'world_frame':'enu', 'publish_tf':False}],
            remappings=[('imu/data_raw', '/camera/imu')]),
    ])