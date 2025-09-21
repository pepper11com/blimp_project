# import os

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch_ros.actions import Node, SetParameter
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch.conditions import IfCondition

# def generate_launch_description():

#     # =================================================================
#     # == PARAMETERS FOR HIGH-PERFORMANCE LOCALIZATION ON RASPBERRY PI 5 ==
#     # =================================================================

#     # Parameters for the high-frequency stereo_odometry node.
#     # These are tuned for speed to ensure smooth, real-time motion tracking.
#     odom_params = {
#         # --- Core odometry settings ---
#         'frame_id': 'camera_link',
#         'subscribe_stereo': True,
#         'subscribe_odom_info': True,
#         'wait_imu_to_init': True,
        
#         # --- Performance optimizations for Pi 5 ---
#         'Odom/Strategy': '1',               # 0=F2M, 1=F2F. Frame-to-Frame is faster.
#         'Vis/CorType': '1',                 # 0=Features, 1=Optical Flow. Optical flow is faster.
#         # IMPORTANT: The feature type used here MUST match the one used to create the map.
#         'Vis/MaxFeatures': '400',           # Reduce the number of features to track to lower CPU load.
#         'Mem/ImagePreDecimation': '2',      # Halve image resolution before processing, a major speed-up.
#     }

#     # Parameters for the rtabmap localization node.
#     # These are tuned to make global localization checks fast and efficient.
#     rtabmap_params = {
#         # --- Core localization settings ---
#         # CRITICAL FIX: frame_id must also be 'camera_link' for consistency.
#         'frame_id': 'camera_link',
#         'subscribe_stereo': True,
#         'subscribe_odom_info': True,
#         'Mem/IncrementalMemory': 'False',       # Must be False for localization mode.
#         'Mem/InitWMWithAllNodes': 'True',       # Load the full map into memory.
#         'database_path': LaunchConfiguration('database_path'),
#         'delete_db_on_start': False,

#         # --- Performance optimizations for Pi 5 ---
#         'Rtabmap/DetectionRate': '4.0',         # High rate for fast localization. Lower to 2.0 if CPU is still too high.
#         # IMPORTANT: This MUST match the feature type used to create the map.
        
#         # FIX: Ensure Vis/FeatureType is also set here to match Kp/DetectorStrategy and odom_params.
#         # This re-enables the 'Mem/UseOdomFeatures' optimization.
#         'RGBD/ProximityBySpace': 'false',       # Disable local proximity checks to save CPU.
#         'Mem/ImagePreDecimation': '2',          # Also pre-decimate images for localization checks.

#         # --- Disable unnecessary publishers ---
#         'Rtabmap/PublishStats': "False",
#         'Rtabmap/PublishLikelihood': "False",
#         'Rtabmap/PublishPdf': "False",
        
#         # --- Other kept parameters ---
#         'Grid/CellSize': '0.1',
#         'Grid/NoiseFilteringRadius': '0.1',
#         'Grid/NoiseFilteringMinNeighbors': '4',
#         'Grid/DepthDecimation': '4',
#     }

#     # Validate DB exists before starting rtabmap
#     def _check_db(context, *args, **kwargs):
#         path = LaunchConfiguration('database_path').perform(context)
#         if not os.path.isfile(path):
#             raise RuntimeError(f'Database file not found: {path}. Set database_path to an existing rtabmap.db.')
#         return

#     remappings=[
#         ('imu', '/imu/data'),
#         ('left/image_rect', '/camera/infra1/image_rect_raw'),
#         ('left/camera_info', '/camera/infra1/camera_info'),
#         ('right/image_rect', '/camera/infra2/image_rect_raw'),
#         ('right/camera_info', '/camera/infra2/camera_info')
#     ]

#     return LaunchDescription([

#         DeclareLaunchArgument(
#             'unite_imu_method', default_value='2',
#             description='0-None, 1-copy, 2-linear_interpolation.'),
#         DeclareLaunchArgument(
#             'database_path', default_value='/home/blimp/.ros/rtabmap.db',
#             description='Path to RTAB-Map database to localize against.'),
#         DeclareLaunchArgument(
#             'start_at_origin', default_value='true',
#             description='Start at map origin when localizing (localization mode only).'),

#         # Fail fast if DB is missing
#         OpaqueFunction(function=_check_db),

#         # Viz throttling
#         DeclareLaunchArgument('use_viz_throttle', default_value='true'),
#         DeclareLaunchArgument('viz_throttle_rate_cloud', default_value='1.0'),
#         DeclareLaunchArgument('viz_throttle_rate_grid', default_value='2.0'),

#         # Hack to disable IR emitter
#         SetParameter(name='depth_module.emitter_enabled', value=0),

#         # Camera driver
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([os.path.join(
#                 get_package_share_directory('realsense2_camera'), 'launch'),
#                 '/rs_launch.py']),
#             launch_arguments={
#                 'camera_namespace': '',
#                 'enable_gyro': 'true',
#                 'enable_accel': 'true',
#                 'unite_imu_method': LaunchConfiguration('unite_imu_method'),
#                 'enable_infra1': 'true',
#                 'enable_infra2': 'true',
#                 'enable_sync': 'true'
#             }.items(),
#         ),

#         # Odometry Node (Optimized for Pi 5)
#         Node(
#             package='rtabmap_odom', executable='stereo_odometry', output='screen',
#             name='stereo_odometry',
#             parameters=[odom_params],
#             remappings=remappings),

#         # RTAB-Map Localization Node (Optimized for Pi 5)
#         Node(
#             package='rtabmap_slam', executable='rtabmap', output='screen',
#             name='rtabmap',
#             parameters=[rtabmap_params],
#             remappings=remappings
#         ),

#         # Odom to Path Node (for RViz visualization)
#         Node(
#             package='blimp_core', # Make sure this is your package name
#             executable='odom_to_path', # Make sure this is your script name
#             name='odom_to_path',
#             output='screen'),

#         # Throttle topics for visualization
#         Node(
#             package='topic_tools', executable='throttle', name='throttle_cloud_map',
#             condition=IfCondition(LaunchConfiguration('use_viz_throttle')),
#             arguments=['messages', '/rtabmap/cloud_map', LaunchConfiguration('viz_throttle_rate_cloud'), '/viz/cloud_map'],
#             output='screen'),
#         Node(
#             package='topic_tools', executable='throttle', name='throttle_grid_map',
#             condition=IfCondition(LaunchConfiguration('use_viz_throttle')),
#             arguments=['messages', '/rtabmap/grid_map', LaunchConfiguration('viz_throttle_rate_grid'), '/viz/grid_map'],
#             output='screen'),

#         # IMU filter
#         Node(
#             package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
#             parameters=[{'use_mag': False, 'world_frame':'enu', 'publish_tf':False}],
#             remappings=[('imu/data_raw', '/camera/imu')]),
#     ])

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    parameters=[{
        'frame_id':'camera_link',
        'subscribe_stereo':True,
        'subscribe_odom_info':True,
        'wait_imu_to_init':True,

        # === Localization mode ===
        'Mem/IncrementalMemory': 'False',      # Localization mode
        'Mem/InitWMWithAllNodes': 'True',      # Load all nodes from DB into WM
        'Mem/LocalizationDataSaved': 'False',  # Do not save data in localization
        'database_path': LaunchConfiguration('database_path'),
        'RGBD/StartAtOrigin': 'true',

        # Make sure rtabmap can write temp files/logs
        # 'Rtabmap/WorkingDirectory': '/home/blimp/.ros',

        # Wrapper safety: never delete DB on start
        'delete_db_on_start': False,

        # === SLAM PERFORMANCE PARAMETERS (kept) ===
        'Rtabmap/DetectionRate': '6.0',
        'Mem/ImagePreDecimation': '2',

        # Enable 3D occupancy grid (OctoMap) publication from RTAB-Map
        'Grid/3D': 'true',

        # Publish less metadata to lighten transport/visualization
        'Rtabmap/PublishStats': "False",
        'Rtabmap/PublishLikelihood': "False",
        'Rtabmap/PublishPdf': "False",

        # === POINT CLOUD DENSITY PARAMETERS (kept) ===
        'Grid/CellSize': '0.1',
        'Grid/NoiseFilteringRadius': '0.1',
        'Grid/NoiseFilteringMinNeighbors': '4',

        # Load and assemble all nodes (no cap)
        'Grid/DepthDecimation': '4',
        # 'GridGlobal/MaxNodes': '0',
        
        'map_publish_to_tf': False, # This often helps trigger the publisher
    }]

    # Validate DB exists before starting rtabmap
    def _check_db(context, *args, **kwargs):
        path = LaunchConfiguration('database_path').perform(context)
        if not os.path.isfile(path):
            raise RuntimeError(f'Database file not found: {path}. Set database_path to an existing rtabmap.db.')
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
            'start_at_origin', default_value='true',
            description='Start at map origin when localizing (localization mode only).'),
            # Default neighborhood-check parameters for the octomap checker
            DeclareLaunchArgument('check_radius', default_value='0.5',
                description='Neighborhood radius (m) around each point to check.'),
            DeclareLaunchArgument('check_step', default_value='0.05',
                description='Sampling step (m); use smaller for denser checks.'),
            DeclareLaunchArgument('check_use_sphere', default_value='true',
                description='If true, sample inside a sphere; else a cube.'),
        # Toggle launching octomap_server (disabled by default; RTAB-Map already publishes OctoMap)
        DeclareLaunchArgument(
            'use_octomap_server', default_value='false',
            description='Launch octomap_server_node (off by default since RTAB-Map publishes OctoMap).'),
        # Topic used by octomap checker to listen for OctoMap messages.
        # If RTAB-Map publishes a different topic (e.g., /rtabmap/octomap), set it via this arg.
        DeclareLaunchArgument(
            'octomap_topic', default_value='/octomap_binary',
            description='OctoMap topic to feed the collision checker.'),

        # Fail fast if DB is missing
        OpaqueFunction(function=_check_db),

        # Viz throttling
        DeclareLaunchArgument('use_viz_throttle', default_value='true'),
        DeclareLaunchArgument('viz_throttle_rate_cloud', default_value='1.0'),
        DeclareLaunchArgument('viz_throttle_rate_grid', default_value='2.0'),

        # Hack to disable IR emitter
        SetParameter(name='depth_module.emitter_enabled', value=0),

        # Camera driver
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
                'enable_sync': 'true'
            }.items(),
        ),

        # Odometry
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        # RTAB-Map in localization mode (loads DB)
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings
            # No -d here: we do not delete DB on start
        ),

        # 3D OctoMap collision check service
        Node(
            package='blimp_navigation', executable='octomap_checker_node', output='screen',
            name='octomap_checker',
            remappings=[
                ('/rtabmap/octomap_obstacles', LaunchConfiguration('octomap_topic'))
                ],
                parameters=[{
                    'check_radius': LaunchConfiguration('check_radius'),
                    'check_step': LaunchConfiguration('check_step'),
                    'check_use_sphere': LaunchConfiguration('check_use_sphere')
                }]
        ),
        
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_octomap_server')),
            parameters=[{
                'frame_id': 'map',     # Global frame
                'resolution': 0.1,     # Match RTAB-Map Grid/CellSize if comparing
                'output_color': False
            }]
        ),

        # Odom to Path
        Node(
            package='blimp_core',
            executable='odom_to_path',
            output='screen'),

        # Throttle Foxglove topics (subscribe to /viz/* in Foxglove)
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

        # IMU filter
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 'world_frame':'enu', 'publish_tf':False}],
            remappings=[('imu/data_raw', '/camera/imu')]),
    ])
