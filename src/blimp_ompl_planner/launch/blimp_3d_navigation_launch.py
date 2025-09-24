#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package paths
    blimp_ompl_planner_share = FindPackageShare('blimp_ompl_planner')
    blimp_core_share = FindPackageShare('blimp_core')
    
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Blimp-specific parameters
    blimp_radius_arg = DeclareLaunchArgument(
        'blimp_radius',
        default_value='0.5',
        description='Radius of the blimp envelope for collision checking'
    )
    
    planning_bounds_config = PathJoinSubstitution([
        blimp_ompl_planner_share,
        'config',
        'blimp_planning_bounds.yaml'
    ])

    # Include the base blimp localization and SLAM launch
    blimp_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                blimp_core_share,
                'launch',
                'blimp_localization_slam_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # OMPL 3D Planner for Blimp
    ompl_planner_node = Node(
        package='blimp_ompl_planner',
        executable='ompl_planner_node',
        name='blimp_3d_planner',
        output='screen',
        parameters=[
            planning_bounds_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_radius': LaunchConfiguration('blimp_radius'),
                'planning_timeout': 15.0,  # Longer timeout for 3D planning
                'planner_id': 'RRTConnect',  # Fast for real-time use
                'global_frame': 'map',
                'robot_frame': 'camera_link',
                'enable_path_simplification': True,
                'enable_path_smoothing': True,
                'max_simplification_steps': 150
            }
        ],
        remappings=[
            ('octomap', '/octomap_binary'),
            ('planned_path', '/blimp/planned_path'),
            ('navigate_to_pose', '/blimp/navigate_to_pose')
        ]
    )

    # Path visualization node (optional, for debugging)
    path_visualizer = Node(
        package='blimp_ompl_planner',
        executable='ompl_planner_node',  # Could be a separate visualizer node
        name='path_visualizer',
        output='log',
        condition='false',  # Disabled by default
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        blimp_radius_arg,
        blimp_localization_launch,
        ompl_planner_node,
        # path_visualizer
    ])