import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    blimp_core_pkg = get_package_share_directory('blimp_core')

    return LaunchDescription([
        # --- Arguments for easy tuning ---
        DeclareLaunchArgument('altitude_kp', default_value='150.0'),
        DeclareLaunchArgument('heading_kp', default_value='0.8'),
        DeclareLaunchArgument('forward_kp', default_value='20.0'),
        
        # --- Arguments for octomap collision checking ---
        DeclareLaunchArgument('check_radius', default_value='0.5',
            description='Neighborhood radius (m) around each point to check.'),
        DeclareLaunchArgument('check_step', default_value='0.05',
            description='Sampling step (m); use smaller for denser checks.'),
        DeclareLaunchArgument('check_use_sphere', default_value='true',
            description='If true, sample inside a sphere; else a cube.'),
        DeclareLaunchArgument('octomap_topic', default_value='/rtabmap/octomap_obstacles',
            description='OctoMap topic to feed the collision checker.'),

        # --- STAGE 1: OctoMap Collision Checker ---
        # This assumes localization is already running and publishing octomap
        Node(
            package='blimp_navigation', 
            executable='octomap_checker_node', 
            output='screen',
            name='octomap_checker',
            # No remapping needed - it subscribes to /rtabmap/octomap_obstacles directly
            parameters=[{
                'check_radius': LaunchConfiguration('check_radius'),
                'check_step': LaunchConfiguration('check_step'),
                'check_use_sphere': LaunchConfiguration('check_use_sphere')
            }]
        ),

        # --- STAGE 2: Launch The Brain (Navigator) ---
        Node(
            package='blimp_navigation',
            executable='blimp_navigator_node',
            name='blimp_navigator',
            output='screen',
            parameters=[{
                'altitude_kp': LaunchConfiguration('altitude_kp'),
                'heading_kp': LaunchConfiguration('heading_kp'),
                'forward_kp': LaunchConfiguration('forward_kp'),
            }]
        ),
    ])