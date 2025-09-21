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