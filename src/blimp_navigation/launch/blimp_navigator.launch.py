#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('blimp_navigation')
    default_param_file = os.path.join(package_share, 'param', 'blimp_navigator.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock')

    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_param_file, description='Path to navigator parameter file')

    navigator_node = Node(
        package='blimp_navigation',
        executable='blimp_navigator_node',
        name='blimp_navigator_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_arg,
        navigator_node,
    ])
