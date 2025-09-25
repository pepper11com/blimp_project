#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ros2 launch blimp_ompl_planner ompl_planner_launch.py

def generate_launch_description():
    # Get path to config file
    config_file = PathJoinSubstitution([
        FindPackageShare('blimp_ompl_planner'),
        'config',
        'blimp_planning_bounds.yaml'
    ])
    
    # Declare launch arguments
    robot_radius_arg = DeclareLaunchArgument(
        'robot_radius',
        default_value='0.3',
        description='Radius of the robot for collision checking (meters)'
    )
    
    planning_timeout_arg = DeclareLaunchArgument(
        'planning_timeout',
        default_value='10.0',
        description='Maximum time allowed for planning (seconds)'
    )
    
    planner_id_arg = DeclareLaunchArgument(
        'planner_id',
        default_value='RRTConnect',
        description='OMPL planner algorithm to use (RRTConnect, RRTstar)'
    )
    
    global_frame_arg = DeclareLaunchArgument(
        'global_frame',
        default_value='map',
        description='Global reference frame for planning'
    )
    
    robot_frame_arg = DeclareLaunchArgument(
        'robot_frame',
        default_value='camera_link',
        description='Robot camera frame'
    )
    
    # Planning bounds arguments
    bounds_min_x_arg = DeclareLaunchArgument('bounds_min_x', default_value='-10.0')
    bounds_max_x_arg = DeclareLaunchArgument('bounds_max_x', default_value='10.0')
    bounds_min_y_arg = DeclareLaunchArgument('bounds_min_y', default_value='-10.0')
    bounds_max_y_arg = DeclareLaunchArgument('bounds_max_y', default_value='10.0')
    bounds_min_z_arg = DeclareLaunchArgument('bounds_min_z', default_value='-0.5')
    bounds_max_z_arg = DeclareLaunchArgument('bounds_max_z', default_value='5.0')
    
    # Path processing arguments
    enable_simplification_arg = DeclareLaunchArgument(
        'enable_path_simplification',
        default_value='true',
        description='Enable path shortcutting simplification'
    )
    
    enable_smoothing_arg = DeclareLaunchArgument(
        'enable_path_smoothing',
        default_value='true',
        description='Enable B-spline path smoothing'
    )

    # OMPL Planner Node
    ompl_planner_node = Node(
        package='blimp_ompl_planner',
        executable='ompl_planner_node',
        name='ompl_planner_node',
        output='screen',
        parameters=[
            config_file  # Load ALL parameters from YAML config file ONLY
        ],
        remappings=[
            ('octomap', '/octomap_binary'),
            ('planned_path', '/planned_path'),
            ('navigate_to_pose', '/navigate_to_pose')
        ]
    )

    # Goal Handler - Converts /goal_pose to navigation actions
    goal_handler = Node(
        package='blimp_ompl_planner',
        executable='rviz_goal_handler.py',
        name='goal_handler',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        robot_radius_arg,
        planning_timeout_arg,
        planner_id_arg,
        global_frame_arg,
        robot_frame_arg,
        bounds_min_x_arg,
        bounds_max_x_arg,
        bounds_min_y_arg,
        bounds_max_y_arg,
        bounds_min_z_arg,
        bounds_max_z_arg,
        enable_simplification_arg,
        enable_smoothing_arg,
        ompl_planner_node,
        goal_handler
    ])