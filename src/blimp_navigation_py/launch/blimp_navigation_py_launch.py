import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

# ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{ header: { frame_id: 'map'}, pose: { position: { x: 0.818, y: 0.318,  z: 0.145 }, orientation: { x: 0.016, y: 0.069, z: 0.214,   w: 0.974 } }  }" --once
# ros2 launch blimp_core blimp_localization_launch.py
# ros2 launch blimp_navigation_py blimp_navigation_py_launch.py
# ros2 topic echo /navigation_debug
# /home/blimp2/yamspy_env/bin/python3 /home/blimp2/yamspy_env/bin/blimp_test.py

def generate_launch_description():

    blimp_core_pkg = get_package_share_directory('blimp_core')
    
    # Yamspy environment to Python path
    yamspy_env_path = '/home/blimp2/yamspy_env/lib/python3.12/site-packages'
    current_python_path = os.environ.get('PYTHONPATH', '')
    if current_python_path:
        new_python_path = f"{yamspy_env_path}:{current_python_path}"
    else:
        new_python_path = yamspy_env_path

    return LaunchDescription([
       
        DeclareLaunchArgument('altitude_kp', default_value='150.0'),
        DeclareLaunchArgument('heading_kp', default_value='0.8'),
        DeclareLaunchArgument('forward_kp', default_value='20.0'),
        
        # --- (Navigator) ---
        Node(
            package='blimp_navigation_py',
            executable='blimp_navigator',
            name='blimp_navigator',
            output='screen',
            additional_env={'PYTHONPATH': new_python_path},
            parameters=[{
                'altitude_kp': LaunchConfiguration('altitude_kp'),
                'altitude_ki': 10.0,
                'altitude_kd': 20.0,
                'heading_kp': LaunchConfiguration('heading_kp'),
                'heading_ki': 0.1,
                'heading_kd': 0.2,
                'forward_kp': LaunchConfiguration('forward_kp'),
                'forward_ki': 2.0,
                'forward_kd': 5.0,
            }]
        ),
    ])