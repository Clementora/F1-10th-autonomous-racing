import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_name = 'control_localisation_pkg'
    config_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        #  vehicle controller node
        Node(
            package=pkg_name,
            executable='control_node',
            name='gap_follower_controller',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        #  custom localiser node
        Node(
            package=pkg_name,
            executable='localiser_node', 
            name='localiser',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        #  robot_localization EKF node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file_path, {'use_sim_time': False}]
        )
    ])