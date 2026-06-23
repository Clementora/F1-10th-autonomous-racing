import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_name = 'control_localisation_pkg'
    config_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'ekf.yaml'
    )


    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='Set to "true" to fuse IMU via EKF, or "false" for pure wheel odometry.'
    )
    

    use_imu = LaunchConfiguration('use_imu')

    return LaunchDescription([
       
        use_imu_arg,

        # vehicle controller node (Runs no matter what)
        Node(
            package=pkg_name,
            executable='control_node',
            name='gap_follower_controller',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # custom localiser node (WHEN IMU IS TRUE)
        # Publishes to /localisation so the EKF can use it.
        Node(
            package=pkg_name,
            executable='localiser_node', 
            name='localiser',
            output='screen',
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(use_imu)
        ),

        # custom localiser node (WHEN IMU IS FALSE)
        # Bypasses the EKF by renaming its output directly to /odometry/filtered.
        Node(
            package=pkg_name,
            executable='localiser_node', 
            name='localiser',
            output='screen',
            parameters=[{'use_sim_time': False}],
            remappings=[('/localisation', '/odometry/filtered')],
            condition=UnlessCondition(use_imu)
        ),

        # robot_localization EKF node
        # Only launches if IMU is true
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file_path, {'use_sim_time': False}],
            condition=IfCondition(use_imu)
        )
    ])