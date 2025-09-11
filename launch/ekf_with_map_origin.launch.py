#!/usr/bin/env python3

"""
Launch file demonstrating integration of EKF localization with map origin transform.
This combines the standard EKF localization with the new map -> odom transform.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    robot_localization_dir = get_package_share_directory('robot_localization')
    
    # Define default parameters files
    default_ekf_params = os.path.join(
        robot_localization_dir, 
        'params', 
        'dual_ekf_navsat_example.yaml'
    )
    
    default_map_origin_params = os.path.join(
        robot_localization_dir, 
        'params', 
        'map_origin_transform.yaml'
    )
    
    # Declare launch arguments
    ekf_params_file_arg = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=default_ekf_params,
        description='Full path to EKF parameters file'
    )
    
    map_origin_params_file_arg = DeclareLaunchArgument(
        'map_origin_params_file',
        default_value=default_map_origin_params,
        description='Full path to map origin transform parameters file'
    )
    
    output_arg = DeclareLaunchArgument(
        'output',
        default_value='screen',
        description='Output destination for node logs'
    )
    
    # EKF localization node (odom frame)
    ekf_odom_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        parameters=[LaunchConfiguration('ekf_params_file')],
        output=LaunchConfiguration('output'),
        remappings=[
            ('odometry/filtered', 'odometry/local')
        ]
    )
    
    # EKF localization node (map frame) 
    ekf_map_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        parameters=[LaunchConfiguration('ekf_params_file')],
        output=LaunchConfiguration('output'),
        remappings=[
            ('odometry/filtered', 'odometry/global')
        ]
    )
    
    # NavSat transform node for GPS integration
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        parameters=[LaunchConfiguration('ekf_params_file')],
        output=LaunchConfiguration('output'),
        remappings=[
            ('imu/data', 'imu/data'),
            ('gps/fix', 'gps/fix'),
            ('gps/filtered', 'gps/filtered'),
            ('odometry/gps', 'odometry/gps'),
            ('odometry/filtered', 'odometry/global')
        ]
    )
    
    # Map origin transform node
    map_origin_transform_node = Node(
        package='robot_localization',
        executable='map_origin_transform_node',
        name='map_origin_transform_node',
        parameters=[LaunchConfiguration('map_origin_params_file')],
        output=LaunchConfiguration('output')
    )
    
    return LaunchDescription([
        ekf_params_file_arg,
        map_origin_params_file_arg,
        output_arg,
        ekf_odom_node,
        ekf_map_node,
        navsat_transform_node,
        map_origin_transform_node
    ])