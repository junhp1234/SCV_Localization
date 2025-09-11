#!/usr/bin/env python3

"""
Basic EKF launch file without map origin transform.
This provides minimal robot_localization functionality with just the odom-frame EKF.
Use this for simple odometry fusion without GPS or map frame integration.
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
    
    # Define default parameters file
    default_ekf_params = os.path.join(
        robot_localization_dir, 
        'params', 
        'dual_ekf_navsat_example.yaml'
    )
    
    # Declare launch arguments
    ekf_params_file_arg = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=default_ekf_params,
        description='Full path to EKF parameters file'
    )
    
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='ekf_filter_node',
        description='Name of the EKF node'
    )
    
    output_arg = DeclareLaunchArgument(
        'output',
        default_value='screen',
        description='Output destination for node logs'
    )
    
    # Basic EKF localization node (odom frame only)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name=LaunchConfiguration('node_name'),
        parameters=[LaunchConfiguration('ekf_params_file')],
        output=LaunchConfiguration('output'),
        remappings=[
            # Standard remappings for odometry fusion
            ('odometry/filtered', 'odometry/filtered'),
            ('imu/data', 'imu/data'),
            ('odom', 'odom')
        ]
    )
    
    return LaunchDescription([
        ekf_params_file_arg,
        node_name_arg,
        output_arg,
        ekf_node
    ])