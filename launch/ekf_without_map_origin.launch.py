#!/usr/bin/env python3

"""
Launch file for EKF localization without map origin transform.
This provides standard robot_localization functionality without the map -> odom transform.
Use this when you want to handle map -> odom transform separately or don't need it.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
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
    
    use_map_ekf_arg = DeclareLaunchArgument(
        'use_map_ekf',
        default_value='true',
        description='Whether to launch EKF node for map frame'
    )
    
    use_navsat_transform_arg = DeclareLaunchArgument(
        'use_navsat_transform',
        default_value='true',
        description='Whether to launch navsat transform node'
    )
    
    output_arg = DeclareLaunchArgument(
        'output',
        default_value='screen',
        description='Output destination for node logs'
    )
    
    # EKF localization node (odom frame) - always launched
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
    
    # EKF localization node (map frame) - optional
    ekf_map_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        parameters=[LaunchConfiguration('ekf_params_file')],
        output=LaunchConfiguration('output'),
        remappings=[
            ('odometry/filtered', 'odometry/global')
        ],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('use_map_ekf'), "' == 'true'"
            ])
        )
    )
    
    # NavSat transform node for GPS integration - optional
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
        ],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('use_navsat_transform'), "' == 'true'"
            ])
        )
    )
    
    return LaunchDescription([
        ekf_params_file_arg,
        use_map_ekf_arg,
        use_navsat_transform_arg,
        output_arg,
        ekf_odom_node,
        ekf_map_node,
        navsat_transform_node
    ])