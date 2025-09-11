#!/usr/bin/env python3

"""
Complete robot_localization launch file with optional map origin transform.
This provides all robot_localization functionality with configurable options.
Map -> odom transform is disabled by default but can be enabled via parameter.
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
    
    use_odom_ekf_arg = DeclareLaunchArgument(
        'use_odom_ekf',
        default_value='true',
        description='Whether to launch EKF node for odom frame'
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
    
    use_map_origin_transform_arg = DeclareLaunchArgument(
        'use_map_origin_transform',
        default_value='false',
        description='Whether to launch map origin transform node (map -> odom TF)'
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
        ],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('use_odom_ekf'), "' == 'true'"
            ])
        )
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
        ],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('use_map_ekf'), "' == 'true'"
            ])
        )
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
        ],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('use_navsat_transform'), "' == 'true'"
            ])
        )
    )
    
    # Map origin transform node (OPTIONAL - disabled by default)
    map_origin_transform_node = Node(
        package='robot_localization',
        executable='map_origin_transform_node',
        name='map_origin_transform_node',
        parameters=[LaunchConfiguration('map_origin_params_file')],
        output=LaunchConfiguration('output'),
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('use_map_origin_transform'), "' == 'true'"
            ])
        )
    )
    
    return LaunchDescription([
        ekf_params_file_arg,
        map_origin_params_file_arg,
        use_odom_ekf_arg,
        use_map_ekf_arg,
        use_navsat_transform_arg,
        use_map_origin_transform_arg,
        output_arg,
        ekf_odom_node,
        ekf_map_node,
        navsat_transform_node,
        map_origin_transform_node
    ])