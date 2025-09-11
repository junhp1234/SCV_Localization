#!/usr/bin/env python3

"""
Launch file for map origin transform node.
This node broadcasts map -> odom transform based on YAML configuration.
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
    
    # Define default parameters file path
    default_params_file = os.path.join(
        robot_localization_dir, 
        'params', 
        'map_origin_transform.yaml'
    )
    
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to parameters file for map origin transform node'
    )
    
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='map_origin_transform_node',
        description='Name of the map origin transform node'
    )
    
    output_arg = DeclareLaunchArgument(
        'output',
        default_value='screen',
        description='Output destination for node logs'
    )
    
    # Map origin transform node
    map_origin_transform_node = Node(
        package='robot_localization',
        executable='map_origin_transform_node',
        name=LaunchConfiguration('node_name'),
        parameters=[LaunchConfiguration('params_file')],
        output=LaunchConfiguration('output'),
        remappings=[
            # Add any topic remappings if needed
        ]
    )
    
    return LaunchDescription([
        params_file_arg,
        node_name_arg,
        output_arg,
        map_origin_transform_node
    ])