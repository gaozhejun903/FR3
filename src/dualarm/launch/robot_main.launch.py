#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('dualarm')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'config.yaml'),
        description='Path to the config file'
    )
    
    # Robot main node
    robot_main_node = Node(
        package='dualarm',
        executable='robot_main',
        name='robot_main',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        config_file_arg,
        robot_main_node
    ])
