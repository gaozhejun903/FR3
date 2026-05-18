#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    save_path_arg = DeclareLaunchArgument(
        'save_path',
        default_value='/tmp/dual_end_tf_data',
        description='Path to save dual end TF data'
    )
    
    auto_capture_arg = DeclareLaunchArgument(
        'auto_capture',
        default_value='false',
        description='Enable auto capture mode'
    )
    
    auto_capture_interval_arg = DeclareLaunchArgument(
        'auto_capture_interval',
        default_value='2000',
        description='Auto capture interval in milliseconds'
    )
    
    tf_update_interval_arg = DeclareLaunchArgument(
        'tf_update_interval',
        default_value='100',
        description='TF update interval in milliseconds'
    )
    
    rend_frame_id_arg = DeclareLaunchArgument(
        'rend_frame_id',
        default_value='Rend',
        description='Right end frame ID'
    )
    
    lend_frame_id_arg = DeclareLaunchArgument(
        'lend_frame_id',
        default_value='Lend',
        description='Left end frame ID'
    )
    
    base_frame_id_arg = DeclareLaunchArgument(
        'base_frame_id',
        default_value='world',
        description='Base frame ID'
    )
    
    rend_base_frame_id_arg = DeclareLaunchArgument(
        'rend_base_frame_id',
        default_value='Rrobot_base',
        description='Right arm base frame ID'
    )
    
    lend_base_frame_id_arg = DeclareLaunchArgument(
        'lend_base_frame_id',
        default_value='Lrobot_base',
        description='Left arm base frame ID'
    )
    
    # Dual End TF Collector Node
    dual_end_tf_collector_node = Node(
        package='tools',
        executable='dual_end_tf_collector_node',
        name='dual_end_tf_collector',
        output='screen',
        parameters=[{
            'save_path': LaunchConfiguration('save_path'),
            'auto_capture': LaunchConfiguration('auto_capture'),
            'auto_capture_interval': LaunchConfiguration('auto_capture_interval'),
            'tf_update_interval': LaunchConfiguration('tf_update_interval'),
            'rend_frame_id': LaunchConfiguration('rend_frame_id'),
            'lend_frame_id': LaunchConfiguration('lend_frame_id'),
            'base_frame_id': LaunchConfiguration('base_frame_id'),
            'rend_base_frame_id': LaunchConfiguration('rend_base_frame_id'),
            'lend_base_frame_id': LaunchConfiguration('lend_base_frame_id'),
        }],
        remappings=[
            # Add any necessary topic remappings here
        ]
    )
    
    return LaunchDescription([
        save_path_arg,
        auto_capture_arg,
        auto_capture_interval_arg,
        tf_update_interval_arg,
        rend_frame_id_arg,
        lend_frame_id_arg,
        base_frame_id_arg,
        rend_base_frame_id_arg,
        lend_base_frame_id_arg,
        dual_end_tf_collector_node,
    ])
