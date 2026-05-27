# AI-Deep修改: 整合静态TF发布与深度处理节点的轻量化launch文件

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # AI-Deep修改: 通过IncludeLaunchDescription引入静态TF发布启动文件
    tools_launch_dir = get_package_share_directory('tools')
    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tools_launch_dir, 'launch', 'static_tf_multiple.launch.py')
        )
    )

    # AI-Deep修改: 深度处理节点
    depth_node = Node(
        package='depth_handler',
        executable='depth_processor_node',
        name='depth_handler_node',
        output='screen',
    )

    # AI-Deep修改: 返回LaunchDescription
    return LaunchDescription([
        static_tf_launch,
        depth_node,
    ])
