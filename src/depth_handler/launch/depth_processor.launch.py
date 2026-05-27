# AI-Deep修改: 整合相机、机械臂控制、目标检测、静态TF、深度处理为一体化的launch文件

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 设备权限设置
    password = '123'
    subprocess.run(
        ['sudo', '-S', 'chmod', '777', '/dev/ttyACM0'],
        input=password + '\n', encoding='utf-8'
    )

    # AI-Deep修改: 声明所有可配置的启动参数
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value='192.168.58.2',
        description='机器人控制器IP地址'
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='L',
        description='机器人名称 (L/R)'
    )
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.5',
        description='目标检测置信度阈值'
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/mihu/FR3_again/src/detector/best2.engine',
        description='YOLO模型路径'
    )

    # AI-Deep修改: 通过IncludeLaunchDescription引入Orbbec相机启动文件
    orbbec_launch_dir = get_package_share_directory('orbbec_camera')
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_launch_dir, 'launch', 'gemini_330_series.launch.py')
        )
    )

    # AI-Deep修改: 通过IncludeLaunchDescription引入静态TF发布启动文件
    tools_launch_dir = get_package_share_directory('tools')
    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tools_launch_dir, 'launch', 'static_tf_multiple.launch.py')
        )
    )

    # AI-Deep修改: 机械臂控制节点，robot_ip和robot_name可通过launch参数配置
    robo_ctrl_node = Node(
        package='robo_ctrl',
        executable='robo_ctrl_node',
        name=[LaunchConfiguration('robot_name'), 'robo_ctrl'],
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'robot_name': LaunchConfiguration('robot_name'),
        }],
        output='screen',
    )

    # AI-Deep修改: 目标检测节点，支持confidence_threshold和model_path可配置
    detector_node = Node(
        package='detector',
        executable='detector_node_exe',
        name='detector_node',
        parameters=[{
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'model_path': LaunchConfiguration('model_path'),
        }],
        output='screen',
    )

    # AI-Deep修改: 深度处理节点
    depth_node = Node(
        package='depth_handler',
        executable='depth_processor_node',
        name='depth_handler_node',
        output='screen',
    )

    # AI-Deep修改: 返回完整的LaunchDescription，包含所有节点和参数声明
    return LaunchDescription([
        robot_ip_arg,
        robot_name_arg,
        confidence_threshold_arg,
        model_path_arg,
        camera_launch,
        static_tf_launch,
        robo_ctrl_node,
        detector_node,
        depth_node,
    ])
