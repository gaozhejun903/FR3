# AI-Deep: 一体化launch文件
# 包含：左臂+右臂控制、左右夹爪、相机、目标检测、静态TF、深度处理

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # AI-Deep: 夹爪设备权限
    password = '123'
    for dev in ['/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2']:
        if os.path.exists(dev):
            subprocess.run(
                ['sudo', '-S', 'chmod', '777', dev],
                input=password + '\n', encoding='utf-8',
                capture_output=True
            )

    # ═══════════════════════════════════════════════════════════════
    # 左臂参数 (192.168.58.2)
    # ═══════════════════════════════════════════════════════════════
    L_ip_arg = DeclareLaunchArgument(
        'L_robot_ip', default_value='192.168.58.2',
        description='左臂控制器IP地址'
    )
    # ═══════════════════════════════════════════════════════════════
    # 右臂参数 (192.168.58.3)
    # ═══════════════════════════════════════════════════════════════
    R_ip_arg = DeclareLaunchArgument(
        'R_robot_ip', default_value='192.168.58.3',
        description='右臂控制器IP地址'
    )
    # ═══════════════════════════════════════════════════════════════
    # 夹爪端口参数
    # ═══════════════════════════════════════════════════════════════
    # 夹爪端口参数 (by-path 持久化路径，重启不变)
    # ═══════════════════════════════════════════════════════════════
    R_gripper_port_arg = DeclareLaunchArgument(
        'R_gripper_port',
        default_value='/dev/serial/by-path/pci-0000:05:00.4-usb-0:1.2:1.0-port0',
        description='右夹爪串口 (by-path)'
    )
    L_gripper_port_arg = DeclareLaunchArgument(
        'L_gripper_port',
        default_value='/dev/serial/by-path/pci-0000:05:00.4-usb-0:1.4:1.0-port0',
        description='左夹爪串口 (by-path)'
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

    # ═══════════════════════════════════════════════════════════════
    # 相机 + 静态TF
    # ═══════════════════════════════════════════════════════════════
    orbbec_launch_dir = get_package_share_directory('orbbec_camera')
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_launch_dir, 'launch', 'gemini_330_series.launch.py')
        )
    )

    tools_launch_dir = get_package_share_directory('tools')
    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tools_launch_dir, 'launch', 'static_tf_multiple.launch.py')
        )
    )

    # ═══════════════════════════════════════════════════════════════
    # 左臂 (L) 控制节点
    # ═══════════════════════════════════════════════════════════════
    L_robo_ctrl_node = Node(
        package='robo_ctrl',
        executable='robo_ctrl_node',
        name='Lrobo_ctrl',
        parameters=[{
            'robot_ip': LaunchConfiguration('L_robot_ip'),
            'robot_name': 'L',
        }],
        output='screen',
    )
    L_high_level_node = Node(
        package='robo_ctrl',
        executable='high_level_node',
        name='Lhigh_level',
        parameters=[{
            'robot_ip': LaunchConfiguration('L_robot_ip'),
            'robot_name': 'L',
        }],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    # 右臂 (R) 控制节点
    # ═══════════════════════════════════════════════════════════════
    R_robo_ctrl_node = Node(
        package='robo_ctrl',
        executable='robo_ctrl_node',
        name='Rrobo_ctrl',
        parameters=[{
            'robot_ip': LaunchConfiguration('R_robot_ip'),
            'robot_name': 'R',
        }],
        output='screen',
    )
    R_high_level_node = Node(
        package='robo_ctrl',
        executable='high_level_node',
        name='Rhigh_level',
        parameters=[{
            'robot_ip': LaunchConfiguration('R_robot_ip'),
            'robot_name': 'R',
        }],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    # 左夹爪节点
    # ═══════════════════════════════════════════════════════════════
    L_gripper_node = Node(
        package='epg50_gripper_ros',
        executable='epg50_gripper_node',
        name='L_gripper_node',
        parameters=[{
            'port': LaunchConfiguration('L_gripper_port'),
        }],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    # 右夹爪节点
    # ═══════════════════════════════════════════════════════════════
    R_gripper_node = Node(
        package='epg50_gripper_ros',
        executable='epg50_gripper_node',
        name='R_gripper_node',
        parameters=[{
            'port': LaunchConfiguration('R_gripper_port'),
        }],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    # AI-Deep: 虚假夹爪TF发布节点 — 为high_level轨迹规划提供水平参考系
    # 创建 Lfake_gripper_frame: 位置=夹爪尖端, Z轴=世界朝上, X轴=夹爪指向(水平投影)
    # ═══════════════════════════════════════════════════════════════
    L_fake_gripper_tf_node = Node(
        package='tools',
        executable='fake_gripper_tf_publisher_node',
        name='Lfake_gripper_tf_publisher',
        parameters=[{
            'gripper_frame': 'Lgripper_tip',
            'base_frame': 'Lrobot_base',
            'fake_frame': 'Lfake_gripper_frame',
            'reference_frame': 'world',
            'robot_name': 'L',
            'rate': 50.0,
        }],
        output='screen',
    )

    R_fake_gripper_tf_node = Node(
        package='tools',
        executable='fake_gripper_tf_publisher_node',
        name='Rfake_gripper_tf_publisher',
        parameters=[{
            'gripper_frame': 'Rgripper_tip',
            'base_frame': 'Rrobot_base',
            'fake_frame': 'Rfake_gripper_frame',
            'reference_frame': 'world',
            'robot_name': 'R',
            'rate': 50.0,
        }],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    # 视觉节点
    # ═══════════════════════════════════════════════════════════════
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

    depth_node = Node(
        package='depth_handler',
        executable='depth_processor_node',
        name='depth_handler_node',
        output='screen',
    )

    return LaunchDescription([
        L_ip_arg,
        R_ip_arg,
        L_gripper_port_arg,
        R_gripper_port_arg,
        confidence_threshold_arg,
        model_path_arg,
        camera_launch,
        static_tf_launch,
        L_robo_ctrl_node,
        L_high_level_node,
        R_robo_ctrl_node,
        R_high_level_node,
        L_gripper_node,
        R_gripper_node,
        L_fake_gripper_tf_node,
        R_fake_gripper_tf_node,
        detector_node,
        depth_node,
    ])
