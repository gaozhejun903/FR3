# 双夹爪启动文件 (左爪 L + 右爪 R + 虚假末端TF)
# 用法: ros2 launch tools grippers.launch.py
#
# 包含:
#   - 串口设备权限设置 (chmod 777)
#   - 左夹爪: epg50_gripper_node + fake_gripper_tf_publisher (L)
#   - 右夹爪: epg50_gripper_node + fake_gripper_tf_publisher (R)
#
# fake_gripper_tf: 为 high_level 轨迹规划提供水平参考系
#   Lfake_gripper_frame / Rfake_gripper_frame: 位置=夹爪尖端，Z轴=世界朝上

import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ═══════════════════════════════════════════════════════════════
    # 夹爪设备权限
    # ═══════════════════════════════════════════════════════════════
    password = '123'
    for dev in ['/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2']:
        if os.path.exists(dev):
            subprocess.run(
                ['sudo', '-S', 'chmod', '777', dev],
                input=password + '\n', encoding='utf-8',
                capture_output=True
            )

    # ═══════════════════════════════════════════════════════════════
    # 夹爪端口参数 (by-path 持久化路径，重启不变)
    # ═══════════════════════════════════════════════════════════════
    L_gripper_port_arg = DeclareLaunchArgument(
        'L_gripper_port',
        default_value='/dev/serial/by-path/pci-0000:05:00.4-usb-0:1.4:1.0-port0',
        description='左夹爪串口 (by-path) — USB拓扑变更后更新 2026-06-06'
    )
    R_gripper_port_arg = DeclareLaunchArgument(
        'R_gripper_port',
        default_value='/dev/serial/by-path/pci-0000:05:00.4-usb-0:1.2:1.0-port0',
        description='右夹爪串口 (by-path) — USB拓扑变更后更新 2026-06-06'
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
            'robot_name': 'L',
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
            'robot_name': 'R',
        }],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    # 左夹爪虚假TF — 为 high_level 轨迹规划提供水平参考系
    # Lfake_gripper_frame: 位置=夹爪尖端, Z轴=世界朝上, X轴=夹爪指向(水平投影)
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

    # ═══════════════════════════════════════════════════════════════
    # 右夹爪虚假TF
    # ═══════════════════════════════════════════════════════════════
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

    return LaunchDescription([
        L_gripper_port_arg,
        R_gripper_port_arg,
        L_gripper_node,
        R_gripper_node,
        L_fake_gripper_tf_node,
        R_fake_gripper_tf_node,
    ])
