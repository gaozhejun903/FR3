# 双臂启动文件 (左臂 L + 右臂 R)
# 用法: ros2 launch robo_ctrl arms.launch.py
#
# 包含:
#   - 残留进程清理 (防止 SDK TCP 连接占用)
#   - 左臂: robo_ctrl_node + high_level_node  @ 192.168.58.2
#   - 右臂: robo_ctrl_node + high_level_node  @ 192.168.58.3
#
# 依赖: 相机、夹爪、视觉节点可以独立启动，不必等双臂就绪

import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ═══════════════════════════════════════════════════════════════
    # 清理残留进程 (robo_ctrl 和 high_level 的旧实例)
    # ═══════════════════════════════════════════════════════════════
    STALE_PROCS = [
        'robo_ctrl_node',
        'high_level_node',
    ]
    for proc in STALE_PROCS:
        subprocess.run(['pkill', '-9', '-f', proc], capture_output=True)
    subprocess.run(['sleep', '1.0'], capture_output=True)

    # ═══════════════════════════════════════════════════════════════
    # 左臂参数
    # ═══════════════════════════════════════════════════════════════
    L_ip_arg = DeclareLaunchArgument(
        'L_robot_ip', default_value='192.168.58.2',
        description='左臂控制器IP地址'
    )
    # ═══════════════════════════════════════════════════════════════
    # 右臂参数
    # ═══════════════════════════════════════════════════════════════
    R_ip_arg = DeclareLaunchArgument(
        'R_robot_ip', default_value='192.168.58.3',
        description='右臂控制器IP地址'
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

    return LaunchDescription([
        L_ip_arg,
        R_ip_arg,
        L_robo_ctrl_node,
        L_high_level_node,
        R_robo_ctrl_node,
        R_high_level_node,
    ])
