"""
实机测试启动脚本 — 仅启动任务节点

前置条件 (已由 robo_ctrl_L.launch.py 启动):
  - robo_ctrl_node (左臂驱动)
  - high_level_node (左臂高级控制)
  - detector_node (YOLOv8 检测)
  - camera_info_interceptor (camera_info 适配)
  - depth_handler_node (3D 体素聚类)
  - static_tf_publisher (静态 TF)
  - gripper_node (夹爪节点)

另外需要:
  - robo_ctrl_R.launch.py 已启动 (右臂驱动)

启动内容:
  - arm_task_node (核心任务管理器, 延迟 5s 等待服务就绪)

使用:
  # 单臂抓取 (默认)
  ros2 launch my_arm_control real_hardware.launch.py

  # 拧瓶盖任务
  ros2 launch my_arm_control real_hardware.launch.py task_mode:=opencap

  # 接球任务
  ros2 launch my_arm_control real_hardware.launch.py task_mode:=ball
"""

from launch import LaunchDescription
from launch.actions import (
    TimerAction,
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- 命令行参数 ---------------------------------------------------
    robot_prefix = LaunchConfiguration("robot_prefix", default="/L")
    target_class_id = LaunchConfiguration("target_class_id", default="-1")
    velocity = LaunchConfiguration("velocity", default="50")
    gripper_slave_id = LaunchConfiguration("gripper_slave_id", default="9")
    task_mode = LaunchConfiguration("task_mode", default="grab")

    # =================================================================
    # 核心任务节点 — 延迟 5s 等待驱动和视觉管线就绪
    # =================================================================
    arm_task_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="my_arm_control",
                executable="arm_task_node",
                name="arm_task_manager",
                parameters=[{
                    "robot_prefix": robot_prefix,
                    "target_class_id": target_class_id,
                    "velocity": 50.0,
                    "gripper_slave_id": gripper_slave_id,
                    "task_mode": task_mode,
                    # 高度参数 (mm)
                    "desk_height": 360.0,
                    "object_height": 89.0,
                    # 观测位姿 (mm/度)
                    "observe_x": 99.917,
                    "observe_y": -144.210,
                    "observe_z": 542.554,
                    "observe_rx": -125.357,
                    "observe_ry": 0.0,
                    "observe_rz": -100.476,
                    # 放置位姿 (mm/度)
                    "place_x": 200.0,
                    "place_y": -300.0,
                    "place_z": 200.0,
                    "place_rx": -90.0,
                    "place_ry": 0.0,
                    "place_rz": -90.0,
                    # 拧瓶盖: 预设关节角度 (度)
                    "cap_open_joints_l": [-55.0, -90.0, -120.0, 30.0, 81.272, 0.0],
                    "cap_open_joints_r": [45.434, -124.551, 128.388, -184.270, 19.218, 0.0],
                    # 拧瓶盖: TCP 到可乐偏移 (mm)
                    "cola_offset_x": -132.0,
                    "cola_offset_y": 45.0,
                }],
                output="screen",
                emulate_tty=True,
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_prefix", default_value="/L"),
        DeclareLaunchArgument("target_class_id", default_value="-1"),
        DeclareLaunchArgument("velocity", default_value="50"),
        DeclareLaunchArgument("gripper_slave_id", default_value="9"),
        DeclareLaunchArgument("task_mode", default_value="grab"),

        arm_task_node,
    ])
