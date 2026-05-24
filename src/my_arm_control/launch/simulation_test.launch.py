"""
模拟测试启动脚本 — 无实机环境下完整测试任务流程

启动内容:
  1. mock_robo_ctrl_node_L  (模拟左臂服务, 发布 /L/robot_state)
  2. mock_robo_ctrl_node_R  (模拟右臂服务, 发布 /R/robot_state)
  3. virtual_vision_node    (发布虚拟物体坐标)
  4. arm_task_node          (核心任务状态机, 延迟启动)

使用:
  # 默认: 单臂抓取任务
  ros2 launch my_arm_control simulation_test.launch.py

  # 拧瓶盖任务
  ros2 launch my_arm_control simulation_test.launch.py task_mode:=opencap
"""

from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    task_mode = LaunchConfiguration("task_mode", default="grab")
    robot_prefix = LaunchConfiguration("robot_prefix", default="/L")

    mock_left = Node(
        package="my_arm_control",
        executable="mock_robo_ctrl_node",
        name="mock_robo_ctrl_L",
        parameters=[{"robot_name": "L"}],
        output="screen",
    )

    mock_right = Node(
        package="my_arm_control",
        executable="mock_robo_ctrl_node",
        name="mock_robo_ctrl_R",
        parameters=[{"robot_name": "R"}],
        output="screen",
    )

    vision_node = Node(
        package="my_arm_control",
        executable="virtual_vision_node",
        name="virtual_vision_node",
        parameters=[{
            "mode": "fixed",
            "frame_id": "base_link",
            "fixed_x": 200.0,
            "fixed_y": -150.0,
            "fixed_z": 450.0,
        }],
        output="screen",
    )

    arm_task_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="my_arm_control",
                executable="arm_task_node",
                name="arm_task_manager",
                parameters=[{
                    "use_moveit": False,
                    "use_virtual_vision": True,
                    "robot_prefix": robot_prefix,
                    "task_mode": task_mode,
                    "target_class_id": -1,
                    "velocity": 50.0,
                    "acceleration": 50.0,
                    "gripper_slave_id": 9,
                    "desk_height": 360.0,
                    "object_height": 89.0,
                    "observe_x": 99.917,
                    "observe_y": -144.210,
                    "observe_z": 542.554,
                    "observe_rx": -125.357,
                    "observe_ry": 0.0,
                    "observe_rz": -100.476,
                    "place_x": 200.0,
                    "place_y": -300.0,
                    "place_z": 200.0,
                    "place_rx": -90.0,
                    "place_ry": 0.0,
                    "place_rz": -90.0,
                    "cap_open_joints_l": [-55.0, -90.0, -120.0, 30.0, 81.272, 0.0],
                    "cap_open_joints_r": [45.434, -124.551, 128.388, -184.270, 19.218, 0.0],
                    "cola_offset_x": -132.0,
                    "cola_offset_y": 45.0,
                }],
                output="screen",
                emulate_tty=True,
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("task_mode", default_value="grab"),
        DeclareLaunchArgument("robot_prefix", default_value="/L"),
        mock_left,
        mock_right,
        vision_node,
        arm_task_node,
    ])
