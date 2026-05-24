"""
仿真 / 真实机器人 测试启动脚本

启动内容:
  1. fairino3_v6_moveit2_config 的 demo 环境 (robot_state_publisher +
     move_group + ros2_control + RViz)
  2. virtual_vision_node (模拟视觉, 定时发送 target_pose)
  3. static_tf_publisher (发布 wrist3_link → camera_link)
  4. arm_task_node (核心任务管理器, 延迟启动等待 MoveIt 就绪)

使用:
  # 仿真 (默认)
  ros2 launch my_arm_control simulation_test.launch.py

  # 真实硬件
  ros2 launch my_arm_control simulation_test.launch.py use_fake_hardware:=false

  # 不带 RViz
  ros2 launch my_arm_control simulation_test.launch.py with_rviz:=false

  # 自定义视觉发布频率
  ros2 launch my_arm_control simulation_test.launch.py vision_rate:=2.0
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---- 包路径 -------------------------------------------------------
    my_arm_control_dir   = get_package_share_directory("my_arm_control")
    moveit_config_dir    = get_package_share_directory("fairino3_v6_moveit2_config")

    # ---- 命令行参数 ---------------------------------------------------
    use_sim_time     = LaunchConfiguration("use_sim_time", default="false")
    with_rviz        = LaunchConfiguration("with_rviz", default="true")
    vision_rate      = LaunchConfiguration("vision_rate", default="5.0")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware", default="true")

    # =================================================================
    # 1. MoveIt 2 演示环境 (仿真模式)
    #    - robot_state_publisher   (使用 fake_components 仿真硬件)
    #    - joint_state_publisher_gui  (可手动拖动关节验证运动学)
    #    - move_group              (OMPL 规划器)
    #    - controller_manager + spawners (joint_state_broadcaster,
    #      fairino3_controller)
    #    - RViz2 + MoveIt 面板
    # =================================================================
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_dir, "launch", "demo.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )

    # =================================================================
    # 2. 虚拟视觉节点 — 定时发布 target_pose
    # =================================================================
    vision_node = Node(
        package="my_arm_control",
        executable="virtual_vision_node",
        name="virtual_vision_node",
        parameters=[{
            "use_sim_time": use_sim_time,
            "frame_id": "base_link",
            # 圆弧轨迹参数 → 机械臂做平滑圆弧运动
            "center_x": 0.4,
            "center_y": 0.0,
            "center_z": 0.5,
            "radius": 0.15,
            "angular_velocity": 0.3,
            "plane": "xz",
            # rate=0.2 → 每 5 秒发布一个目标
            "rate": 0.2,
        }],
        output="screen",
    )

    # =================================================================
    # 3. 静态 TF 发布 — 发布 wrist3_link → camera_link (仿真/真实均需要)
    # =================================================================
    static_tf_node = Node(
        package="tools",
        executable="static_tf_publisher_node",
        name="static_tf_publisher",
        parameters=[{
            "config_file": os.path.join(
                my_arm_control_dir, "config", "wrist3_to_camera.yaml"
            ),
        }],
        output="screen",
    )

    # =================================================================
    # 4. 核心任务节点 — 延迟 8s 等待 MoveIt 就绪
    #    真实模式下自动启用夹爪串口并调高运动速度
    # =================================================================
    arm_task_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="my_arm_control",
                executable="arm_task_node",
                name="arm_task_manager",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    # 仿真模式使用 MoveIt 后端
                    "use_moveit": PythonExpression([
                        "True if 'true' == '", use_fake_hardware,
                        "' else False",
                    ]),
                    # 仿真模式传空串口跳过夹爪初始化
                    "gripper_port": PythonExpression([
                        "'' if 'true' == '", use_fake_hardware,
                        "' else '/dev/ttyACM0'",
                    ]),
                    "gripper_slave_id": 9,
                    "velocity_scale": PythonExpression([
                        "0.4 if 'true' == '", use_fake_hardware,
                        "' else 0.6",
                    ]),
                    "acceleration_scale": PythonExpression([
                        "0.4 if 'true' == '", use_fake_hardware,
                        "' else 0.6",
                    ]),
                }],
                output="screen",
                emulate_tty=True,
            )
        ],
    )

    return LaunchDescription([
        # 参数声明 (便于命令行覆盖)
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("with_rviz", default_value="true"),
        DeclareLaunchArgument("vision_rate", default_value="5.0"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),

        demo_launch,
        vision_node,
        static_tf_node,
        arm_task_node,
    ])
