# 视觉 + TF 启动文件 (检测 + 深度 + 静态TF)
# 用法: ros2 launch depth_handler vision.launch.py
#
# 包含:
#   - 静态 TF 发布 (camera_static_tf + gripper_static_tf)
#   - 目标检测 (YOLO detector)
#   - 深度处理 (depth_processor)
#
# 依赖: 相机需先启动 (ros2 launch orbbec_camera gemini_330_series.launch.py)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ═══════════════════════════════════════════════════════════════
    # 参数
    # ═══════════════════════════════════════════════════════════════
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.4',
        description='目标检测置信度阈值'
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/mihu/FR3_again/src/detector/best2.engine',
        description='YOLO模型路径'
    )

    # ═══════════════════════════════════════════════════════════════
    # 静态 TF 发布 (camera → world / gripper → robot_base)
    # 通过 IncludeLaunchDescription 引入 static_tf_multiple
    # ═══════════════════════════════════════════════════════════════
    tools_launch_dir = get_package_share_directory('tools')
    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tools_launch_dir, 'launch', 'static_tf_multiple.launch.py')
        )
    )

    # ═══════════════════════════════════════════════════════════════
    # 目标检测节点 (YOLO)
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

    # ═══════════════════════════════════════════════════════════════
    # 深度处理节点
    # ═══════════════════════════════════════════════════════════════
    depth_node = Node(
        package='depth_handler',
        executable='depth_processor_node',
        name='depth_handler_node',
        output='screen',
    )

    return LaunchDescription([
        confidence_threshold_arg,
        model_path_arg,
        static_tf_launch,
        detector_node,
        depth_node,
    ])
