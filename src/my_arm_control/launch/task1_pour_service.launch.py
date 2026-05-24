"""任务一倒水服务 — 模拟测试启动文件。

启动顺序:
  1. mock_robo_ctrl_L (模拟左臂)
  2. mock_robo_ctrl_R (模拟右臂)
  3. virtual_vision_task1 (发布瓶子+杯子的假坐标, 发布在 camera_depth_frame)
  4. static_transform_publisher (camera_depth_frame → base_link)
  5. task1_pour_service (倒水服务状态机, 带 TF 坐标变换, 延迟3秒启动)

坐标系说明:
  - 虚拟视觉节点发布坐标在 camera_depth_frame
  - static_transform_publisher 定义了 camera → base_link 的外参
  - task1_pour_service 收到视觉数据后, 使用 TF 自动变换到 base_link
  - 实机部署时, 只需修改 static_transform_publisher 的参数即可
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_prefix = LaunchConfiguration('robot_prefix', default='/L')

    mock_L = Node(
        package='my_arm_control',
        executable='mock_robo_ctrl_node',
        name='mock_robo_ctrl_L',
        namespace='L',
        parameters=[{'robot_name': 'L'}],
        output='screen',
    )

    mock_R = Node(
        package='my_arm_control',
        executable='mock_robo_ctrl_node',
        name='mock_robo_ctrl_R',
        namespace='R',
        parameters=[{'robot_name': 'R'}],
        output='screen',
    )

    vision = Node(
        package='my_arm_control',
        executable='virtual_vision_task1',
        name='virtual_vision_task1',
        parameters=[{
            'bottle_water_x': 250.0,
            'bottle_water_y': -200.0,
            'bottle_water_z': 360.0,
            'bottle_cola_x': 300.0,
            'bottle_cola_y': -100.0,
            'bottle_cola_z': 360.0,
            'cup_1_x': 200.0,
            'cup_1_y': -150.0,
            'cup_1_z': 360.0,
            'cup_2_x': 280.0,
            'cup_2_y': -250.0,
            'cup_2_z': 360.0,
            'frame_id': 'camera_depth_frame',
            'rate': 5.0,
        }],
        output='screen',
    )

    # 模拟相机外参: camera_depth_frame → base_link
    # 实机部署时根据实际相机标定结果修改 x/y/z/roll/pitch/yaw
    camera_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_to_base',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_depth_frame',
        ],
        output='screen',
    )

    pour_service = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='my_arm_control',
                executable='task1_pour_service',
                name='task1_pour_service',
                parameters=[{
                    'robot_prefix': robot_prefix,
                    'observe_pose': [99.917, -144.210, 542.554, -125.357, 0.0, -100.476],
                    'velocity': 50.0,
                    'acceleration': 50.0,
                    'desk_height': 360.0,
                    'bottle_height': 180.0,
                    'cup_height': 120.0,
                    'approach_offset_z': 150.0,
                    'retreat_z': 80.0,
                    'cap_open_joints_l': [-55.0, -90.0, -120.0, 30.0, 81.272, 0.0],
                    'cap_open_joints_r': [45.434, -124.551, 128.388, -184.270, 19.218, 0.0],
                    'pour_j6_angle': 60.0,
                    'place_bottle_pose': [200.0, -200.0, 200.0, -90.0, 0.0, -90.0],
                    'place_cup_pose': [250.0, -100.0, 200.0, -90.0, 0.0, -90.0],
                    'gripper_id_L': 9,
                    'gripper_id_R': 10,
                    'drink_order': ['water', 'cola'],
                    'target_frame': 'base_link',
                    'vision_frame': 'camera_depth_frame',
                    'tf_timeout': 1.0,
                }],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_prefix', default_value='/L'),
        mock_L,
        mock_R,
        vision,
        camera_to_base,
        pour_service,
    ])
