# ╔══════════════════════════════════════════════════════════════════════╗
# ║  AI-Deep: 此文件已废弃!                                           ║
# ║  fake_gripper_tf_publisher_node 已集成到统一 launch:               ║
# ║    ros2 launch depth_handler depth_full.launch.py                  ║
# ╚══════════════════════════════════════════════════════════════════════╝
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
#
# def generate_launch_description():
#     gripper_frame = LaunchConfiguration('gripper_frame', default='gripper_link')
#     base_frame = LaunchConfiguration('base_frame', default='base_link')
#     fake_frame = LaunchConfiguration('fake_frame', default='fake_gripper_frame')
#     reference_frame = LaunchConfiguration('reference_frame', default='world')
#     rate = LaunchConfiguration('rate', default='10.0')
#     declare_gripper_frame_cmd = DeclareLaunchArgument('gripper_frame', default_value='Lgripper')
#     declare_base_frame_cmd = DeclareLaunchArgument('base_frame', default_value='Lrobot_base')
#     declare_fake_frame_cmd = DeclareLaunchArgument('fake_frame', default_value='Lfake_gripper_frame')
#     declare_reference_frame_cmd = DeclareLaunchArgument('reference_frame', default_value='world')
#     declare_rate_cmd = DeclareLaunchArgument('rate', default_value='10.0')
#     fake_gripper_tf_publisher_node = Node(package='tools', executable='fake_gripper_tf_publisher_node', name='fake_gripper_tf_publisher', parameters=[{'gripper_frame': gripper_frame, 'base_frame': base_frame, 'fake_frame': fake_frame, 'reference_frame': reference_frame, 'rate': rate}], output='screen')
#     return LaunchDescription([declare_gripper_frame_cmd, declare_base_frame_cmd, declare_fake_frame_cmd, declare_reference_frame_cmd, declare_rate_cmd, fake_gripper_tf_publisher_node])

# AI-Deep: 空壳函数
def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import LogInfo
    return LaunchDescription([
        LogInfo(msg='⚠️ fake_gripper_tf_publisher.launch.py 已废弃! 请使用: ros2 launch depth_handler depth_full.launch.py')
    ])
