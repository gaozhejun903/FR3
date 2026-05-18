import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('tools')
    
    # 声明参数 - camera_link 配置
    camera_config_file_arg = DeclareLaunchArgument(
        'camera_config_file',
        default_value=os.path.join(pkg_dir, 'config', 'static_tf_config.yaml'),
        description='相机静态TF配置文件路径'
    )
    
    # 声明参数 - gripper 配置
    gripper_config_file_arg = DeclareLaunchArgument(
        'gripper_config_file',
        default_value=os.path.join(pkg_dir, 'config', 'tcp_to_gripper_tf_config.yaml'),
        description='夹爪静态TF配置文件路径'
    )
    
    # 创建发布 tcp 到 camera_link 的静态 TF 节点
    camera_tf_node = Node(
        package='tools',
        executable='static_tf_publisher_node',
        name='camera_static_tf_publisher',
        parameters=[{
            'config_file': LaunchConfiguration('camera_config_file'),
        }],
        output='screen'
    )
    
    # 创建发布 tcp 到 gripper 的静态 TF 节点
    gripper_tf_node = Node(
        package='tools',
        executable='static_tf_publisher_node',
        name='gripper_static_tf_publisher',
        parameters=[{
            'config_file': LaunchConfiguration('gripper_config_file'),
        }],
        output='screen'
    )
    
    # 返回 LaunchDescription 对象
    return LaunchDescription([
        camera_config_file_arg,
        gripper_config_file_arg,
        camera_tf_node,
        gripper_tf_node,
    ])