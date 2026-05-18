import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('tools')
    
    # 声明参数 - 合并后的静态TF配置
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'static_transforms.yaml'),
        description='静态TF配置文件路径，包含多个TF变换'
    )
    
    # 创建静态TF发布节点
    static_tf_node = Node(
        package='tools',
        executable='static_tf_publisher_node',
        name='static_tf_publisher',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
        }],
        output='screen',
        emulate_tty=True
    )
    
    # 返回LaunchDescription对象
    return LaunchDescription([
        config_file_arg,
        static_tf_node,
    ])