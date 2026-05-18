from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明参数
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='/L',
        description='Robot name prefix for topics and services'
    )
    
    # 键盘控制器节点
    keyboard_controller_node = Node(
        package='tools',
        executable='keyboard_tcp_controller_node',
        name='keyboard_tcp_controller',
        parameters=[{
            'robot_name': LaunchConfiguration('robot_name')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        robot_name_arg,
        keyboard_controller_node
    ])
