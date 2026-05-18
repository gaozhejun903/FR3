from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOTNAME = 'R'  # 默认机器人名称



def generate_launch_description():
    # 声明启动参数
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='10.2.20.202',
        description='机器人控制器的IP地址'
    )
    
    robot_port_arg = DeclareLaunchArgument(
        'robot_port',
        default_value='8080',
        description='机器人控制器的端口号'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=ROBOTNAME,
        description='机器人名称'
    )
    state_query_interval_arg = DeclareLaunchArgument(
        'state_query_interval',
        default_value='0.01',
        description='状态查询间隔时间'
    )
    gripper_frame = LaunchConfiguration('gripper_frame', default=ROBOTNAME+'gripper')
    base_frame = LaunchConfiguration('base_frame', default=ROBOTNAME+'robot_base')
    fake_frame = LaunchConfiguration('fake_frame', default=ROBOTNAME+'fake_gripper_frame')
    reference_frame = LaunchConfiguration('reference_frame', default='world')
    rate = LaunchConfiguration('rate', default='10.0')
    
    # 声明launch参数
    declare_gripper_frame_cmd = DeclareLaunchArgument(
        'gripper_frame',
        default_value=ROBOTNAME+'gripper',
        description='夹爪坐标系名称')
        
    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value=ROBOTNAME+'robot_base',
        description='基座坐标系名称')
        
    declare_fake_frame_cmd = DeclareLaunchArgument(
        'fake_frame',
        default_value=ROBOTNAME+'fake_gripper_frame',
        description='虚假坐标系名称')
        
    declare_reference_frame_cmd = DeclareLaunchArgument(
        'reference_frame',
        default_value='world',
        description='参考坐标系名称')
        
    declare_rate_cmd = DeclareLaunchArgument(
        'rate',
        default_value='50.0',
        description='发布频率(Hz)')
    
    # 创建机器人控制节点
    robo_ctrl_node = Node(
        package='robo_ctrl',
        executable='robo_ctrl_node',
        name=ROBOTNAME+'robo_ctrl',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'robot_port': LaunchConfiguration('robot_port'),
            'robot_name': LaunchConfiguration('robot_name'),
            'state_query_interval': LaunchConfiguration('state_query_interval')
        }],
        output='screen'
    )
    # 创建规划和执行节点
    high_level_node = Node(
        package='robo_ctrl',
        executable='high_level_node',
        name=ROBOTNAME+'high_level',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'robot_port': LaunchConfiguration('robot_port'),
            'robot_name': LaunchConfiguration('robot_name'),
            'state_query_interval': LaunchConfiguration('state_query_interval')
        }],
        output='screen'
    )
    fake_gripper_tf_publisher_node = Node(
        package='tools',
        executable='fake_gripper_tf_publisher_node',
        name=ROBOTNAME+'fake_gripper_tf_publisher',
        parameters=[{
            'gripper_frame': gripper_frame,
            'base_frame': base_frame,
            'fake_frame': fake_frame,
            'reference_frame': reference_frame,
            'rate': rate,
        }],
        output='screen'
    )

    pkg_dir = get_package_share_directory('tools')
    
    # 声明参数 - 合并后的静态TF配置
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'static_transforms.yaml'),
        description='静态TF配置文件路径，包含多个TF变换'
    )
    # 创建静态TF发布节点

    
    # 返回启动描述
    return LaunchDescription([
        robot_ip_arg,
        robot_port_arg,
        robot_name_arg,
        state_query_interval_arg,
        robo_ctrl_node,
        high_level_node
        , fake_gripper_tf_publisher_node,
        declare_gripper_frame_cmd,
        declare_base_frame_cmd,
        declare_fake_frame_cmd,
        declare_reference_frame_cmd,
        declare_rate_cmd,
        config_file_arg,
    ])