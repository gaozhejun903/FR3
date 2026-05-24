from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import subprocess
import os

ROBOTNAME = 'L'

def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.58.2',
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
    
    gripper_port_arg = DeclareLaunchArgument(
        'gripper_port',
        default_value='/dev/ttyUSB0',
        description='夹爪串口路径'
    )
    
    gripper_slave_id_arg = DeclareLaunchArgument(
        'gripper_slave_id',
        default_value='9',
        description='夹爪Modbus从站ID'
    )
    
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
    
    gripper_node = Node(
        package='epg50_gripper_ros',
        executable='epg50_gripper_node',
        name='gripper_node',
        parameters=[{
            'port': LaunchConfiguration('gripper_port'),
            'slave_id': LaunchConfiguration('gripper_slave_id'),
            'debug': False,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        robot_ip_arg,
        robot_port_arg,
        robot_name_arg,
        state_query_interval_arg,
        gripper_port_arg,
        gripper_slave_id_arg,
        robo_ctrl_node,
        high_level_node,
        gripper_node
    ])
