from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 创建launch参数
    gripper_frame = LaunchConfiguration('gripper_frame', default='gripper_link')
    base_frame = LaunchConfiguration('base_frame', default='base_link')
    fake_frame = LaunchConfiguration('fake_frame', default='fake_gripper_frame')
    reference_frame = LaunchConfiguration('reference_frame', default='world')
    rate = LaunchConfiguration('rate', default='10.0')
    
    # 声明launch参数
    declare_gripper_frame_cmd = DeclareLaunchArgument(
        'gripper_frame',
        default_value='Lgripper',
        description='夹爪坐标系名称')
        
    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value='Lrobot_base',
        description='基座坐标系名称')
        
    declare_fake_frame_cmd = DeclareLaunchArgument(
        'fake_frame',
        default_value='Lfake_gripper_frame',
        description='虚假坐标系名称')
        
    declare_reference_frame_cmd = DeclareLaunchArgument(
        'reference_frame',
        default_value='world',
        description='参考坐标系名称')
        
    declare_rate_cmd = DeclareLaunchArgument(
        'rate',
        default_value='10.0',
        description='发布频率(Hz)')
    
    # 创建节点
    fake_gripper_tf_publisher_node = Node(
        package='tools',
        executable='fake_gripper_tf_publisher_node',
        name='fake_gripper_tf_publisher',
        parameters=[{
            'gripper_frame': gripper_frame,
            'base_frame': base_frame,
            'fake_frame': fake_frame,
            'reference_frame': reference_frame,
            'rate': rate,
        }],
        output='screen'
    )
    
    # 返回LaunchDescription
    return LaunchDescription([
        declare_gripper_frame_cmd,
        declare_base_frame_cmd,
        declare_fake_frame_cmd,
        declare_reference_frame_cmd,
        declare_rate_cmd,
        fake_gripper_tf_publisher_node
    ])