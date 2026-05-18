from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 声明启动参数
    save_path_arg = DeclareLaunchArgument(
        'save_path',
        default_value='./tmp/eye_hand_calibration_dataL',
        description='数据保存路径'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/color/image_raw',
        description='相机图像话题'
    )
    
    tcp_pose_topic_arg = DeclareLaunchArgument(
        'tcp_pose_topic',
        default_value='/L/robot_state',
        description='机器人TCP位姿话题(TCPPose类型)'
    )
    
    pose_stamped_topic_arg = DeclareLaunchArgument(
        'pose_stamped_topic',
        default_value='/robot/tcp_pose_stamped',
        description='机器人TCP位姿话题(PoseStamped类型)，优先使用此话题进行消息同步'
    )
    
    robot_state_topic_arg = DeclareLaunchArgument(
        'robot_state_topic',
        default_value='/L/robot_state',
        description='机器人状态话题，包含TCP位姿信息'
    )
    
    auto_capture_arg = DeclareLaunchArgument(
        'auto_capture',
        default_value='false',
        description='是否自动采集数据'
    )
    
    auto_capture_interval_arg = DeclareLaunchArgument(
        'auto_capture_interval',
        default_value='2000',
        description='自动采集间隔（毫秒）'
    )
    
    camera_frame_id_arg = DeclareLaunchArgument(
        'camera_frame_id',
        default_value='camera_color_optical_frame',
        description='相机坐标系ID'
    )
    
    robot_base_frame_id_arg = DeclareLaunchArgument(
        'robot_base_frame_id',
        default_value='robot_base',
        description='机器人基座坐标系ID'
    )
    
    # 创建节点
    eye_hand_calibration_node = Node(
        package='tools',
        executable='eye_hand_calibration_data_collector_node',
        name='eye_hand_calibration_data_collector',
        parameters=[{
            'save_path': LaunchConfiguration('save_path'),
            'image_topic': LaunchConfiguration('image_topic'),
            'tcp_pose_topic': LaunchConfiguration('tcp_pose_topic'),
            'pose_stamped_topic': LaunchConfiguration('pose_stamped_topic'),
            'robot_state_topic': LaunchConfiguration('robot_state_topic'),
            'auto_capture': LaunchConfiguration('auto_capture'),
            'auto_capture_interval': LaunchConfiguration('auto_capture_interval'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'robot_base_frame_id': LaunchConfiguration('robot_base_frame_id'),
        }],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        save_path_arg,
        image_topic_arg,
        tcp_pose_topic_arg,
        pose_stamped_topic_arg,
        robot_state_topic_arg,
        auto_capture_arg,
        auto_capture_interval_arg,
        camera_frame_id_arg,
        robot_base_frame_id_arg,
        eye_hand_calibration_node
    ])