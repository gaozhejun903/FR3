from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_info_interceptor',
            executable='camera_info_interceptor_node',
            name='camera_info_interceptor',
            output='screen',
            parameters=[{
                'input_topic': '/camera/color/camera_info',
                'output_topic': '/camera/camera_info',
                'target_frame_id': 'camera_depth_frame'
            }],
            emulate_tty=True
        )
    ])
