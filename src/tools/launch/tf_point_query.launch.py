from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """创建并返回一个LaunchDescription对象，用于启动TF点位置查询服务节点。
    
    该启动文件将启动一个TF点位置查询服务节点，该节点提供一个服务来查询一个坐标系下的点在另一个坐标系下的位置。
    """
    
    # 创建TF点位置查询节点
    tf_point_query_node = Node(
        package='tools',
        executable='tf_point_query_node',
        name='tf_point_query',
        output='screen'
    )
    
    # 返回LaunchDescription对象
    return LaunchDescription([
        tf_point_query_node,
    ])