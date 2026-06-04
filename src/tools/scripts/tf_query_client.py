#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TF 点位置查询客户端脚本

该脚本提供了一个命令行工具，用于调用 transform_point 服务，
将一个坐标系下的点转换到另一个坐标系下。
"""

import sys
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from tools.srv import TransformPoint

class TFQueryClient(Node):
    """TF点位置查询客户端"""

    def __init__(self):
        """初始化客户端节点"""
        super().__init__('tf_query_client')
        self.client = self.create_client(TransformPoint, 'transform_point')
        
        # 等待服务可用
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('服务不可用，请确保 tf_point_query 节点已运行')
            sys.exit(1)
            
        self.request = TransformPoint.Request()
            
    def send_request(self, source_frame, target_frame, x, y, z, timeout=1.0):
        """
        发送坐标转换请求
        
        参数:
            source_frame (str): 源坐标系名称
            target_frame (str): 目标坐标系名称
            x, y, z (float): 源坐标系中的点坐标
            timeout (float): 等待变换的超时时间（秒）
            
        返回:
            Future对象，用于获取响应
        """
        self.request.source_frame = source_frame
        self.request.target_frame = target_frame
        self.request.point = Point(x=float(x), y=float(y), z=float(z))
        self.request.timeout = float(timeout)
        
        self.get_logger().info(f'发送请求: 从 {source_frame} 到 {target_frame} 转换点 [{x}, {y}, {z}]')
        return self.client.call_async(self.request)


def main():
    """主函数"""
    # 命令行参数解析
    parser = argparse.ArgumentParser(description='TF点位置查询客户端')
    parser.add_argument('source_frame', help='源坐标系名称')
    parser.add_argument('target_frame', help='目标坐标系名称')
    parser.add_argument('x', type=float, help='X坐标')
    parser.add_argument('y', type=float, help='Y坐标')
    parser.add_argument('z', type=float, help='Z坐标')
    parser.add_argument('--timeout', '-t', type=float, default=1.0, help='等待变换的超时时间（秒）')
    
    args = parser.parse_args()
    
    # 初始化ROS客户端
    rclpy.init()
    client = TFQueryClient()
    
    # 发送请求
    future = client.send_request(
        args.source_frame,
        args.target_frame,
        args.x, args.y, args.z,
        args.timeout
    )
    
    # 等待并处理响应
    rclpy.spin_until_future_complete(client, future)
    
    # 处理响应结果
    if future.done():
        try:
            response = future.result()
            
            if response.success:
                point = response.transformed_point.point
                frame = response.transformed_point.header.frame_id
                client.get_logger().info(f'转换成功: 结果点 [{point.x}, {point.y}, {point.z}] 在 {frame} 坐标系下')
                print(f"\n转换结果:")
                print(f"坐标系: {frame}")
                print(f"X: {point.x}")
                print(f"Y: {point.y}")
                print(f"Z: {point.z}\n")
                return 0
            else:
                client.get_logger().error(f'转换失败: {response.error_msg}')
                return 1
        except Exception as e:
            client.get_logger().error(f'处理响应时出错: {str(e)}')
            return 1
    else:
        client.get_logger().error('服务调用失败')
        return 1
    
    # 关闭客户端
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())