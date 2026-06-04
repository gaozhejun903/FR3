#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
机器人位姿记录脚本

该脚本订阅机器人状态话题，并提供服务来打印当前TCP位姿信息。
当调用服务时，将打印最新的x, y, z, rx, ry, rz位姿数据。
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from robo_ctrl.msg import RobotState


class PoseRecorder(Node):
    """位姿记录节点"""

    def __init__(self):
        """初始化节点"""
        super().__init__('pose_recorder')
        
        # 使用可重入回调组以支持并发操作
        self.callback_group = ReentrantCallbackGroup()
        
        # 存储最新的机器人状态
        self.latest_robot_state = None
        
        # 订阅机器人状态话题
        self.state_subscription = self.create_subscription(
            RobotState,
            '/L/robot_state',
            self.state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 创建服务来打印当前位姿
        self.print_pose_service = self.create_service(
            Trigger,
            'print_current_pose',
            self.print_pose_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('位姿记录节点已启动')
        self.get_logger().info('订阅话题: state')
        self.get_logger().info('提供服务: print_current_pose')
        
    def state_callback(self, msg):
        """机器人状态回调函数"""
        self.latest_robot_state = msg
        # 可选择性地记录日志（可根据需要取消注释）
        # self.get_logger().debug(f'收到机器人状态更新: TCP位姿 x={msg.tcp_pose.x:.3f}, y={msg.tcp_pose.y:.3f}, z={msg.tcp_pose.z:.3f}')
        
    def print_pose_callback(self, request, response):
        """打印位姿服务回调函数"""
        if self.latest_robot_state is None:
            response.success = False
            response.message = "尚未接收到机器人状态数据"
            self.get_logger().warn("服务调用失败: 尚未接收到机器人状态数据")
            return response
            
        # 获取TCP位姿数据
        tcp_pose = self.latest_robot_state.tcp_pose
        
        # 打印位姿信息
        pose_info = (
            f"当前TCP位姿:\n"
            f"{tcp_pose.x:.3f} "
            f", {tcp_pose.y:.3f} "
            f", {tcp_pose.z:.3f} "
            f", {tcp_pose.rx:.3f} "
            f", {tcp_pose.ry:.3f} "
            f", {tcp_pose.rz:.3f} "
        )

        joint_info = (
            f"关节角度:\n"
            f"{self.latest_robot_state.joint_position.j1:.3f}, "
            f"{self.latest_robot_state.joint_position.j2:.3f}, "
            f"{self.latest_robot_state.joint_position.j3:.3f}, "
            f"{self.latest_robot_state.joint_position.j4:.3f}, "
            f"{self.latest_robot_state.joint_position.j5:.3f}, "
            f"{self.latest_robot_state.joint_position.j6:.3f}"
        )
        
        # 同时在终端和日志中输出
        print(pose_info)
        print(joint_info)
        self.get_logger().info(joint_info)
        self.get_logger().info(pose_info)
        
        # 简洁格式输出（单行）
        print(f"x={tcp_pose.x:.3f}, y={tcp_pose.y:.3f}, z={tcp_pose.z:.3f}, "
              f"rx={tcp_pose.rx:.3f}, ry={tcp_pose.ry:.3f}, rz={tcp_pose.rz:.3f}")
        print(f"init_pose: \n    x: {tcp_pose.x:.3f}\n   y: {tcp_pose.y:.3f}\n  z: {tcp_pose.z:.3f}\n  rx: {tcp_pose.rx:.3f}\n   ry: {tcp_pose.ry:.3f}\n   rz: {tcp_pose.rz:.3f}\n")
        
        response.success = True
        response.message = f"位姿已打印: x={tcp_pose.x:.3f}, y={tcp_pose.y:.3f}, z={tcp_pose.z:.3f}, rx={tcp_pose.rx:.3f}, ry={tcp_pose.ry:.3f}, rz={tcp_pose.rz:.3f}"
        
        return response


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        # 创建节点
        pose_recorder = PoseRecorder()
        
        # 使用多线程执行器以支持并发回调
        executor = MultiThreadedExecutor()
        executor.add_node(pose_recorder)
        
        # 等待停止信号
        try:
            executor.spin()
        except KeyboardInterrupt:
            pose_recorder.get_logger().info('收到停止信号，正在关闭节点...')
            
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()