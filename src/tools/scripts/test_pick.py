#!/usr/bin/env python3
"""
读取深度节点检测到的物体3D坐标，驱动机械臂移动到物体上方抓取。
使用 robot_move_cart (已验证可用)，速度 20%。
用法: python3 test_pick.py
"""

import rclpy
from rclpy.node import Node
from depth_handler.msg import Bbox3dArray
from robo_ctrl.srv import RobotMoveCart
from robo_ctrl.msg import TCPPose, RobotState


class PickTest(Node):
    def __init__(self):
        super().__init__("pick_test")
        self.move_client = self.create_client(RobotMoveCart, "/L/robot_move_cart")
        self.bbox_sub = self.create_subscription(
            Bbox3dArray, "/depth_handler/bbox3d", self.bbox_callback, 10
        )
        self.state_sub = self.create_subscription(
            RobotState, "/L/robot_state", self.state_callback, 10
        )
        self.current_rx = None
        self.current_ry = None
        self.current_rz = None

        self.get_logger().info("等待 robot_move_cart 服务...")
        self.move_client.wait_for_service()
        self.get_logger().info("就绪，等待检测到物体...")

    def state_callback(self, msg: RobotState):
        self.current_rx = msg.tcp_pose.rx
        self.current_ry = msg.tcp_pose.ry
        self.current_rz = msg.tcp_pose.rz

    def bbox_callback(self, msg: Bbox3dArray):
        if not msg.results:
            return
        if self.current_rx is None:
            self.get_logger().warn("尚未收到机器人姿态，跳过")
            return

        obj = msg.results[0]
        cx = obj.x + obj.width / 2
        cy = obj.y + obj.height / 2
        cz = obj.z + obj.depth / 2

        self.get_logger().info(f"检测到物体 class={obj.class_id} 中心=({cx:.3f},{cy:.3f},{cz:.3f})")
        self.destroy_subscription(self.bbox_sub)

        gripper_length = 0.15

        # 第一步：移到物体上方
        above_z = cz + gripper_length + 0.05
        self.move_to(cx, cy, above_z, "移到物体上方")

        # 第二步：下探
        grasp_z = cz + gripper_length
        self.move_to(cx, cy, grasp_z, "下探抓取")

        self.get_logger().info("完成！")

    def move_to(self, x_m, y_m, z_m, desc=""):
        req = RobotMoveCart.Request()
        req.tcp_pose = TCPPose()
        req.tcp_pose.x = x_m * 1000.0      # m → mm
        req.tcp_pose.y = y_m * 1000.0
        req.tcp_pose.z = z_m * 1000.0
        req.tcp_pose.rx = self.current_rx
        req.tcp_pose.ry = self.current_ry
        req.tcp_pose.rz = self.current_rz
        req.velocity = 20.0
        req.acceleration = 20.0
        req.ovl = 100.0
        req.blend_time = -1.0              # 阻塞
        req.use_increment = False
        req.config = -1

        self.get_logger().info(f"{desc}: ({x_m:.3f},{y_m:.3f},{z_m:.3f})m 姿态({self.current_rx:.1f},{self.current_ry:.1f},{self.current_rz:.1f})")
        future = self.move_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp is not None and resp.success:
            self.get_logger().info(f"  {desc} 成功")
        else:
            self.get_logger().error(f"  {desc} 失败: {resp.message if resp else '无响应'}")


def main():
    rclpy.init()
    node = PickTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
