#!/usr/bin/env python3
"""
检查相机图像和机器人状态话题的时间戳是否对齐。
用法:
    python3 check_timestamp_sync.py
    python3 check_timestamp_sync.py --image_topic /camera/color/image_raw --robot_topic /L/robot_state
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from robo_ctrl.msg import RobotState
import argparse
from collections import deque


class TimestampSyncChecker(Node):
    def __init__(self, image_topic, robot_topic):
        super().__init__('timestamp_sync_checker')

        self.image_samples = deque(maxlen=50)
        self.robot_samples = deque(maxlen=50)
        self.pair_count = 0
        self.total_diff = 0.0
        self.max_diff = 0.0
        self.min_diff = float('inf')

        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.robot_sub = self.create_subscription(
            RobotState, robot_topic, self.robot_callback, 10)

        self.get_logger().info(f'订阅图像话题: {image_topic}')
        self.get_logger().info(f'订阅机器人话题: {robot_topic}')
        self.get_logger().info('等待消息... (Ctrl+C 退出)')

    def image_callback(self, msg):
        stamp_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        self.image_samples.append(stamp_ns)
        self.try_match()

    def robot_callback(self, msg):
        stamp_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        self.robot_samples.append(stamp_ns)
        self.try_match()

    def try_match(self):
        if not self.image_samples or not self.robot_samples:
            return

        img_ns = self.image_samples[-1]
        rob_ns = self.robot_samples[-1]
        diff_ms = abs(img_ns - rob_ns) / 1e6

        self.pair_count += 1
        self.total_diff += diff_ms
        self.max_diff = max(self.max_diff, diff_ms)
        self.min_diff = min(self.min_diff, diff_ms)

        if self.pair_count <= 10 or self.pair_count % 20 == 0:
            status = "OK" if diff_ms < 50 else "WARN"
            self.get_logger().info(
                f'[{self.pair_count:3d}] 图像: {img_ns/1e9:.6f}s  '
                f'机器人: {rob_ns/1e9:.6f}s  '
                f'差值: {diff_ms:8.2f}ms  [{status}]'
            )

    def destroy_node(self):
        if self.pair_count > 0:
            avg_ms = self.total_diff / self.pair_count
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'统计 ({self.pair_count} 对消息):')
            self.get_logger().info(f'  平均时间差: {avg_ms:.2f} ms')
            self.get_logger().info(f'  最小时间差: {self.min_diff:.2f} ms')
            self.get_logger().info(f'  最大时间差: {self.max_diff:.2f} ms')
            if avg_ms < 100:
                self.get_logger().info('✓ 时间戳对齐良好，ApproximateTime 同步可用')
            else:
                self.get_logger().warn('✗ 时间偏差较大，请检查系统时钟同步')
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='检查话题时间戳对齐')
    parser.add_argument('--image_topic', default='/camera/color/image_raw')
    parser.add_argument('--robot_topic', default='/L/robot_state')
    args = parser.parse_args()

    import signal
    rclpy.init()
    node = TimestampSyncChecker(args.image_topic, args.robot_topic)

    def signal_handler(*_):
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, signal_handler)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.RCLError):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except rclpy.RCLError:
            pass


if __name__ == '__main__':
    main()
