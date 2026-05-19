#!/usr/bin/env python3
"""
一键诊断脚本：对比 /detector/detections 和深度图话题的时间戳差异

用法:
  cd /home/mihu/FR3_again
  source install/setup.bash
  python3 src/detector/scripts/check_sync.py

或安装后:
  ros2 run detector check_sync.py
"""

import rclpy
from rclpy.node import Node
from detector.msg import Bbox2dArray
from sensor_msgs.msg import Image
import time
import sys

class SyncChecker(Node):
    def __init__(self, depth_topic):
        super().__init__('sync_checker')
        self.depth_topic = depth_topic
        self.det_times = []
        self.dep_times = []
        self.max_samples = 30  # 最多采集30帧
        
        self.det_sub = self.create_subscription(
            Bbox2dArray, '/detector/detections', self.det_cb, 10
        )
        self.dep_sub = self.create_subscription(
            Image, depth_topic, self.dep_cb, 10
        )
        
        self.start_time = time.time()
        self.get_logger().info(f'开始采集: 检测框=/detector/detections, 深度图={depth_topic}')
        self.get_logger().info('请确保 detector 和深度相机都在运行...')
        self.get_logger().info('采集10秒后自动输出报告')
        
    def stamp_to_sec(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9
        
    def det_cb(self, msg):
        t = self.stamp_to_sec(msg.header.stamp)
        self.det_times.append(t)
        if len(self.det_times) <= self.max_samples:
            self.get_logger().info(f'[DET] t={t:.6f}, 累计{len(self.det_times)}帧')
            
    def dep_cb(self, msg):
        t = self.stamp_to_sec(msg.header.stamp)
        self.dep_times.append(t)
        if len(self.dep_times) <= self.max_samples:
            self.get_logger().info(f'[DEP] t={t:.6f}, 累计{len(self.dep_times)}帧')
            
    def check_done(self):
        elapsed = time.time() - self.start_time
        return elapsed > 10.0 and (len(self.det_times) > 0 or len(self.dep_times) > 0)
        
    def report(self):
        print('\n' + '='*60)
        print('时间戳对比报告')
        print('='*60)
        print(f'深度图话题: {self.depth_topic}')
        print(f'检测框帧数: {len(self.det_times)}')
        print(f'深度图帧数: {len(self.dep_times)}')
        
        if len(self.det_times) == 0:
            print('\n[!] 错误: 没有收到 /detector/detections')
            print('    请确认 detector 节点正在运行')
            return False
            
        if len(self.dep_times) == 0:
            print('\n[!] 错误: 没有收到深度图')
            print(f'    请确认深度相机正在发布 {self.depth_topic}')
            print('    常见深度话题: /camera/depth/image_rect_raw, /camera/aligned_depth_to_color/image_raw')
            return False
        
        # 计算最近10帧检测框与深度图的时间差
        diffs = []
        for dt in self.det_times[-10:]:
            closest = min(self.dep_times, key=lambda x: abs(x - dt))
            diffs.append(abs(dt - closest))
            
        avg_diff = sum(diffs) / len(diffs)
        max_diff = max(diffs)
        min_diff = min(diffs)
        
        print(f'\n最近10帧配对统计:')
        print(f'  平均时间差: {avg_diff*1000:.2f} ms')
        print(f'  最大时间差: {max_diff*1000:.2f} ms')
        print(f'  最小时间差: {min_diff*1000:.2f} ms')
        
        # 检查时间基准是否漂移
        if len(self.det_times) >= 2 and len(self.dep_times) >= 2:
            det_period = self.det_times[-1] - self.det_times[0]
            dep_period = self.dep_times[-1] - self.dep_times[0]
            print(f'\n采集期间时间跨度:')
            print(f'  检测框: {det_period:.2f} 秒')
            print(f'  深度图: {dep_period:.2f} 秒')
            if abs(det_period - dep_period) > 1.0:
                print('\n[!] 警告: 两个话题的时间跨度差异很大')
                print('    可能使用了不同的时间源（系统时间 vs 硬件时间）')
        
        if max_diff > 0.1:
            print('\n[!] 警告: 最大时间差 > 100ms')
            print('    ApproximateTime 同步可能失败！')
            print('    建议: 检查相机驱动时间戳设置，或增大同步队列')
        else:
            print('\n[OK] 时间差在合理范围内，同步应该正常')
            
        # 打印示例时间戳
        print('\n示例时间戳对比:')
        for i, dt in enumerate(self.det_times[-3:]):
            closest = min(self.dep_times, key=lambda x: abs(x - dt))
            diff_ms = abs(dt - closest) * 1000
            print(f'  检测框 {i+1}: {dt:.6f} -> 最近深度图: {closest:.6f} (差 {diff_ms:.2f} ms)')
        
        return True


def main():
    # 常见深度图话题候选
    depth_candidates = [
        '/camera/depth/image_rect_raw',
        '/camera/depth/image_raw',
        '/camera/aligned_depth_to_color/image_raw',
    ]
    
    rclpy.init()
    
    # 尝试自动检测正在发布的深度话题
    import subprocess
    active_topics = subprocess.check_output(['ros2', 'topic', 'list']).decode()
    depth_topic = None
    for candidate in depth_candidates:
        if candidate in active_topics:
            depth_topic = candidate
            break
            
    if depth_topic is None:
        print('未检测到常见深度话题，请手动指定')
        print('用法: python3 check_sync.py /camera/depth/image_raw')
        sys.exit(1)
    
    node = SyncChecker(depth_topic)
    
    # 主循环：采集10秒或收到足够数据
    start = time.time()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start > 10.0:
            break
            
    success = node.report()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
