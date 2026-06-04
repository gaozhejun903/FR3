#!/usr/bin/env python3
"""
四点法 TCP 标定 — 交互式版本。  # AI-Deep修改：新增交互式四点TCP标定脚本

用法:
    # 左臂
    python3 4point_interactive.py --robot_state_topic /L/robot_state --save_path ./tcp_data/L

    # 右臂
    python3 4point_interactive.py --robot_state_topic /R/robot_state --save_path ./tcp_data/R

    # 断点续采
    python3 4point_interactive.py --robot_state_topic /L/robot_state --save_path ./tcp_data/L --resume

操作:
    1. 固定一个尖点（如标定针）
    2. 移动夹爪尖端触碰该点
    3. 按 Enter 记录当前 TCP 位姿（自动存盘）
    4. 换不同姿态再次触碰同一点，按 Enter 记录
    5. 重复 4 次以上，按 q 结束并计算结果
"""

import rclpy
from rclpy.node import Node
from robo_ctrl.msg import RobotState
import numpy as np
from scipy.spatial.transform import Rotation as R
import argparse
import sys
import signal
import json
import os
from datetime import datetime


class FourPointTCPCalibrator(Node):  # AI-Deep修改：交互式四点TCP标定器
    def __init__(self, robot_state_topic, save_path=None, resume=False):
        super().__init__('four_point_tcp_calibrator')
        self.poses = []
        self.latest_pose = None
        self.save_path = save_path

        self.sub = self.create_subscription(
            RobotState, robot_state_topic, self.robot_state_callback, 10)

        # 断点续采  # AI-Deep修改：支持从文件恢复已采集数据
        if resume and save_path and os.path.isdir(save_path):
            self._load_poses()

        self.get_logger().info(f'订阅话题: {robot_state_topic}')
        if save_path:
            self.get_logger().info(f'数据保存: {save_path}')
        if self.poses:
            self.get_logger().info(f'已恢复 {len(self.poses)} 组历史数据')
        self.get_logger().info('')
        self.get_logger().info('=== 四点法 TCP 标定 ===')
        self.get_logger().info('1. 移动夹爪尖端触碰固定尖点')
        self.get_logger().info('2. 按 Enter 记录当前位姿')
        self.get_logger().info('3. 换姿态重复，至少 4 次')
        self.get_logger().info('4. 输入 q 结束并计算结果')
        self.get_logger().info('')

    def _load_poses(self):  # AI-Deep修改：加载已保存数据
        files = sorted([f for f in os.listdir(self.save_path) if f.startswith('pose_') and f.endswith('.json')])  # AI-Deep修改: 只加载 pose_ 前缀文件
        for fname in files:
            with open(os.path.join(self.save_path, fname), 'r') as fp:
                data = json.load(fp)
            rx = np.deg2rad(data['rx'])
            ry = np.deg2rad(data['ry'])
            rz = np.deg2rad(data['rz'])
            rot = R.from_euler('xyz', [rx, ry, rz])
            x, y, z = data['x'], data['y'], data['z']
            self.poses.append({'R': rot.as_matrix(), 't': np.array([x, y, z])})

    def _save_pose(self, index, x, y, z, rx, ry, rz):  # AI-Deep修改：保存单组数据
        if not self.save_path:
            return
        os.makedirs(self.save_path, exist_ok=True)
        data = {'x': x, 'y': y, 'z': z, 'rx': rx, 'ry': ry, 'rz': rz,
                'timestamp': datetime.now().isoformat()}
        fpath = os.path.join(self.save_path, f'pose_{index:04d}.json')
        with open(fpath, 'w') as fp:
            json.dump(data, fp, indent=2)
        self.get_logger().info(f'已保存: {fpath}')

    def robot_state_callback(self, msg):
        self.latest_pose = msg.tcp_pose

    def record_pose(self):
        if self.latest_pose is None:
            self.get_logger().warn('尚未收到位姿数据')
            return

        # robo_ctrl 导出 tcp_pose 单位为 mm 和 deg，转为 m 和 rad  # AI-Deep修改
        x = self.latest_pose.x / 1000.0
        y = self.latest_pose.y / 1000.0
        z = self.latest_pose.z / 1000.0

        rx = np.deg2rad(self.latest_pose.rx)
        ry = np.deg2rad(self.latest_pose.ry)
        rz = np.deg2rad(self.latest_pose.rz)

        rot = R.from_euler('xyz', [rx, ry, rz])

        self.poses.append({'R': rot.as_matrix(), 't': np.array([x, y, z])})

        # 自动存盘
        idx = len(self.poses) - 1
        self._save_pose(idx, x, y, z, self.latest_pose.rx, self.latest_pose.ry, self.latest_pose.rz)

        self.get_logger().info(f'已记录 {len(self.poses)} 组: '
                               f'pos=[{x:.4f}, {y:.4f}, {z:.4f}] '
                               f'rpy=[{self.latest_pose.rx:.1f}, {self.latest_pose.ry:.1f}, {self.latest_pose.rz:.1f}]')

    def calibrate(self):
        if len(self.poses) < 4:
            self.get_logger().error(f'至少需要 4 组数据，当前仅 {len(self.poses)} 组')
            return None, None, None

        R0, t0 = self.poses[0]['R'], self.poses[0]['t']
        A, b = [], []
        for i in range(1, len(self.poses)):
            Ri, ti = self.poses[i]['R'], self.poses[i]['t']
            A.append(R0 - Ri)
            b.append(ti - t0)

        A = np.vstack(A)
        b = np.vstack(b).reshape(-1, 1)
        offset, residuals, _, _ = np.linalg.lstsq(A, b, rcond=None)

        offset = offset.flatten()

        # 反算固定点位置，用于诊断各组数据质量  # AI-Deep修改
        fixed_points = []
        for i, p in enumerate(self.poses):
            fp = p['R'] @ offset + p['t']
            fixed_points.append(fp)

        return offset, residuals, fixed_points

    def print_result(self, offset, residuals, fixed_points):
        self.get_logger().info('')
        self.get_logger().info('========== 标定结果 ==========')
        self.get_logger().info(f'TCP偏移 (法兰 → 夹爪尖端):')
        self.get_logger().info(f'  x: {offset[0]*1000:.2f} mm')
        self.get_logger().info(f'  y: {offset[1]*1000:.2f} mm')
        self.get_logger().info(f'  z: {offset[2]*1000:.2f} mm')
        if len(residuals) > 0:
            rms = np.sqrt(np.mean(residuals))
            self.get_logger().info(f'拟合残差 RMS: {rms*1000:.3f} mm')
        self.get_logger().info('--------------------------------')

        # 诊断每组数据：反算固定点偏差  # AI-Deep修改
        if fixed_points is not None and len(fixed_points) > 0:
            mean_fp = np.mean(fixed_points, axis=0)
            deviations = [np.linalg.norm(fp - mean_fp) * 1000 for fp in fixed_points]  # mm
            med_dev = np.median(deviations)
            threshold = max(med_dev * 3.0, 10.0)  # 3倍中位偏差 或 至少10mm

            self.get_logger().info(f'各组固定点偏差 (阈值={threshold:.1f}mm):')
            for i, dev in enumerate(deviations):
                flag = ' ⚠️ 异常!' if dev > threshold else ''
                self.get_logger().info(f'  样本{i}: {dev:.1f} mm{flag}')

            if any(d > threshold for d in deviations):
                self.get_logger().warn('')
                self.get_logger().warn('⚠️ 存在异常样本! 建议删除对应 pose_XXXX.json 后重算.')
                self.get_logger().warn(f'  删除命令: rm <保存路径>/pose_{{{",".join(str(i) for i, d in enumerate(deviations) if d > threshold)}}}.json')
                self.get_logger().warn('  然后运行: python3 4point_interactive.py --resume ... 加载数据后直接 q 即可')

        self.get_logger().info('================================')

        # 保存结果文件  # AI-Deep修改：结果自动存盘
        if self.save_path:
            result = {'flange_to_gripper_tip_mm': {'x': offset[0]*1000, 'y': offset[1]*1000, 'z': offset[2]*1000},
                      'num_samples': len(self.poses),
                      'rms_mm': float(np.sqrt(np.mean(residuals)))*1000 if len(residuals) > 0 else None,
                      'per_sample_deviation_mm': [float(d) for d in deviations] if fixed_points is not None else []}
            rpath = os.path.join(self.save_path, 'tcp_calibration_result.json')
            with open(rpath, 'w') as fp:
                json.dump(result, fp, indent=2)
            self.get_logger().info(f'结果已保存: {rpath}')

        self.get_logger().info('')
        self.get_logger().info('将以上偏移写入 robo_ctrl 的 TCP 配置文件即可。')


def main():  # AI-Deep修改
    parser = argparse.ArgumentParser(description='四点法 TCP 标定')
    parser.add_argument('--robot_state_topic', default='/L/robot_state',
                        help='机器人状态话题 (默认: /L/robot_state)')
    parser.add_argument('--save_path', default=None,  # AI-Deep修改：可选保存路径
                        help='数据保存路径 (自动存盘，支持断点续采)')
    parser.add_argument('--resume', action='store_true',  # AI-Deep修改：断点续采开关
                        help='从 save_path 恢复已有数据继续采集')
    args = parser.parse_args()

    rclpy.init()
    node = FourPointTCPCalibrator(args.robot_state_topic, args.save_path, args.resume)

    def signal_handler(*_):
        if len(node.poses) >= 4:
            node.get_logger().info('收到退出信号，开始计算...')
            offset, residuals, fixed_points = node.calibrate()  # AI-Deep修改
            node.print_result(offset, residuals, fixed_points)  # AI-Deep修改
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGTERM, signal_handler)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            ch = input()
            if ch.strip().lower() == 'q':
                if len(node.poses) >= 4:
                    offset, residuals, fixed_points = node.calibrate()  # AI-Deep修改
                    node.print_result(offset, residuals, fixed_points)  # AI-Deep修改
                else:
                    node.get_logger().error(f'至少需要 4 组数据，当前仅 {len(node.poses)} 组')
                break
            node.record_pose()
    except KeyboardInterrupt:
        node.get_logger().info('')
        if len(node.poses) >= 4:
            offset, residuals, fixed_points = node.calibrate()  # AI-Deep修改
            node.print_result(offset, residuals, fixed_points)  # AI-Deep修改
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
