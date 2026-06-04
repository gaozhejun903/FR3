#!/usr/bin/env python3
"""
双臂标定 — 顺序碰点法 (Point-Pair Method)。  # AI-Deep修改：新增

原理:
    左右臂先后触碰同一个固定点，通过多组点对直接求解 ^{L}T_R。
    不需要双臂同时运动，不需要刚性连接。

用法:
    # 交互式采集
    python3 dualarm_points.py --save_path ./dual_calib_data

    # 断点续采
    python3 dualarm_points.py --save_path ./dual_calib_data --resume

    # 仅计算 (不采集)
    python3 dualarm_points.py --save_path ./dual_calib_data --compute_only

操作:
    1. 左右臂 robo_ctrl 都已启动
    2. 在桌面固定一个尖点
    3. 左臂碰点 → 按 L 记录左臂位置
    4. 右臂碰同一点 → 按 R 记录右臂位置（完成一组）
    5. 换一个点（移动尖点），重复 3-4
    6. 至少 4 组，按 q 计算结果
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


# 已标定的 TCP 偏移 (法兰 → 爪尖)  # AI-Deep修改: 从 static_transforms.yaml 同步
TCP_OFFSET = {
    'L': np.array([-0.00166, 0.00182, 0.18564]),   # m
    'R': np.array([0.00389, 0.00306, 0.19254]),     # m
}


class DualArmPointCalibrator(Node):
    def __init__(self, save_path=None, resume=False):
        super().__init__('dualarm_point_calibrator')
        self.pairs = []   # [(P_left, P_right), ...]  in respective base frames
        self.latest_L = None  # latest left robot state
        self.latest_R = None  # latest right robot state
        self.save_path = save_path
        self.pending_left = None  # 等待配对

        self.sub_L = self.create_subscription(
            RobotState, '/L/robot_state', self.cb_L, 10)
        self.sub_R = self.create_subscription(
            RobotState, '/R/robot_state', self.cb_R, 10)

        if resume and save_path:
            self._load_pairs()

        self.get_logger().info('=== 双臂标定 — 顺序碰点法 ===')
        self.get_logger().info('操作:')
        self.get_logger().info('  按 Enter → 记录 (先左后右自动配对)')
        self.get_logger().info('  按 U → 撤销上一组')
        self.get_logger().info('  重复 4 组以上，输入 q → 计算结果')
        self.get_logger().info('')
        if self.pairs:
            self.get_logger().info(f'已恢复 {len(self.pairs)} 组历史数据')
        if self.pending_left is not None:
            self.get_logger().info('已有待配对的左臂数据，按 Enter 记录右臂')
        self._print_status()

    def _data_ready(self):
        """Check if both arm states have been received."""  # AI-Deep修改
        return self.latest_L is not None and self.latest_R is not None

    def _print_status(self):
        if not self._data_ready():
            self.get_logger().info('[等待数据] 左右臂 robo_ctrl 连接中...')
            return
        if self.pending_left is not None:
            pl = self.pending_left
            self.get_logger().info(f'→ 右臂碰同一点后按 Enter (左臂已记录)')
        else:
            n = len(self.pairs)
            if n == 0:
                self.get_logger().info(f'→ 将固定点放好，左臂碰点后按 Enter')
            else:
                self.get_logger().info(f'✅ 已完成 {n} 组 | → 移动固定点到新位置，左臂碰点后按 Enter')

    def _tip_position(self, msg, side):
        """Compute tip position in base frame from flange pose."""
        x = msg.tcp_pose.x / 1000.0
        y = msg.tcp_pose.y / 1000.0
        z = msg.tcp_pose.z / 1000.0
        rx, ry, rz = np.deg2rad(msg.tcp_pose.rx), np.deg2rad(msg.tcp_pose.ry), np.deg2rad(msg.tcp_pose.rz)
        rot = R.from_euler('xyz', [rx, ry, rz])
        flange_pos = np.array([x, y, z])
        tip_pos = flange_pos + rot.as_matrix() @ TCP_OFFSET[side]
        return tip_pos, flange_pos, rot

    def cb_L(self, msg):
        self.latest_L = msg

    def cb_R(self, msg):
        self.latest_R = msg

    def record_left(self):
        if self.latest_L is None:
            self.get_logger().warn('尚未收到左臂数据')
            return
        tip, _, _ = self._tip_position(self.latest_L, 'L')
        self.pending_left = tip
        self.get_logger().info(f'左臂已记录 → 移动右臂碰同一点后按 Enter')
        self._save_pending()

    def record_right(self):
        if self.pending_left is None:
            self.get_logger().warn('请先按 Enter 记录左臂')
            return
        if self.latest_R is None:
            self.get_logger().warn('尚未收到右臂数据')
            return
        tip, _, _ = self._tip_position(self.latest_R, 'R')
        P_L = self.pending_left
        P_R = tip
        self.pairs.append((P_L, P_R))
        self.pending_left = None
        idx = len(self.pairs) - 1
        self._save_pair(idx, P_L, P_R)
        self.get_logger().info(f'✅ 第 {len(self.pairs)} 组完成')
        self._print_status()

    def undo(self):
        if self.pending_left is not None:
            self.pending_left = None
            self._save_pending()
            self.get_logger().info('已撤销左臂记录')
            self._print_status()
            return
        if self.pairs:
            removed = self.pairs.pop()
            self.get_logger().info(f'已撤销第 {len(self.pairs)+1} 组')
            self._clean_saved_pair(len(self.pairs))
        else:
            self.get_logger().info('没有可撤销的数据')
        self._print_status()

    def calibrate(self):
        if len(self.pairs) < 3:
            self.get_logger().error(f'至少需要 3 组数据，当前仅 {len(self.pairs)} 组')
            return None, None, None

        P_L = np.array([p[0] for p in self.pairs])  # Nx3
        P_R = np.array([p[1] for p in self.pairs])  # Nx3

        # Arun's method: solve R * P_R_i + t = P_L_i  (i=1..N)  # AI-Deep修改
        centroid_L = np.mean(P_L, axis=0)
        centroid_R = np.mean(P_R, axis=0)

        H = (P_R - centroid_R).T @ (P_L - centroid_L)
        U, _, Vt = np.linalg.svd(H)
        R_X = Vt.T @ U.T
        if np.linalg.det(R_X) < 0:
            Vt[-1, :] *= -1
            R_X = Vt.T @ U.T

        t_X = centroid_L - R_X @ centroid_R

        # 残差
        P_R_transformed = (R_X @ P_R.T).T + t_X
        errors = np.linalg.norm(P_L - P_R_transformed, axis=1) * 1000  # mm
        rms = np.sqrt(np.mean(errors**2))

        T_X = np.eye(4)
        T_X[:3, :3] = R_X
        T_X[:3, 3] = t_X

        return T_X, errors, rms

    def print_result(self, T_X, errors, rms):
        R_X = T_X[:3, :3]
        t_X = T_X[:3, 3]
        euler = R.from_matrix(R_X).as_euler('xyz', degrees=True)

        self.get_logger().info('')
        self.get_logger().info('========== 双臂标定结果: ^{L}T_R ==========')
        self.get_logger().info(f'平移 (x, y, z): [{t_X[0]:.4f}, {t_X[1]:.4f}, {t_X[2]:.4f}] m')
        self.get_logger().info(f'欧拉角 (xyz): [{euler[0]:.1f}, {euler[1]:.1f}, {euler[2]:.1f}]°')
        self.get_logger().info(f'RMS: {rms:.2f} mm')
        self.get_logger().info('----------------------------------------------')

        # 各组偏差  # AI-Deep修改
        med_err = np.median(errors)
        threshold = max(med_err * 3.0, 10.0)
        self.get_logger().info(f'各组点对偏差 (阈值={threshold:.1f}mm):')
        for i, err in enumerate(errors):
            flag = ' ⚠️ 异常!' if err > threshold else ''
            self.get_logger().info(f'  第{i+1}组: {err:.1f} mm{flag}')

        if any(e > threshold for e in errors):
            self.get_logger().warn('')
            self.get_logger().warn('⚠️ 存在异常点对! 建议按 U 撤销对应组后重算.')

        self.get_logger().info('==============================================')

        # 保存
        if self.save_path:
            result = {
                'L_TR': {
                    'translation_m': [float(x) for x in t_X],
                    'euler_xyz_deg': [float(x) for x in euler],
                },
                'rms_mm': float(rms),
                'per_pair_error_mm': [float(e) for e in errors],
                'num_pairs': len(self.pairs),
            }
            rpath = os.path.join(self.save_path, 'dual_arm_result.json')
            with open(rpath, 'w') as f:
                json.dump(result, f, indent=2)
            self.get_logger().info(f'结果已保存: {rpath}')

    def _save_pending(self):
        if not self.save_path:
            return
        os.makedirs(self.save_path, exist_ok=True)
        if self.pending_left is not None:
            data = {'P_L': [float(x) for x in self.pending_left]}
            with open(os.path.join(self.save_path, 'pending.json'), 'w') as f:
                json.dump(data, f)
        else:
            pf = os.path.join(self.save_path, 'pending.json')
            if os.path.exists(pf):
                os.remove(pf)

    def _save_pair(self, idx, P_L, P_R):
        if not self.save_path:
            return
        os.makedirs(self.save_path, exist_ok=True)
        data = {'P_L': [float(x) for x in P_L], 'P_R': [float(x) for x in P_R],
                'timestamp': datetime.now().isoformat()}
        with open(os.path.join(self.save_path, f'pair_{idx:04d}.json'), 'w') as f:
            json.dump(data, f, indent=2)

    def _clean_saved_pair(self, idx):
        if not self.save_path:
            return
        fpath = os.path.join(self.save_path, f'pair_{idx:04d}.json')
        if os.path.exists(fpath):
            os.remove(fpath)

    def _load_pairs(self):
        if not self.save_path or not os.path.isdir(self.save_path):
            return
        # 加载已完成点对
        files = sorted([f for f in os.listdir(self.save_path)
                        if f.startswith('pair_') and f.endswith('.json')])
        for fname in files:
            with open(os.path.join(self.save_path, fname)) as f:
                data = json.load(f)
            self.pairs.append((np.array(data['P_L']), np.array(data['P_R'])))
        # 加载待配对
        pf = os.path.join(self.save_path, 'pending.json')
        if os.path.exists(pf):
            with open(pf) as f:
                data = json.load(f)
            self.pending_left = np.array(data['P_L'])


def main():
    parser = argparse.ArgumentParser(description='双臂标定 — 顺序碰点法')
    parser.add_argument('--save_path', default=None, help='数据保存路径')
    parser.add_argument('--resume', action='store_true', help='断点续采')
    parser.add_argument('--compute_only', action='store_true', help='仅计算已有数据')
    args = parser.parse_args()

    rclpy.init()
    node = DualArmPointCalibrator(args.save_path, args.resume)

    if args.compute_only:
        if len(node.pairs) >= 3:
            T_X, errors, rms = node.calibrate()
            node.print_result(T_X, errors, rms)
        else:
            node.get_logger().error(f'至少需要 3 组数据，当前仅 {len(node.pairs)} 组')
        node.destroy_node()
        rclpy.shutdown()
        return

    def signal_handler(*_):
        if len(node.pairs) >= 3:
            node.get_logger().info('收到退出信号，开始计算...')
            T_X, errors, rms = node.calibrate()
            node.print_result(T_X, errors, rms)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGTERM, signal_handler)

    # 等待左右臂数据就绪 (ROS2 发现 ~0.5s)  # AI-Deep修改
    for i in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node._data_ready():
            node.get_logger().info(f'数据就绪 (第{i+1}次spin)')
            break
        if i % 5 == 0:
            node.get_logger().info(f'等待数据中... ({i+1}/20)')
    else:
        node.get_logger().error('未收到左右臂数据，请检查 robo_ctrl')
        # debug: check subscription count
        node.get_logger().info(f'L订阅: {node.sub_L.get_publisher_count()} 个发布者')
        node.get_logger().info(f'R订阅: {node.sub_R.get_publisher_count()} 个发布者')
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            ch = input()
            cmd = ch.strip().lower()
            if cmd == 'q':
                if len(node.pairs) >= 3:
                    T_X, errors, rms = node.calibrate()
                    node.print_result(T_X, errors, rms)
                else:
                    node.get_logger().error(f'至少需要 3 组数据，当前仅 {len(node.pairs)} 组')
                break
            elif cmd == 'u':
                node.undo()
            elif cmd == '':  # AI-Deep修改: Enter 自动判断左/右
                if not node._data_ready():
                    node.get_logger().warn('尚未收到左右臂数据，请等待...')
                elif node.pending_left is None:
                    node.record_left()
                else:
                    node.record_right()
            elif cmd in ('l', 'r'):  # 兼容旧习惯
                if cmd == 'l':
                    node.record_left()
                else:
                    node.record_right()
            else:
                node.get_logger().info('未知命令。Enter=记录 U=撤销 q=完成')
    except KeyboardInterrupt:
        pass  # signal_handler already handles shutdown  # AI-Deep修改: 避免重复 shutdown
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
