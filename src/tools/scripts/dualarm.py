#!/usr/bin/env python3
"""
双臂标定 — AX=XB 方法求解左臂基座到右臂基座的变换矩阵 ^{L}T_R。  # AI-Deep修改：全面重构

用法:
    # 默认路径
    python3 dualarm.py

    # 自定义路径
    python3 dualarm.py --data_dir /tmp/dual_end_tf_data/dual_poses/

方法:
    双臂同时做多组运动，采集成对末端位姿。
    通过 A_i X = X B_i 求解 X = ^{L}T_R。

操作:
    1. 先完成双臂各自的 TCP 标定 (4point_interactive.py)
    2. 启动 dual_end_tf_collector 采集成对姿态
    3. 两爪尖从不同姿态触碰同一固定点，采集 10-20 组
    4. 运行本脚本求解
"""

import os
import json
import argparse
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

# 支持的手眼标定方法
METHODS = {
    'TSAI':       cv2.CALIB_HAND_EYE_TSAI,
    'PARK':       cv2.CALIB_HAND_EYE_PARK,
    'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
    'ANDREFF':    cv2.CALIB_HAND_EYE_ANDREFF,
}


def pose_to_matrix(pos, quat):
    """Convert position and quaternion to 4x4 transformation matrix."""
    T = np.eye(4)
    rot = R.from_quat([quat['x'], quat['y'], quat['z'], quat['w']])
    T[:3, :3] = rot.as_matrix()
    T[:3, 3] = [pos['x'], pos['y'], pos['z']]
    return T


def load_poses_from_dir(json_dir):  # AI-Deep修改: 只加载 dual_pose_ 前缀
    """Load T_L and T_R from all JSON files in directory."""
    T_L_list, T_R_list = [], []
    fnames = []
    files = sorted([f for f in os.listdir(json_dir)
                    if f.startswith('dual_pose_') and f.endswith('.json')])
    if not files:
        # 回退：匹配所有 json
        files = sorted([f for f in os.listdir(json_dir) if f.endswith('.json')])

    for fname in files:
        try:
            with open(os.path.join(json_dir, fname), 'r') as f:
                data = json.load(f)
            T_L = pose_to_matrix(data['lend_pose']['position'],
                                 data['lend_pose']['orientation'])
            T_R = pose_to_matrix(data['rend_pose']['position'],
                                 data['rend_pose']['orientation'])
            T_L_list.append(T_L)
            T_R_list.append(T_R)
            fnames.append(fname)
        except (KeyError, TypeError) as e:
            print(f"⚠️ 跳过 {fname}: 缺少字段 {e}")

    return T_L_list, T_R_list, fnames


def build_relative_transforms(T_list):
    """Build relative motions between adjacent frames."""
    A_list = []
    for i in range(1, len(T_list)):
        T_prev_inv = np.linalg.inv(T_list[i - 1])
        A = T_prev_inv @ T_list[i]
        A_list.append(A)
    return A_list


def compute_residuals(A_list, B_list, T_X):
    """Compute per-pair residuals: || A X - X B ||_F.  # AI-Deep修改：诊断用"""
    residuals = []
    for A, B in zip(A_list, B_list):
        err = A @ T_X - T_X @ B
        residuals.append(np.linalg.norm(err, 'fro'))
    return np.array(residuals)


def run_calibration(T_L_list, T_R_list):
    """Run calibration with multiple methods.  # AI-Deep修改: 多方法对比"""
    A_list = build_relative_transforms(T_L_list)
    B_list = build_relative_transforms(T_R_list)

    if len(A_list) < 3:
        print(f"⚠️ 需要至少 3 组相对运动 (当前 {len(A_list)})，请采集更多样本!")
        return None, None, None, None, None

    R_A, t_A, R_B, t_B = [], [], [], []
    for A, B in zip(A_list, B_list):
        R_A.append(A[:3, :3])
        t_A.append(A[:3, 3])
        R_B.append(B[:3, :3])
        t_B.append(B[:3, 3])

    results = {}
    for name, method in METHODS.items():
        try:
            R_X, t_X = cv2.calibrateHandEye(R_A, t_A, R_B, t_B, method=method)
            T_X = np.eye(4)
            T_X[:3, :3] = R_X
            T_X[:3, 3] = t_X.flatten()
            residuals = compute_residuals(A_list, B_list, T_X)
            results[name] = {'T': T_X, 'rms': np.sqrt(np.mean(residuals**2)),
                             'residuals': residuals}
        except cv2.error as e:
            print(f"⚠️ {name} 方法失败: {e}")

    return results, A_list, B_list, T_L_list, T_R_list


def print_transform(T, label=''):
    """Print a 4x4 transformation matrix."""
    quat = R.from_matrix(T[:3, :3]).as_quat()
    euler = R.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
    trans = T[:3, 3]
    print(f'  {label}平移: [{trans[0]:.4f}, {trans[1]:.4f}, {trans[2]:.4f}] m')
    print(f'  {label}欧拉: [{euler[0]:.1f}, {euler[1]:.1f}, {euler[2]:.1f}]°')
    print(f'  {label}四元数: [{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}]')


def main():
    parser = argparse.ArgumentParser(description='双臂标定 — AX=XB 方法')
    parser.add_argument('--data_dir', default='/tmp/dual_end_tf_data/dual_poses/',
                        help='成对姿态数据目录 (默认: /tmp/dual_end_tf_data/dual_poses/)')
    args = parser.parse_args()

    print(f'加载数据: {args.data_dir}')
    T_L_list, T_R_list, fnames = load_poses_from_dir(args.data_dir)

    if not T_L_list:
        print('❌ 未找到有效数据文件')
        return

    print(f'已加载 {len(T_L_list)} 组成对姿态')
    if len(T_L_list) < 3:
        print(f'⚠️ 至少需要 3 组样本 (当前 {len(T_L_list)})，请继续采集!')
        return

    # 打印各组姿态概况  # AI-Deep修改
    print()
    print('==================== 姿态概况 ====================')
    for i, (TL, TR, fn) in enumerate(zip(T_L_list, T_R_list, fnames)):
        pL = TL[:3, 3]
        pR = TR[:3, 3]
        print(f'  {fn}: L=[{pL[0]:+.3f},{pL[1]:+.3f},{pL[2]:+.3f}]  '
              f'R=[{pR[0]:+.3f},{pR[1]:+.3f},{pR[2]:+.3f}]')
    print('==================================================')

    # 执行标定
    print()
    print('正在计算...')
    results, A_list, B_list, T_L_list, T_R_list = run_calibration(T_L_list, T_R_list)

    if not results:
        return

    # 多方法对比  # AI-Deep修改
    print()
    print('==================== 多方法对比 ====================')
    print(f'{"方法":<12} {"平移x(m)":>10} {"平移y(m)":>10} {"平移z(m)":>10} {"RMS":>8}')
    print('-' * 52)
    for name, r in results.items():
        t = r['T'][:3, 3]
        print(f'{name:<12} {t[0]:>10.4f} {t[1]:>10.4f} {t[2]:>10.4f} {r["rms"]:>8.4f}')

    # 选 RMS 最小的方法
    best_name = min(results, key=lambda k: results[k]['rms'])
    best = results[best_name]
    print()
    print(f'✅ 最优方法: {best_name} (RMS={best["rms"]:.4f})')
    print('==================================================')

    # 诊断：各组相对运动的残差  # AI-Deep修改
    print()
    print('==================== 各组偏差诊断 ====================')
    residuals = best['residuals']
    med_dev = np.median(residuals)
    threshold = max(med_dev * 3.0, 0.01)

    for i, (r, fn1, fn2) in enumerate(zip(residuals, fnames[:-1], fnames[1:])):
        flag = ' ⚠️ 异常!' if r > threshold else ''
        print(f'  运动{i} ({fn1}→{fn2}): 残差={r*1000:.2f} mm{flag}')

    if any(r > threshold for r in residuals):
        print()
        print('⚠️ 存在异常样本! 建议检查对应姿态对，删除后重新采集补充.')
        print('   异常可能原因: 双臂运动不同步 / 某臂碰歪了 / 姿态变化太小')

    print('==================================================')

    # 输出最终结果
    print()
    print('==================== 最终结果: ^{L}T_R ====================')
    print_transform(best['T'])

    # 保存结果  # AI-Deep修改
    save_path = os.path.join(os.path.dirname(args.data_dir.rstrip('/')),
                             'dual_arm_calibration_result.json')
    result = {
        'method': best_name,
        'rms': float(best['rms']),
        'L_TR': {
            'translation_m': [float(x) for x in best['T'][:3, 3]],
            'euler_xyz_deg': [float(x) for x in
                              R.from_matrix(best['T'][:3, :3]).as_euler('xyz', degrees=True)],
            'quaternion_xyzw': [float(x) for x in
                                R.from_matrix(best['T'][:3, :3]).as_quat()],
        },
        'per_motion_residuals_mm': [float(r) * 1000 for r in residuals],
        'num_samples': len(T_L_list),
    }
    with open(save_path, 'w') as f:
        json.dump(result, f, indent=2)
    print(f'\n结果已保存: {save_path}')


if __name__ == '__main__':
    main()
