import os
import json
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

def pose_to_matrix(pos, quat):
    """Convert position and quaternion to 4x4 transformation matrix."""
    T = np.eye(4)
    rot = R.from_quat([quat['x'], quat['y'], quat['z'], quat['w']])
    T[:3, :3] = rot.as_matrix()
    T[:3, 3] = [pos['x'], pos['y'], pos['z']]
    return T

def load_poses_from_dir(json_dir):
    """Load T_L and T_R from all JSON files in directory."""
    T_L_list, T_R_list = [], []
    files = sorted([f for f in os.listdir(json_dir) if f.endswith(".json")])
    for fname in files:
        with open(os.path.join(json_dir, fname), 'r') as f:
            data = json.load(f)
        T_L = pose_to_matrix(data['lend_pose']['position'], data['lend_pose']['orientation'])
        T_R = pose_to_matrix(data['rend_pose']['position'], data['rend_pose']['orientation'])
        T_L_list.append(T_L)
        T_R_list.append(T_R)
    return T_L_list, T_R_list

def build_relative_transforms(T_list):
    """Build relative motions between adjacent frames."""
    A_list = []
    for i in range(1, len(T_list)):
        T_prev_inv = np.linalg.inv(T_list[i - 1])
        A = T_prev_inv @ T_list[i]
        A_list.append(A)
    return A_list

def run_calibration(T_L_list, T_R_list):
    A_list = build_relative_transforms(T_L_list)
    B_list = build_relative_transforms(T_R_list)

    R_A, t_A, R_B, t_B = [], [], [], []
    for A, B in zip(A_list, B_list):
        R_A.append(A[:3, :3])
        t_A.append(A[:3, 3])
        R_B.append(B[:3, :3])
        t_B.append(B[:3, 3])

    R_X, t_X = cv2.calibrateHandEye(R_A, t_A, R_B, t_B, method=cv2.CALIB_HAND_EYE_TSAI)
    print("Estimated Rotation Matrix R_X:\n", R_X)
    print("Estimated Translation Vector t_X:\n", t_X)
    T_X = np.eye(4)
    T_X[:3, :3] = R_X
    T_X[:3, 3] = t_X.flatten()
    return T_X

def print_transform(T):
    print("=== Estimated ^L T_R (Right base in Left base frame) ===")
    print("4x4 Matrix:\n", T)
    quat = R.from_matrix(T[:3, :3]).as_quat()  # x, y, z, w
    print("Quaternion (x y z w):", quat)
    euler = R.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
    print("Euler XYZ (deg):", euler)
    print("Translation (x y z):", T[:3, 3])

if __name__ == "__main__":
    # 修改为你的数据文件夹路径
    json_dir = "/tmp/dual_end_tf_data/dual_poses/"
    
    T_L_list, T_R_list = load_poses_from_dir(json_dir)
    if len(T_L_list) < 3:
        print("⚠️ 至少需要3个及以上样本用于标定！")
        exit(1)

    T_X = run_calibration(T_L_list, T_R_list)
    print_transform(T_X)
