import os
import json
import numpy as np
from scipy.spatial.transform import Rotation as R

# ✅ 替换为你的实际路径
pose_dir = "/home/phoenix/roboarm/FairinoDualArm/tmp/eye_hand_calibration_dataL/poses"

def pose_to_rt(pose):
    quat = pose["orientation"]
    pos = pose["position"]
    r = R.from_quat([quat["x"], quat["y"], quat["z"], quat["w"]])
    R_mat = r.as_matrix()
    t_vec = np.array([[pos["x"]], [pos["y"]], [pos["z"]]])
    return R_mat, t_vec

def calibrate_tcp_offset(pose_data):
    if len(pose_data) < 4:
        raise ValueError("至少需要4个姿态进行TCP标定")
    R0, t0 = pose_to_rt(pose_data[0])
    A = []
    b = []
    for i in range(1, len(pose_data)):
        Ri, ti = pose_to_rt(pose_data[i])
        A.append(R0 - Ri)
        b.append(ti - t0)
    A = np.vstack(A)
    b = np.vstack(b)
    tcp_offset, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    return tcp_offset.flatten()

# ✅ 加载所有 JSON 文件
pose_files = sorted([f for f in os.listdir(pose_dir) if f.endswith(".json")])
pose_list = []
for filename in pose_files:
    with open(os.path.join(pose_dir, filename), "r") as f:
        pose = json.load(f)
        pose_list.append(pose)

# ✅ 执行标定
if len(pose_list) >= 4:
    tcp_tip_offset = calibrate_tcp_offset(pose_list)
    print("TCP 坐标系下夹爪尖端的位置（单位：米）:")
    print(f"x: {tcp_tip_offset[0]:.6f}, y: {tcp_tip_offset[1]:.6f}, z: {tcp_tip_offset[2]:.6f}")
else:
    print(f"仅有 {len(pose_list)} 个样本，无法进行标定（至少需要 4 个）")
