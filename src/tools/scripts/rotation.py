# AI-Deep: 更新为双臂标定真实结果 (dual_calib_data/dual_arm_result.json, 4组, RMS=2.47mm)
# 此脚本用于验证 ^{L}T_R 齐次变换矩阵的计算
import numpy as np
from scipy.spatial.transform import Rotation as R

# 双臂基座标定结果: Lrobot_base → Rrobot_base
t = np.array([-0.09815, 1.05515, -0.01051])  # m
euler = np.array([0.869, -1.130, 34.878])    # deg (xyz)
rot = R.from_euler('xyz', euler, degrees=True)
R_mat = rot.as_matrix()

# 构造齐次变换矩阵
T = np.eye(4)
T[:3, :3] = R_mat
T[:3, 3] = t

quat = R.from_matrix(R_mat).as_quat()  # [x, y, z, w]

# 打印
np.set_printoptions(precision=6, suppress=True)

print("=== 双臂基座标定: ^{L}T_R ===")
print(f"数据来源: dual_calib_data/dual_arm_result.json (4组, RMS=2.47mm)")

print("\n=== 平移向量 t (m) ===")
print(t)

print("\n=== 欧拉角 (xyz, deg) ===")
print(euler)

print("\n=== 旋转矩阵 R ===")
print(R_mat)

print("\n=== 齐次变换矩阵 T ===")
print(T)

print("\n=== 四元数 Quaternion (x, y, z, w) ===")
print(quat)

print("\n=== ROS static_transform_publisher 示例 ===")
print(f"ros2 run tf2_ros static_transform_publisher \\\n"
      f"  {t[0]:.6f} {t[1]:.6f} {t[2]:.6f} \\\n"
      f"  {quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f} \\\n"
      f"  Lrobot_base Rrobot_base")

# 验证: 把R坐标系下的点转换到L坐标系
print("\n=== 验证示例 ===")
p_R = np.array([0.5, 0.0, 0.3])  # R基座坐标系下的一个点
p_L = R_mat @ p_R + t
print(f"点 p_R = {p_R}")
print(f"变换后 p_L = {p_L}")
