import numpy as np
from scipy.spatial.transform import Rotation as R

# 原始数据
t = np.array([-0.067, 0.799, 0.0])
yaw_deg = 130
rot = R.from_euler('z', yaw_deg, degrees=True)
R_mat = rot.as_matrix()

# 等效平移
t_prime = R_mat @ t

# 构造齐次变换矩阵
T = np.eye(4)
T[:3, :3] = R_mat
T[:3, 3] = t_prime

quat = R.from_matrix(R_mat).as_quat()  # [x, y, z, w]

# 打印
np.set_printoptions(precision=6, suppress=True)

print("=== 旋转矩阵 R (绕 Z 轴 130°) ===")
print(R_mat)

print("\n=== 原始平移向量 t ===")
print(t)

print("\n=== 等效平移向量 t' = R @ t ===")
print(t_prime)

print("\n=== 齐次变换矩阵 T ===")
print(T)

print("\n=== 四元数 Quaternion (x, y, z, w) ===")
print(quat)

print("\n=== ROS static_transform_publisher 示例 ===")
print(f"ros2 run tf2_ros static_transform_publisher \\\n"
      f"  {t_prime[0]:.6f} {t_prime[1]:.6f} {t_prime[2]:.6f} \\\n"
      f"  {quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f} \\\n"
      f"  left_base right_base")
