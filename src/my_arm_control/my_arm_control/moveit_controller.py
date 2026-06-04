"""
MoveIt 规划 + robo_ctrl 执行 — 混合控制器

控制链路
----------
MoveIt2 (fairino3_v6_planner)  规划轨迹
    → RobotServoJoint 服务       逐点发送到实机

用法
-----
    ctrl = MoveItController(node, prefix="/L")
    # 笛卡尔空间: 规划 + 执行
    ctrl.plan_and_move_cart([x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg])
    # 关节空间: 规划 + 执行
    ctrl.plan_and_move_joint([j1, j2, j3, j4, j5, j6])
"""

import time
import math
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from robo_ctrl.srv import RobotServoJoint


class MoveItController:
    """MoveIt 规划 + robo_ctrl 执行 混合控制器。"""

    # 关节角度限位 (度) — 与 high_level.cpp 一致
    JOINT_LIMITS_MIN = [-175.0, -265.0, -150.0, -265.0, -175.0, -175.0]
    JOINT_LIMITS_MAX = [175.0, 85.0, 150.0, 85.0, 175.0, 175.0]

    def __init__(
        self,
        node: Node,
        prefix: str = "/L",
        planner_service_name: str = "get_joint_states",
        servo_service_name: str = "robot_servo_joint",
    ):
        """
        初始化混合控制器。

        Args:
            node: ROS2 节点实例
            prefix: 机器人命名空间前缀 (如 "/L" 或 "/R")
            planner_service_name: fairino3_v6_planner 的 get_joint_states 服务名
            servo_service_name: robo_ctrl 的 robot_servo_joint 服务名
        """
        self._node = node
        self._prefix = prefix
        self._logger = node.get_logger().get_child("MoveItCtrl")

        # --- fairino3_v6_planner 客户端 (仅规划) ---
        # 注意: planner 服务没有命名空间，是全局服务
        self._planner_client = node.create_client(
            fairino3_v6_planner.srv.GetJointStates,
            planner_service_name,
        )

        # --- robo_ctrl RobotServoJoint 客户端 (执行) ---
        self._servo_client = node.create_client(
            RobotServoJoint,
            f"{prefix}/{servo_service_name}",
        )

        # 默认伺服参数
        self._acc = 80.0           # 加速度百分比
        self._vel = 80.0           # 速度百分比
        self._cmd_time = 0.01      # 指令周期 (秒)
        self._filter_time = -1.0   # 滤波时间 (-1 = 默认)
        self._gain = 0.0           # 位置增益

        self._logger.info(
            f"MoveItController 初始化 | prefix={prefix} | "
            f"planner={planner_service_name} | servo={prefix}/{servo_service_name}"
        )

    # =================================================================
    # 公共接口
    # =================================================================

    def plan_and_move_cart(
        self,
        pose: List[float],
        frame_id: str = "base_link",
        timeout: float = 30.0,
    ) -> bool:
        """
        笛卡尔空间: 调用 MoveIt 规划 → 通过 RobotServoJoint 执行。

        Args:
            pose: [x, y, z, rx, ry, rz] — 单位 mm, 度
            frame_id: 目标位姿参考坐标系
            timeout: 总超时 (秒)

        Returns:
            True 成功, False 失败
        """
        self._logger.info(f"━━━ plan_and_move_cart: {pose} ━━━")

        # Step 1: 调用 fairino3_v6_planner 规划
        planner_resp = self._call_planner(pose, frame_id, timeout)
        if planner_resp is None or not planner_resp.success:
            self._logger.error("MoveIt 规划失败")
            return False

        trajectory = planner_resp.trajectory_joint_states
        self._logger.info(
            f"MoveIt 规划成功, 轨迹点: {len(trajectory)}"
        )

        if len(trajectory) == 0:
            self._logger.error("轨迹点为空")
            return False

        # Step 2: 通过 RobotServoJoint 执行
        return self._execute_trajectory(trajectory, timeout)

    def plan_and_move_joint(
        self,
        joints: List[float],
        timeout: float = 30.0,
    ) -> bool:
        """
        关节空间: 直接线性插值 + RobotServoJoint 执行。

        注意: 当前 fairino3_v6_planner 只支持笛卡尔目标规划,
        关节空间暂用线性插值 (与原有 _act_j 一致),
        后续可扩展为 MoveGroup action 规划。

        Args:
            joints: 6 个关节角度 (度)
            timeout: 总超时 (秒)

        Returns:
            True 成功, False 失败
        """
        self._logger.info(f"━━━ plan_and_move_joint: {joints} ━━━")

        # 关节限位检查
        if not self._check_joint_limits(joints):
            return False

        # 生成线性插值轨迹点
        trajectory = self._generate_joint_trajectory(joints)

        # 通过 RobotServoJoint 执行
        return self._execute_trajectory(trajectory, timeout)

    def plan_and_move_incremental(
        self,
        dx: float, dy: float, dz: float,
        timeout: float = 30.0,
    ) -> bool:
        """
        增量运动 (笛卡尔增量) — 用于接近物体等场景。

        直接构造成 RobotServoJoint 请求发送增量轨迹点,
        不经过 MoveIt 规划 (增量运动不需要避障规划)。

        Args:
            dx: X 增量 (mm)
            dy: Y 增量 (mm)
            dz: Z 增量 (mm)
            timeout: 总超时 (秒)

        Returns:
            True 成功, False 失败
        """
        self._logger.info(f"━━━ plan_and_move_incremental: ({dx}, {dy}, {dz}) ━━━")

        # 构造一个简单的 JointState 消息 (增量模式)
        traj_msg = JointState()
        traj_msg.header.stamp = self._node.get_clock().now().to_msg()
        traj_msg.name = [f"joint{i+1}" for i in range(6)]
        # 增量模式: 只需要一次发送增量值
        traj_msg.position = [dx * 0.001, dy * 0.001, dz * 0.001, 0.0, 0.0, 0.0]

        return self._execute_trajectory([traj_msg], timeout, is_incremental=True)

    # =================================================================
    # 内部方法
    # =================================================================

    def _call_planner(
        self,
        pose: List[float],
        frame_id: str,
        timeout: float,
    ):
        """调用 fairino3_v6_planner 的 get_joint_states 服务。"""
        if not self._planner_client.wait_for_service(timeout_sec=5.0):
            self._logger.error("fairino3_v6_planner/get_joint_states 服务不可用")
            return None

        from fairino3_v6_planner.srv import GetJointStates

        req = GetJointStates.Request()

        # 构建 PoseStamped, 毫米→米, 度→弧度
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = self._node.get_clock().now().to_msg()
        ps.pose.position.x = pose[0] / 1000.0
        ps.pose.position.y = pose[1] / 1000.0
        ps.pose.position.z = pose[2] / 1000.0

        # 将 RPY (度) 转换为四元数
        from tf_transformations import quaternion_from_euler
        rx_r = math.radians(pose[3])
        ry_r = math.radians(pose[4])
        rz_r = math.radians(pose[5])
        q = quaternion_from_euler(rx_r, ry_r, rz_r)
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]

        req.target_pose = ps

        future = self._planner_client.call_async(req)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                self._logger.error("规划超时")
                return None
            time.sleep(0.01)

        return future.result()

    def _execute_trajectory(
        self,
        trajectory: List[JointState],
        timeout: float = 30.0,
        is_incremental: bool = False,
    ) -> bool:
        """
        通过 RobotServoJoint 服务执行轨迹。

        协议:
        1. ServoMoveStart (command_type=0) + 所有轨迹点
        2. ServoMoveEnd   (command_type=1)
        """
        if not self._servo_client.wait_for_service(timeout_sec=5.0):
            self._logger.error("RobotServoJoint 服务不可用")
            return False

        # Step 1: ServoMoveStart
        start_req = RobotServoJoint.Request()
        start_req.command_type = 0
        start_req.joint_positions = list(trajectory)
        start_req.acc = self._acc
        start_req.vel = self._vel
        start_req.cmd_time = self._cmd_time
        start_req.filter_time = self._filter_time
        start_req.gain = self._gain
        start_req.use_incremental = is_incremental

        start_resp = self._call_service(
            self._servo_client, start_req, timeout, "ServoMoveStart"
        )
        if start_resp is None or not start_resp.success:
            self._logger.error(
                f"ServoMoveStart 失败: {start_resp.message if start_resp else '超时'}"
            )
            return False

        self._logger.info("ServoMoveStart 成功, 等待执行完成...")

        # 等待运动完成
        time.sleep(1.0)

        # Step 2: ServoMoveEnd
        end_req = RobotServoJoint.Request()
        end_req.command_type = 1
        end_req.use_incremental = is_incremental

        end_resp = self._call_service(
            self._servo_client, end_req, timeout, "ServoMoveEnd"
        )
        if end_resp is None or not end_resp.success:
            self._logger.warn(
                f"ServoMoveEnd 失败: {end_resp.message if end_resp else '超时'}"
            )
            # 不返回 False, 运动可能已经完成

        self._logger.info("✓ 轨迹执行完成")
        return True

    def _generate_joint_trajectory(
        self,
        target_joints: List[float],
        num_points: int = 100,
        use_incremental: bool = False,
    ) -> List[JointState]:
        """生成关节空间的线性插值轨迹点 (与 high_level.cpp 逻辑一致)。"""
        trajectory = []

        for i in range(num_points):
            js = JointState()
            js.header.stamp = self._node.get_clock().now().to_msg()
            js.name = [f"joint{i+1}" for i in range(6)]
            js.position = [0.0] * 6

            if use_incremental:
                # 增量模式: 每个点包含增量值 / point_count
                for j in range(6):
                    js.position[j] = target_joints[j] / num_points
            else:
                # 绝对模式: 线性插值从 0 → target
                for j in range(6):
                    js.position[j] = (i + 1) * target_joints[j] / num_points

            trajectory.append(js)

        return trajectory

    def _check_joint_limits(self, joints: List[float]) -> bool:
        """检查关节角度是否在限位范围内。"""
        if len(joints) != 6:
            self._logger.error(f"关节角度数组长度必须为6, 实际={len(joints)}")
            return False

        for i in range(6):
            if joints[i] < self.JOINT_LIMITS_MIN[i] or \
               joints[i] > self.JOINT_LIMITS_MAX[i]:
                self._logger.error(
                    f"关节{i+1}角度 {joints[i]:.1f}° 超出限位 "
                    f"[{self.JOINT_LIMITS_MIN[i]}, {self.JOINT_LIMITS_MAX[i]}]"
                )
                return False
        return True

    def _call_service(self, client, request, timeout, name):
        """异步调用服务并等待结果。"""
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                return None
            time.sleep(0.01)
        return future.result()

    # =================================================================
    # 参数配置
    # =================================================================

    def set_servo_params(
        self,
        acc: float = 80.0,
        vel: float = 80.0,
        cmd_time: float = 0.01,
        filter_time: float = -1.0,
        gain: float = 0.0,
    ):
        """配置 RobotServoJoint 参数。"""
        self._acc = acc
        self._vel = vel
        self._cmd_time = cmd_time
        self._filter_time = filter_time
        self._gain = gain
        self._logger.info(
            f"伺服参数更新: acc={acc} vel={vel} "
            f"cmd_time={cmd_time} filter_time={filter_time} gain={gain}"
        )
