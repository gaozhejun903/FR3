"""
任务节点 — 视觉检测 → 机械臂移动 → 夹爪抓取 → 放置复位

双后端支持:
  - use_moveit=True  → MoveIt Action Client (仿真, 用于 fairino3_v6_moveit2_config demo)
  - use_moveit=False → robo_ctrl 服务接口 (实机)

状态机:
  IDLE -> OBSERVATION -> APPROACHING -> GRABBING -> RETREATING -> PLACING -> IDLE
"""

import time
import math
import threading
from enum import Enum
from typing import Optional, List, Dict

import rclpy
from rclpy.node import Node


class TaskState(Enum):
    IDLE = "IDLE"
    OBSERVATION = "OBSERVATION"
    PLANNING = "PLANNING"
    APPROACHING = "APPROACHING"
    GRABBING = "GRABBING"
    RETREATING = "RETREATING"
    PLACING = "PLACING"
    ERROR = "ERROR"


# =================================================================
# 检测滤波: Kalman 滤波 + 目标跟踪
# =================================================================
class _Kalman1D:
    """单轴 Kalman 滤波器 (匀速模型)。"""

    def __init__(self, process_noise: float = 0.01, measurement_noise: float = 0.05):
        self.x = 0.0       # 状态: [位置, 速度]
        self.v = 0.0
        self.P = [[1.0, 0.0], [0.0, 1.0]]  # 协方差
        self.Q = process_noise      # 过程噪声
        self.R = measurement_noise  # 测量噪声
        self.initialized = False

    def update(self, z: float) -> float:
        """输入测量值, 返回滤波后的位置。"""
        if not self.initialized:
            self.x = z
            self.v = 0.0
            self.P = [[1.0, 0.0], [0.0, 1.0]]
            self.initialized = True
            return self.x

        # 预测
        x_pred = self.x + self.v
        P00 = self.P[0][0] + self.P[1][0] + self.P[0][1] + self.P[1][1] + self.Q
        P01 = self.P[0][1] + self.P[1][1]
        P10 = self.P[1][0] + self.P[1][1]
        P11 = self.P[1][1] + self.Q

        # 更新
        S = P00 + self.R
        K0 = P00 / S
        K1 = P10 / S
        residual = z - x_pred
        self.x = x_pred + K0 * residual
        self.v = self.v + K1 * residual
        self.P[0][0] = (1.0 - K0) * P00
        self.P[0][1] = (1.0 - K0) * P01
        self.P[1][0] = P10 - K1 * P00
        self.P[1][1] = P11 - K1 * P01

        return self.x


class TrackedObject:
    """跟踪的目标: Kalman 滤波 + 帧计数。"""

    def __init__(self, class_id: int, pos_mm: list,
                 process_noise: float = 0.01,
                 measurement_noise: float = 0.05):
        self.class_id = class_id
        self.kalman_x = _Kalman1D(process_noise, measurement_noise)
        self.kalman_y = _Kalman1D(process_noise, measurement_noise)
        self.kalman_z = _Kalman1D(process_noise, measurement_noise)
        self.pos_mm = list(pos_mm)  # 滤波后的位置
        self.age = 0          # 距离上次更新的帧数
        self.valid_count = 1  # 有效检测次数
        self._update(pos_mm)

    def _update(self, pos_mm: list):
        self.pos_mm[0] = self.kalman_x.update(pos_mm[0])
        self.pos_mm[1] = self.kalman_y.update(pos_mm[1])
        self.pos_mm[2] = self.kalman_z.update(pos_mm[2])

    def distance_to(self, pos_mm: list) -> float:
        dx = self.pos_mm[0] - pos_mm[0]
        dy = self.pos_mm[1] - pos_mm[1]
        dz = self.pos_mm[2] - pos_mm[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)


class ArmTaskManager(Node):
    """任务管理节点, 支持 MoveIt (仿真) 和 robo_ctrl (实机) 双后端。"""

    def __init__(self):
        super().__init__("arm_task_manager")

        # ====== 参数 ====================================================
        self.declare_parameter("use_moveit", False)
        self._use_moveit = self.get_parameter("use_moveit").value

        # 机器人命名空间前缀 (实机模式)
        self.declare_parameter("robot_prefix", "/L")

        # 观测位姿
        #   仿真模式: 单位=米 (MoveIt)
        #   实机模式: 单位=毫米 (robo_ctrl)
        self.declare_parameter("observe_x", 0.4 if self._use_moveit else 99.917)
        self.declare_parameter("observe_y", 0.0 if self._use_moveit else -144.210)
        self.declare_parameter("observe_z", 0.5 if self._use_moveit else 542.554)
        self.declare_parameter("observe_rx", 0.0 if self._use_moveit else -125.357)
        self.declare_parameter("observe_ry", 0.0)
        self.declare_parameter("observe_rz", 0.0 if self._use_moveit else -100.476)

        # 放置位姿
        self.declare_parameter("place_x", 0.1 if self._use_moveit else 200.0)
        self.declare_parameter("place_y", -0.3 if self._use_moveit else -300.0)
        self.declare_parameter("place_z", 0.2 if self._use_moveit else 200.0)
        self.declare_parameter("place_rx", 0.0 if self._use_moveit else -90.0)
        self.declare_parameter("place_ry", 0.0)
        self.declare_parameter("place_rz", 0.0 if self._use_moveit else -90.0)

        # MoveIt 专用参数
        self.declare_parameter("planning_group", "fairino3_v6_group")
        self.declare_parameter("end_effector_link", "wrist3_link")
        self.declare_parameter("reference_frame", "base_link")
        self.declare_parameter("planning_time", 5.0)
        self.declare_parameter("planning_attempts", 10)

        # 高度参数 (mm) — 用于计算抓取 z 轴目标
        self.declare_parameter("desk_height", 360.0)
        self.declare_parameter("object_height", 89.0)

        # 预设关节角度 (度) — 拧瓶盖
        self.declare_parameter("cap_open_joints_l", [-55.0, -90.0, -120.0, 30.0, 81.272, 0.0])
        self.declare_parameter("cap_open_joints_r", [45.434, -124.551, 128.388, -184.270, 19.218, 0.0])

        # 预设关节角度 (度) — 接球
        self.declare_parameter("ball_L_joint_init_pose", [0.0, -66.994, -51.016, -87.069, 81.874, -90.0])
        self.declare_parameter("ball_R_joint_init_pose", [66.251, -123.403, 126.849, -184.040, -97.664, 90.0])
        self.declare_parameter("ball_L_1_joint_pose", [-18.158, -127.475, -80.388, 28.213, 115.327, -90.0])
        self.declare_parameter("ball_R_per_1_joint_pose", [108.784, -48.573, 65.323, -194.233, -129.820, 87.784])
        self.declare_parameter("ball_R_1_joint_pose", [103.339, -23.279, 22.041, -180.788, -126.890, 90.0])
        self.declare_parameter("ball_L_2_joint_pose", [-29.826, -136.002, -50.200, -3.902, 115.484, -89.971])
        self.declare_parameter("ball_R_2_joint_pose", [93.138, -52.470, 69.132, -190.739, -123.491, 89.984])

        # TCP 到可乐偏移 (mm) — 接近可乐时的 x/y 校准
        self.declare_parameter("cola_offset_x", -132.0)
        self.declare_parameter("cola_offset_y", 45.0)

        # 夹爪参数
        self.declare_parameter("gripper_port", "/dev/ttyACM0")
        self.declare_parameter("gripper_slave_id", 9)
        self.declare_parameter("approach_offset_z", 150.0)
        self.declare_parameter("retreat_z", 80.0)

        # 运动参数 (实机模式)
        self.declare_parameter("velocity", 50.0)
        self.declare_parameter("acceleration", 50.0)

        # MoveIt 运动参数
        self.declare_parameter("velocity_scale", 0.4)
        self.declare_parameter("acceleration_scale", 0.4)

        # 检测参数
        self.declare_parameter("target_class_id", -1)
        self.declare_parameter("detection_timeout", 30.0)

        # 检测滤波参数
        self.declare_parameter("distance_threshold", 150.0)  # mm, 同一目标合并距离
        self.declare_parameter("age_threshold", 5)            # 帧, 超过则丢弃
        self.declare_parameter("valid_threshold", 1)          # 最低有效检测次数
        self.declare_parameter("kalman_process_noise", 0.01)
        self.declare_parameter("kalman_measurement_noise", 0.05)

        # 任务模式: "grab"=单臂抓取, "opencap"=拧瓶盖+倒可乐, "ball"=接球
        self.declare_parameter("task_mode", "grab")

        # ====== 读取参数 ================================================
        prefix = self.get_parameter("robot_prefix").value
        self._observe_pose = [
            self.get_parameter("observe_x").value,
            self.get_parameter("observe_y").value,
            self.get_parameter("observe_z").value,
            self.get_parameter("observe_rx").value,
            self.get_parameter("observe_ry").value,
            self.get_parameter("observe_rz").value,
        ]
        self._place_pose = [
            self.get_parameter("place_x").value,
            self.get_parameter("place_y").value,
            self.get_parameter("place_z").value,
            self.get_parameter("place_rx").value,
            self.get_parameter("place_ry").value,
            self.get_parameter("place_rz").value,
        ]
        self._desk_height = self.get_parameter("desk_height").value
        self._object_height = self.get_parameter("object_height").value
        self._gripper_id = self.get_parameter("gripper_slave_id").value

        # 预设关节角度
        self._cap_open_joints_l = self.get_parameter("cap_open_joints_l").value
        self._cap_open_joints_r = self.get_parameter("cap_open_joints_r").value
        self._ball_L_joint_init_pose = self.get_parameter("ball_L_joint_init_pose").value
        self._ball_R_joint_init_pose = self.get_parameter("ball_R_joint_init_pose").value
        self._ball_L_1_joint_pose = self.get_parameter("ball_L_1_joint_pose").value
        self._ball_R_per_1_joint_pose = self.get_parameter("ball_R_per_1_joint_pose").value
        self._ball_R_1_joint_pose = self.get_parameter("ball_R_1_joint_pose").value
        self._ball_L_2_joint_pose = self.get_parameter("ball_L_2_joint_pose").value
        self._ball_R_2_joint_pose = self.get_parameter("ball_R_2_joint_pose").value

        # TCP 到可乐偏移
        self._cola_offset_x = self.get_parameter("cola_offset_x").value
        self._cola_offset_y = self.get_parameter("cola_offset_y").value
        self._approach_offset_z = self.get_parameter("approach_offset_z").value
        self._retreat_z = self.get_parameter("retreat_z").value
        self._target_class_id = self.get_parameter("target_class_id").value
        self._detection_timeout = self.get_parameter("detection_timeout").value
        self._task_mode = self.get_parameter("task_mode").value

        # 检测滤波参数
        self._distance_threshold = self.get_parameter("distance_threshold").value
        self._age_threshold = self.get_parameter("age_threshold").value
        self._valid_threshold = self.get_parameter("valid_threshold").value
        self._kalman_pn = self.get_parameter("kalman_process_noise").value
        self._kalman_mn = self.get_parameter("kalman_measurement_noise").value

        # ====== 状态 ====================================================
        self._state = TaskState.IDLE
        self._tracked_objects: List[TrackedObject] = []
        self._services_ready = False
        self._lock = threading.Lock()
        self._detection_paused = False

        # 实机模式需要的 TCP 状态
        self._current_tcp = None
        self._R_current_tcp = None
        self._right_arm_ready = False

        # ====== 根据模式初始化后端 ======================================
        if self._use_moveit:
            self._init_moveit_backend()
        else:
            self._init_robo_ctrl_backend(prefix)

        # ====== 视觉订阅 ================================================
        if self._use_moveit:
            # 仿真: 订阅 target_pose (来自 virtual_vision_node)
            from geometry_msgs.msg import PoseStamped
            self._pose_sub = self.create_subscription(
                PoseStamped, "target_pose", self._target_pose_cb, 10
            )
            self.get_logger().info("仿真模式: 已订阅 target_pose")
        else:
            # 实机: 订阅 bbox3d (来自 depth_handler)
            from depth_handler.msg import Bbox3dArray
            from robo_ctrl.msg import RobotState
            self._bbox_sub = self.create_subscription(
                Bbox3dArray, "/depth_handler/bbox3d", self._bbox_cb, 10
            )
            self._state_sub = self.create_subscription(
                RobotState, f"{prefix}/robot_state", self._robot_state_cb, 10
            )
            self._R_state_sub = self.create_subscription(
                RobotState, "/R/robot_state", self._R_robot_state_cb, 10
            )
            self.get_logger().info("实机模式: 已订阅 /depth_handler/bbox3d + 左右臂状态")

        # ====== 后台初始化线程 ==========================================
        self._init_thread = threading.Thread(target=self._init_services, daemon=True)
        self._init_thread.start()

        mode = "MoveIt(仿真)" if self._use_moveit else "robo_ctrl(实机)"
        self.get_logger().info(
            f"ArmTaskManager 启动 | mode={mode} | task={self._task_mode} | "
            f"gripper_id={self._gripper_id} | "
            f"target_class_id={self._target_class_id}"
        )

    # =================================================================
    # MoveIt 后端初始化
    # =================================================================
    def _init_moveit_backend(self):
        from rclpy.action import ActionClient
        from moveit_msgs.action import MoveGroup
        from geometry_msgs.msg import PoseStamped, Vector3
        from shape_msgs.msg import SolidPrimitive
        from moveit_msgs.msg import (
            Constraints, PositionConstraint, OrientationConstraint,
            RobotState as MoveItRobotState, BoundingVolume,
        )

        self._planning_group = self.get_parameter("planning_group").value
        self._ee_link = self.get_parameter("end_effector_link").value
        self._ref_frame = self.get_parameter("reference_frame").value

        self._move_group_client = ActionClient(self, MoveGroup, "/move_action")

        # 夹爪: 串口直连
        try:
            from my_arm_control.gripper_controller import GripperController
            port = self.get_parameter("gripper_port").value
            if port:
                self._gripper_ctrl = GripperController(port=port, slave_id=self._gripper_id)
            else:
                self._gripper_ctrl = None
        except Exception as e:
            self.get_logger().warn(f"夹爪控制器初始化失败: {e}")
            self._gripper_ctrl = None

    # =================================================================
    # robo_ctrl 后端初始化
    # =================================================================
    def _init_robo_ctrl_backend(self, prefix):
        from robo_ctrl.srv import RobotMoveCart, RobotAct, RobotActJ
        from epg50_gripper_ros.srv import GripperCommand

        self._velocity = self.get_parameter("velocity").value
        self._accel = self.get_parameter("acceleration").value

        # 主臂 (左臂) 客户端
        self._move_cart_client = self.create_client(
            RobotMoveCart, f"{prefix}/robot_move_cart"
        )
        self._robot_act_client = self.create_client(
            RobotAct, f"{prefix}/robot_act"
        )
        self._robot_act_j_client = self.create_client(
            RobotActJ, f"{prefix}/robot_act_j"
        )

        # 右臂客户端 (双臂协调用)
        self._R_move_cart_client = self.create_client(
            RobotMoveCart, "/R/robot_move_cart"
        )
        self._R_robot_act_client = self.create_client(
            RobotAct, "/R/robot_act"
        )
        self._R_robot_act_j_client = self.create_client(
            RobotActJ, "/R/robot_act_j"
        )

        self._gripper_client = self.create_client(
            GripperCommand, "gripper_command"
        )
        self._R_gripper_client = self.create_client(
            GripperCommand, "R_gripper_command"
        )

    # =================================================================
    # 等待服务就绪
    # =================================================================
    def _init_services(self):
        self.get_logger().info("等待服务就绪...")

        if self._use_moveit:
            if not self._move_group_client.wait_for_server(timeout_sec=30.0):
                self.get_logger().error("MoveGroup action server 不可用 (30s 超时)")
                return
            self.get_logger().info(
                f"MoveGroup 就绪 | 组={self._planning_group} "
                f"末端={self._ee_link}"
            )
        else:
            prefix = self.get_parameter("robot_prefix").value
            required = [
                (self._move_cart_client, f"{prefix}/robot_move_cart"),
                (self._robot_act_client, f"{prefix}/robot_act"),
                (self._robot_act_j_client, f"{prefix}/robot_act_j"),
            ]
            for client, name in required:
                if not client.wait_for_service(timeout_sec=30.0):
                    self.get_logger().error(f"服务 {name} 不可用 (30s 超时)")
                    return

            # 右臂客户端 (可选, 5s 超时)
            right_clients = [
                (self._R_move_cart_client, "/R/robot_move_cart"),
                (self._R_robot_act_client, "/R/robot_act"),
                (self._R_robot_act_j_client, "/R/robot_act_j"),
            ]
            self._right_arm_ready = True
            for client, name in right_clients:
                if not client.wait_for_service(timeout_sec=5.0):
                    self.get_logger().warn(f"右臂服务 {name} 不可用, 双臂功能受限")
                    self._right_arm_ready = False
                    break

            # 夹爪服务可选 (左臂)
            if not self._gripper_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn("左臂夹爪服务不可用, 跳过左臂夹爪控制")
                self._gripper_client = None

            # 夹爪服务可选 (右臂)
            if not self._R_gripper_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn("右臂夹爪服务不可用, 跳过右臂夹爪控制")
                self._R_gripper_client = None

        self._services_ready = True
        self.get_logger().info("服务就绪, 开始任务流程")
        self._start_task_flow()

    # =================================================================
    # 回调
    # =================================================================
    def _target_pose_cb(self, msg):
        """仿真模式: 收到 target_pose 立即执行任务。"""
        if self._state != TaskState.IDLE:
            self.get_logger().info(f"当前繁忙 ({self._state.value}), 忽略目标")
            return
        self.get_logger().info(
            f"收到目标位姿: ({msg.pose.position.x:.3f}, "
            f"{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})"
        )
        self._pending_pose = msg
        self._start_task_flow()

    def _bbox_cb(self, msg):
        """实机模式: 更新检测到的物体列表 (坐标转 mm, Kalman 滤波)。"""
        if self._detection_paused:
            return

        with self._lock:
            # 所有已有目标帧计数 +1
            for obj in self._tracked_objects:
                obj.age += 1

            for bbox in msg.results:
                if self._target_class_id >= 0 and bbox.class_id != self._target_class_id:
                    continue
                cx = (bbox.x + bbox.width / 2.0) * 1000.0
                cy = (bbox.y + bbox.height / 2.0) * 1000.0
                cz = (bbox.z + bbox.depth / 2.0) * 1000.0
                pos = [cx, cy, cz]

                # 距离关联: 找最近的已有目标
                best_obj = None
                best_dist = float("inf")
                for obj in self._tracked_objects:
                    if obj.class_id != bbox.class_id:
                        continue
                    d = obj.distance_to(pos)
                    if d < best_dist:
                        best_dist = d
                        best_obj = obj

                if best_obj is not None and best_dist < self._distance_threshold:
                    # 匹配到已有目标 → Kalman 更新
                    best_obj._update(pos)
                    best_obj.age = 0
                    best_obj.valid_count += 1
                else:
                    # 新目标
                    self._tracked_objects.append(
                        TrackedObject(bbox.class_id, pos,
                                      self._kalman_pn, self._kalman_mn)
                    )

            # 丢弃过期或无效目标
            self._tracked_objects = [
                obj for obj in self._tracked_objects
                if obj.age < self._age_threshold and obj.valid_count >= self._valid_threshold
            ]

    def _robot_state_cb(self, msg):
        """实机模式: 更新左臂当前 TCP 位姿。"""
        self._current_tcp = msg.tcp_pose

    def _R_robot_state_cb(self, msg):
        """实机模式: 更新右臂当前 TCP 位姿。"""
        self._R_current_tcp = msg.tcp_pose

    # =================================================================
    # 任务流程
    # =================================================================
    def _start_task_flow(self):
        if self._task_mode == "opencap":
            thread = threading.Thread(target=self._task_opencap, daemon=True)
        else:
            thread = threading.Thread(target=self._task_flow, daemon=True)
        thread.start()

    def _task_flow(self):
        """完整抓取流程: 观测 → 检测 → 接近 → 抓取 → 撤离 → 放置。"""
        try:
            # 1. 移动到观测位姿
            self._set_state(TaskState.OBSERVATION)
            if not self._move_to_pose(self._observe_pose, phase="前往观测位"):
                self._set_state(TaskState.ERROR)
                return

            # 2. 获取目标
            if self._use_moveit:
                # 仿真: 直接使用收到的 target_pose
                target_pose = getattr(self, '_pending_pose', None)
                if target_pose is None:
                    self.get_logger().error("无目标位姿")
                    self._set_state(TaskState.ERROR)
                    return
                target_pos = [
                    target_pose.pose.position.x,
                    target_pose.pose.position.y,
                    target_pose.pose.position.z,
                ]
            else:
                # 实机: 等待 bbox3d 检测
                target = self._wait_for_detection()
                if target is None:
                    self.get_logger().error("未检测到目标")
                    self._set_state(TaskState.ERROR)
                    return
                target_pos = target["pos_mm"]
                self.get_logger().info(
                    f"检测到目标: class_id={target['class_id']} "
                    f"pos=({target_pos[0]:.1f}, {target_pos[1]:.1f}, {target_pos[2]:.1f}) mm"
                )

            # 3. 接近目标
            self._set_state(TaskState.APPROACHING)
            if not self._approach_object(target_pos):
                self.get_logger().error("接近目标失败")
                self._set_state(TaskState.ERROR)
                return

            # 4. 抓取
            self._set_state(TaskState.GRABBING)
            self._close_gripper()
            time.sleep(0.5)

            # 5. 撤离
            self._set_state(TaskState.RETREATING)
            if self._use_moveit:
                self._move_to_pose([0, 0, self._retreat_z, 0, 0, 0], phase="撤离", incremental=True)
            else:
                self._move_to_pose([0, 0, self._retreat_z, 0, 0, 0], phase="撤离", incremental=True)

            # 6. 放置
            self._set_state(TaskState.PLACING)
            if not self._move_to_pose(self._place_pose, phase="前往放置位"):
                self._set_state(TaskState.ERROR)
                return

            # 7. 释放
            self._open_gripper()
            time.sleep(0.5)

            self.get_logger().info("==== 任务完成 ====")
            self._set_state(TaskState.IDLE)

        except Exception as e:
            self.get_logger().error(f"任务异常: {e}")
            self._set_state(TaskState.ERROR)
            time.sleep(3)
            self._set_state(TaskState.IDLE)

    # =================================================================
    # 任务一: 拧瓶盖 + 倒可乐
    # =================================================================
    def _task_opencap(self):
        """拧瓶盖任务流程 (对应 dualarm main.cpp)。

        流程:
          1. L arm → 观测位
          2. R arm → CAP_OPEN_JOINTS_R (预备位)
          3. L arm 姿态修正 → rx=-90, ry=0, rz=-90
          4. 等待检测可乐 (class_id=1)
          5. L arm → 接近可乐 (增量, 带偏移)
          6. L 夹爪夹紧
          7. L arm → 撤离 (-150x, +30z)
          8. L arm → CAP_OPEN_JOINTS_L (瓶盖准备位)
          9. R arm → CAP_OPEN_JOINTS_R (瓶盖位)
          10. 拧瓶盖循环 (3轮): R夹 → 圆弧60° → R松 → 回位
          11. 最后转30°
          12. 倒可乐: L J6 旋转
          13. 放回杯子
        """
        GRIPPER_ID_L = self._gripper_id
        GRIPPER_ID_R = 10

        # opencap 任务强制检测可乐 (class_id=1)
        orig_class_id = self._target_class_id
        self._target_class_id = 1

        try:
            # ---- Step 0: 夹爪使能 ----
            self.get_logger().info("[拧瓶盖] Step 0: 夹爪使能")
            self._enable_gripper(GRIPPER_ID_L)
            self._enable_gripper(GRIPPER_ID_R)
            time.sleep(1.0)

            # ---- Step 0.5: 伺服模式启动 ----
            self.get_logger().info("[拧瓶盖] Step 0.5: 伺服模式启动")
            self._servo_move_start(side="L")
            self._servo_move_start(side="R")
            time.sleep(1.0)

            # ---- Step 1: L arm → 观测位 ----
            self._set_state(TaskState.OBSERVATION)
            self.get_logger().info("[拧瓶盖] Step 1: 左臂 → 观测位")
            # 先尝试关节运动 (更可靠), 失败再用笛卡尔
            obs_joints = self._cap_open_joints_l  # 使用瓶盖准备位作为初始关节角
            if not self._robo_ctrl_act_j(obs_joints, side="L"):
                self.get_logger().warn("关节运动失败, 尝试笛卡尔运动")
                if not self._robo_ctrl_move_cart(self._observe_pose, side="L"):
                    self._set_state(TaskState.ERROR)
                    return
            time.sleep(2.0)

            # ---- Step 2: R arm → 预备位 ----
            self.get_logger().info("[拧瓶盖] Step 2: 右臂 → 预备位")
            if not self._robo_ctrl_act_j(self._cap_open_joints_r, side="R"):
                self.get_logger().warn("右臂预备位移动失败, 继续...")
            time.sleep(2.0)

            # ---- Step 3: L arm 姿态修正 ----
            # 关节运动已到达正确位姿, 跳过姿态修正 (姿态差值过大时 MoveCart 会超时)
            self.get_logger().info("[拧瓶盖] Step 3: 跳过姿态修正 (关节运动已到位)")

            # ---- Step 4: 等待检测可乐 ----
            self._set_state(TaskState.OBSERVATION)
            self.get_logger().info("[拧瓶盖] Step 4: 等待检测可乐 (class_id=1)")
            target = self._wait_for_detection()
            if target is None:
                self.get_logger().error("未检测到可乐")
                self._set_state(TaskState.ERROR)
                return
            cola_pos = target["pos_mm"]
            self.get_logger().info(
                f"可乐位置: ({cola_pos[0]:.1f}, {cola_pos[1]:.1f}, {cola_pos[2]:.1f}) mm"
            )

            # 禁用物体更新 (抓取过程中目标不变)
            self._detection_paused = True
            self.get_logger().info("检测已暂停, 抓取过程中目标锁定")
            time.sleep(0.5)

            # ---- Step 5: L arm → 接近可乐 ----
            self._set_state(TaskState.APPROACHING)
            self.get_logger().info("[拧瓶盖] Step 5: 左臂 → 接近可乐")
            if self._current_tcp is None:
                self.get_logger().error("左臂 TCP 未知")
                self._set_state(TaskState.ERROR)
                return

            dx = cola_pos[0] + self._cola_offset_x - self._current_tcp.x
            dy = cola_pos[1] + self._cola_offset_y - self._current_tcp.y
            dz = self._desk_height + self._object_height - self._current_tcp.z
            self.get_logger().info(f"接近增量: dx={dx:.1f} dy={dy:.1f} dz={dz:.1f}")
            if not self._robo_ctrl_act_incremental(dx, dy, dz):
                self.get_logger().error("接近可乐失败")
                self._set_state(TaskState.ERROR)
                return
            time.sleep(2.5)

            # ---- Step 6: L 夹爪夹紧 ----
            self._set_state(TaskState.GRABBING)
            self.get_logger().info("[拧瓶盖] Step 6: 左臂夹紧")
            self._close_gripper(gripper_id=GRIPPER_ID_L)
            time.sleep(0.5)

            # ---- Step 7: L arm 撤离 ----
            self._set_state(TaskState.RETREATING)
            self.get_logger().info("[拧瓶盖] Step 7: 左臂撤离")
            self._robo_ctrl_move_cart([-150, 0, 30, 0, 0, 0], is_incremental=True, side="L")
            time.sleep(2.0)

            # 恢复检测 (撤离后可以更新目标)
            self._detection_paused = False

            # ---- Step 8: L arm → 瓶盖准备位 ----
            self.get_logger().info("[拧瓶盖] Step 8: 左臂 → 瓶盖准备位")
            self._robo_ctrl_act_j(self._cap_open_joints_l, side="L")
            time.sleep(2.0)

            # ---- Step 9: R arm → 瓶盖位 ----
            self.get_logger().info("[拧瓶盖] Step 9: 右臂 → 瓶盖位")
            self._robo_ctrl_act_j(self._cap_open_joints_r, side="R")
            time.sleep(2.0)

            # ---- Step 10: 拧瓶盖循环 (3轮) ----
            self._set_state(TaskState.APPROACHING)
            for i in range(3):
                self.get_logger().info(f"[拧瓶盖] Step 10: 拧瓶盖 第{i+1}/3轮")

                # R 夹爪夹紧
                self._close_gripper(gripper_id=GRIPPER_ID_R)
                time.sleep(0.5)

                # R 圆弧旋转 60°
                self._robo_ctrl_arc(
                    circle_center=[155, 0, 0], radian=60.0,
                    side="R", point_count=200, message_time=0.006
                )
                time.sleep(3.0)

                # R 夹爪松开
                self._open_gripper(gripper_id=GRIPPER_ID_R)
                time.sleep(0.5)

                # R 回到瓶盖位
                self._robo_ctrl_act_j(self._cap_open_joints_r, side="R")
                time.sleep(2.0)

            # ---- Step 11: 最后转30° ----
            self.get_logger().info("[拧瓶盖] Step 11: 最后转30°")
            self._close_gripper(gripper_id=GRIPPER_ID_R)
            time.sleep(0.5)
            self._robo_ctrl_arc(
                circle_center=[155, 0, 0], radian=30.0,
                side="R", point_count=150, message_time=0.008
            )
            time.sleep(2.5)
            self._open_gripper(gripper_id=GRIPPER_ID_R)
            time.sleep(0.5)

            # ---- Step 12: 倒可乐 ----
            self._set_state(TaskState.PLACING)
            self.get_logger().info("[拧瓶盖] Step 12: 倒可乐")

            # L J6 旋转 60°
            self._robo_ctrl_act_j([0, 0, 0, 0, 0, 60], incremental=True,
                                   point_count=70, message_time=0.06, side="L")
            time.sleep(5.0)

            # L J6 再旋转 30°
            self._robo_ctrl_act_j([0, 0, 0, 0, 0, 30], incremental=True,
                                   point_count=30, message_time=0.03, side="L")
            time.sleep(2.0)

            # L J6 回转 -87°
            self._robo_ctrl_act_j([0, 0, 0, 0, 0, -87], incremental=True,
                                   point_count=30, message_time=0.03, side="L")
            time.sleep(3.0)

            # ---- Step 13: 放回杯子 ----
            self.get_logger().info("[拧瓶盖] Step 13: 放回杯子")

            # 先移到放置位
            if not self._robo_ctrl_move_cart(self._place_pose, side="L"):
                self.get_logger().warn("移到放置位失败, 尝试原地释放")
            time.sleep(2.0)

            # 松开夹爪
            self._open_gripper(gripper_id=GRIPPER_ID_L)
            time.sleep(0.7)

            # L arm 撤离
            self._robo_ctrl_move_cart([-80, 0, 30, 0, 0, 0], is_incremental=True, side="L")
            time.sleep(2.0)

            self.get_logger().info("==== 拧瓶盖任务完成 ====")
            self._set_state(TaskState.IDLE)

        except Exception as e:
            self.get_logger().error(f"拧瓶盖任务异常: {e}")
            self._set_state(TaskState.ERROR)
            time.sleep(3)
            self._set_state(TaskState.IDLE)
        finally:
            # 恢复检测和原始 class_id
            self._detection_paused = False
            self._target_class_id = orig_class_id

    # =================================================================
    # 等待检测 (实机)
    # =================================================================
    def _wait_for_detection(self):
        self.get_logger().info(
            f"等待检测目标 (class_id={self._target_class_id}, "
            f"超时={self._detection_timeout}s)..."
        )
        deadline = time.monotonic() + self._detection_timeout
        while time.monotonic() < deadline:
            with self._lock:
                for obj in self._tracked_objects:
                    if obj.valid_count >= self._valid_threshold:
                        return {
                            "class_id": obj.class_id,
                            "pos_mm": list(obj.pos_mm),
                        }
            time.sleep(0.1)
        return None

    # =================================================================
    # 接近目标
    # =================================================================
    def _approach_object(self, target_pos) -> bool:
        """接近目标。

        target_pos:
          仿真模式: [x, y, z] 米, 用于 MoveIt PoseStamped
          实机模式: [x, y, z] mm, 用于 robo_ctrl 增量运动
        """
        if self._use_moveit:
            # 仿真: 直接 MoveIt 规划到目标位置 (PoseStamped)
            from geometry_msgs.msg import PoseStamped
            pose = PoseStamped()
            pose.header.frame_id = self._ref_frame
            pose.pose.position.x = target_pos[0]
            pose.pose.position.y = target_pos[1]
            pose.pose.position.z = target_pos[2]
            pose.pose.orientation.w = 1.0
            return self._moveit_move_to_pose(pose, phase="接近目标")
        else:
            # 实机: 计算增量偏移
            if self._current_tcp is None:
                self.get_logger().error("TCP 位姿未知")
                return False

            dx = target_pos[0] - self._current_tcp.x
            dy = target_pos[1] - self._current_tcp.y
            # z 使用 桌面高度+物体高度 计算 (与 dualarm 一致)
            dz = self._desk_height + self._object_height - self._current_tcp.z

            self.get_logger().info(
                f"接近增量: dx={dx:.1f} dy={dy:.1f} dz={dz:.1f} mm | "
                f"TCP=({self._current_tcp.x:.1f}, {self._current_tcp.y:.1f}, "
                f"{self._current_tcp.z:.1f}) | "
                f"desk={self._desk_height} obj={self._object_height}"
            )
            return self._robo_ctrl_act_incremental(dx, dy, dz)

    # =================================================================
    # 统一运动接口
    # =================================================================
    def _move_to_pose(self, pose, phase="移动", incremental=False) -> bool:
        """统一运动接口, 根据模式分发。"""
        if self._use_moveit:
            from geometry_msgs.msg import PoseStamped
            ps = PoseStamped()
            ps.header.frame_id = self._ref_frame
            ps.pose.position.x = pose[0]
            ps.pose.position.y = pose[1]
            ps.pose.position.z = pose[2]
            ps.pose.orientation.w = 1.0
            return self._moveit_move_to_pose(ps, phase=phase)
        else:
            return self._robo_ctrl_move_cart(pose, is_incremental=incremental)

    # =================================================================
    # MoveIt 后端
    # =================================================================
    def _moveit_move_to_pose(self, pose_stamped, phase="移动") -> bool:
        """MoveIt 规划+执行。"""
        from rclpy.action import ActionClient
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import (
            Constraints, PositionConstraint, OrientationConstraint,
            RobotState as MoveItRobotState, BoundingVolume,
        )
        from shape_msgs.msg import SolidPrimitive

        if not self._services_ready:
            self.get_logger().error("MoveGroup 未就绪")
            return False

        self._set_state(TaskState.PLANNING if phase == "接近目标" else TaskState.OBSERVATION)
        self.get_logger().info(f"━━━ {phase}: MoveIt 规划中 ━━━")

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self._planning_group
        req.num_planning_attempts = self.get_parameter("planning_attempts").value
        req.allowed_planning_time = self.get_parameter("planning_time").value
        req.max_velocity_scaling_factor = self.get_parameter("velocity_scale").value
        req.max_acceleration_scaling_factor = self.get_parameter("acceleration_scale").value
        req.pipeline_id = "ompl"
        req.start_state = MoveItRobotState()
        req.start_state.is_diff = True

        # 位置约束
        pos_con = PositionConstraint()
        pos_con.header.frame_id = pose_stamped.header.frame_id
        pos_con.link_name = self._ee_link
        pos_con.target_point_offset.x = 0.0
        pos_con.target_point_offset.y = 0.0
        pos_con.target_point_offset.z = 0.0
        pos_con.weight = 1.0

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.05, 0.05, 0.05]
        bv = BoundingVolume()
        bv.primitives = [primitive]
        bv.primitive_poses = [pose_stamped.pose]
        pos_con.constraint_region = bv

        orient_con = OrientationConstraint()
        orient_con.header.frame_id = pose_stamped.header.frame_id
        orient_con.link_name = self._ee_link
        orient_con.orientation = pose_stamped.pose.orientation
        orient_con.weight = 1.0
        orient_con.absolute_x_axis_tolerance = 0.15
        orient_con.absolute_y_axis_tolerance = 0.15
        orient_con.absolute_z_axis_tolerance = 0.15

        constraints = Constraints()
        constraints.position_constraints = [pos_con]
        constraints.orientation_constraints = [orient_con]
        req.goal_constraints = [constraints]

        # 工作空间
        req.workspace_parameters.min_corner.x = -2.0
        req.workspace_parameters.min_corner.y = -2.0
        req.workspace_parameters.min_corner.z = -2.0
        req.workspace_parameters.max_corner.x = 2.0
        req.workspace_parameters.max_corner.y = 2.0
        req.workspace_parameters.max_corner.z = 2.0

        # 异步发送并等待
        event = threading.Event()
        result = {"ok": False}

        def on_goal_response(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn(f"✗ {phase}: goal 被拒绝")
                event.set()
                return
            def on_result(f):
                res = f.result()
                result["ok"] = res.result.error_code.val == 1
                event.set()
            goal_handle.get_result_async().add_done_callback(on_result)

        self._move_group_client.send_goal_async(goal).add_done_callback(on_goal_response)
        event.wait(timeout=30.0)

        if result["ok"]:
            self.get_logger().info(f"✓ {phase}: 执行完成")
        else:
            self.get_logger().warn(f"✗ {phase}: 失败")
            self._set_state(TaskState.ERROR)
            time.sleep(2)
            self._set_state(TaskState.IDLE)

        return result["ok"]

    # =================================================================
    # robo_ctrl 后端
    # =================================================================
    def _call_service(self, client, request, timeout_sec=30.0, service_name=""):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error(f"服务 {service_name} 超时 ({timeout_sec}s)")
                return None
            time.sleep(0.01)
        return future.result()

    def _reset_robot_error(self, side="L") -> bool:
        """重置机器人错误 (RobotActJ command_type=3)。"""
        from robo_ctrl.srv import RobotActJ

        req = RobotActJ.Request()
        req.command_type = 3  # ResetAllError
        req.target_joints = [0.0] * 6
        req.point_count = 0
        req.message_time = 0.0
        req.use_incremental = False

        if side == "R":
            client = self._R_robot_act_j_client
            name = "R/robot_act_j_reset"
        else:
            client = self._robot_act_j_client
            name = "L/robot_act_j_reset"

        resp = self._call_service(client, req, 5.0, name)
        if resp is not None and resp.success:
            self.get_logger().info(f"{name} 成功")
            return True
        self.get_logger().warn(f"{name} 失败或不可用")
        return False

    def _servo_move_start(self, side="L") -> bool:
        """启动伺服模式 (RobotAct command_type=0)。"""
        from robo_ctrl.srv import RobotAct
        from robo_ctrl.msg import TCPPose

        req = RobotAct.Request()
        req.command_type = 0  # ServoMoveStart
        req.tcp_pose = TCPPose(x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0)
        req.point_count = 0
        req.message_time = 0.0
        req.plan_type = 0
        req.use_incremental = False

        if side == "R":
            client = self._R_robot_act_client
            name = "R/robot_act_servo_start"
        else:
            client = self._robot_act_client
            name = "L/robot_act_servo_start"

        resp = self._call_service(client, req, 10.0, name)
        if resp is None:
            return False
        if not resp.success:
            self.get_logger().error(f"{name} 失败: {resp.message}")
            return False
        self.get_logger().info(f"{name} 成功")
        return True

    def _robo_ctrl_move_cart(self, pose, is_incremental=False, side="L") -> bool:
        from robo_ctrl.srv import RobotMoveCart
        from robo_ctrl.msg import TCPPose

        if not self._services_ready:
            return False

        req = RobotMoveCart.Request()
        req.tcp_pose = TCPPose(
            x=float(pose[0]), y=float(pose[1]), z=float(pose[2]),
            rx=float(pose[3]), ry=float(pose[4]), rz=float(pose[5]),
        )
        req.velocity = self._velocity
        req.acceleration = self._accel
        req.config = -1
        req.blend_time = -1.0
        req.use_increment = is_incremental
        req.tool = -1
        req.user = -1
        req.ovl = 0.0

        if side == "R":
            client = self._R_move_cart_client
            name = "R/robot_move_cart"
        else:
            client = self._move_cart_client
            name = "L/robot_move_cart"

        resp = self._call_service(client, req, 30.0, name)
        if resp is None:
            return False
        if not resp.success:
            self.get_logger().error(f"{name} 失败: {resp.message}")
            return False
        return True

    def _robo_ctrl_act_incremental(self, dx, dy, dz) -> bool:
        from robo_ctrl.srv import RobotAct
        from robo_ctrl.msg import TCPPose

        req = RobotAct.Request()
        req.command_type = 0
        req.tcp_pose = TCPPose(x=dx, y=dy, z=dz, rx=0.0, ry=0.0, rz=0.0)
        req.point_count = 180
        req.message_time = 0.01
        req.plan_type = 0
        req.use_incremental = True

        resp = self._call_service(self._robot_act_client, req, 30.0, "robot_act")
        if resp is None:
            return False
        if not resp.success:
            self.get_logger().error(f"robot_act 失败: {resp.message}")
            return False
        return True

    def _robo_ctrl_act_j(self, joints, incremental=False,
                         point_count=100, message_time=0.01,
                         side="L") -> bool:
        """关节空间运动 (RobotActJ)。

        joints: 6 个关节角度 (度)
        incremental: True=增量, False=绝对
        side: "L"=左臂, "R"=右臂
        """
        from robo_ctrl.srv import RobotActJ

        req = RobotActJ.Request()
        req.command_type = 0
        req.target_joints = list(joints)
        req.point_count = point_count
        req.message_time = message_time
        req.use_incremental = incremental

        if side == "R":
            client = self._R_robot_act_j_client
            name = "R/robot_act_j"
        else:
            client = self._robot_act_j_client
            name = "L/robot_act_j"

        resp = self._call_service(client, req, 30.0, name)
        if resp is None:
            return False
        if not resp.success:
            self.get_logger().error(f"{name} 失败: {resp.message}")
            return False
        return True

    def _robo_ctrl_arc(self, circle_center, radian,
                       initial_orientation=None, face_center=True,
                       point_count=200, message_time=0.006,
                       side="L") -> bool:
        """圆弧运动 (RobotAct plan_type=1)。"""
        from robo_ctrl.srv import RobotAct
        from robo_ctrl.msg import TCPPose
        from geometry_msgs.msg import Point, Vector3

        req = RobotAct.Request()
        req.command_type = 0
        req.tcp_pose = TCPPose(x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0)
        req.point_count = point_count
        req.message_time = message_time
        req.plan_type = 1
        req.use_incremental = True
        req.circle_center = Point(
            x=float(circle_center[0]), y=float(circle_center[1]), z=float(circle_center[2])
        )
        req.radian = float(radian)
        if initial_orientation is None:
            req.initial_orientation = Vector3(x=0.0, y=-1.0, z=0.0)
        else:
            req.initial_orientation = Vector3(
                x=float(initial_orientation[0]),
                y=float(initial_orientation[1]),
                z=float(initial_orientation[2]),
            )
        req.face_center = face_center

        if side == "R":
            client = self._R_robot_act_client
            name = "R/robot_act_arc"
        else:
            client = self._robot_act_client
            name = "L/robot_act_arc"

        resp = self._call_service(client, req, 30.0, name)
        if resp is None:
            return False
        if not resp.success:
            self.get_logger().error(f"{name} 失败: {resp.message}")
            return False
        return True

    # =================================================================
    # 姿态修正
    # =================================================================
    def _fix_orientation(self, target_rx=-90.0, target_ry=0.0, target_rz=-90.0,
                         side="L", velocity=90.0) -> bool:
        """修正末端姿态到目标 rx/ry/rz (增量模式)。

        dualarm 的 L_fix() 逻辑: 计算当前姿态与目标的差值, 用增量运动修正。
        """
        if side == "R":
            tcp = self._R_current_tcp
        else:
            tcp = self._current_tcp

        if tcp is None:
            self.get_logger().error(f"{side}臂 TCP 位姿未知, 无法修正姿态")
            return False

        drx = float(target_rx - tcp.rx)
        dry = float(target_ry - tcp.ry)
        drz = float(target_rz - tcp.rz)
        total_diff = abs(drx) + abs(dry) + abs(drz)

        self.get_logger().info(
            f"姿态修正 ({side}臂): drx={drx:.1f} dry={dry:.1f} drz={drz:.1f} "
            f"total_diff={total_diff:.1f}"
        )

        if total_diff < 1.0:
            self.get_logger().info("姿态已达标, 跳过修正")
            return True

        # 保存原始速度, 修正后恢复
        orig_vel = self._velocity
        self._velocity = velocity
        ok = self._robo_ctrl_move_cart(
            [0, 0, 0, drx, dry, drz], is_incremental=True, side=side
        )
        self._velocity = orig_vel

        # 等待运动完成 (每度 50ms, 最少 1s)
        wait_ms = max(1000, int(total_diff * 50))
        time.sleep(wait_ms / 1000.0)

        if ok:
            self.get_logger().info(f"姿态修正完成 ({side}臂)")
        else:
            self.get_logger().error(f"姿态修正失败 ({side}臂)")
        return ok

    # =================================================================
    # 夹爪控制
    # =================================================================
    def _get_gripper_client(self, gripper_id):
        """根据 gripper_id 返回对应的夹爪服务客户端。"""
        if gripper_id == 10:  # 右臂夹爪
            return self._R_gripper_client
        return self._gripper_client  # 左臂夹爪 (默认)

    def _enable_gripper(self, gripper_id=None) -> bool:
        """使能夹爪 (command=1, position=OPEN)。"""
        if gripper_id is None:
            gripper_id = self._gripper_id
        from epg50_gripper_ros.srv import GripperCommand
        client = self._get_gripper_client(gripper_id)
        if client is None:
            self.get_logger().warn(f"夹爪服务不可用 (id={gripper_id})")
            return False
        req = GripperCommand.Request()
        req.slave_id = gripper_id
        req.command = 1  # GRIPPER_ENABLE
        req.position = 0  # GRIPPER_OPEN
        req.speed = 255
        req.torque = 255
        resp = self._call_service(client, req, 5.0, "gripper_enable")
        if resp is not None and resp.success:
            self.get_logger().info(f"夹爪 {gripper_id} 使能成功")
            return True
        self.get_logger().warn(f"夹爪 {gripper_id} 使能失败")
        return False

    def _open_gripper(self, gripper_id=None) -> bool:
        return self._gripper_command(position=0, gripper_id=gripper_id)

    def _close_gripper(self, gripper_id=None) -> bool:
        return self._gripper_command(position=255, gripper_id=gripper_id)

    def _gripper_command(self, position: int, gripper_id=None) -> bool:
        if gripper_id is None:
            gripper_id = self._gripper_id

        if self._use_moveit:
            # 仿真: 串口直连
            if not hasattr(self, '_gripper_ctrl') or self._gripper_ctrl is None:
                self.get_logger().warn("夹爪控制器未初始化")
                return False
            if position == 0:
                return self._gripper_ctrl.open()
            else:
                return self._gripper_ctrl.close()
        else:
            # 实机: GripperCommand 服务
            from epg50_gripper_ros.srv import GripperCommand
            client = self._get_gripper_client(gripper_id)
            if client is None:
                self.get_logger().warn(f"夹爪服务不可用 (id={gripper_id})")
                return False
            req = GripperCommand.Request()
            req.slave_id = gripper_id
            req.command = 2  # GRIPPER_SET
            req.position = position
            req.speed = 255
            req.torque = 255
            resp = self._call_service(client, req, 5.0, "gripper_command")
            return resp is not None and resp.success

    # =================================================================
    # 状态管理
    # =================================================================
    def _set_state(self, state: TaskState) -> None:
        self._state = state
        self.get_logger().info(f"[状态] {state.value}")

    def get_state(self) -> TaskState:
        return self._state


def main(args=None):
    rclpy.init(args=args)
    node = ArmTaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("手动中断")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
