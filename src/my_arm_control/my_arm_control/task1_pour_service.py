import time
import math
import threading
from enum import Enum
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from robo_ctrl.msg import RobotState, TCPPose
from robo_ctrl.srv import RobotMoveCart, RobotAct, RobotActJ
from epg50_gripper_ros.srv import GripperCommand
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped


class PourState(Enum):
    IDLE = "IDLE"
    OBSERVE = "OBSERVE"
    WAIT_VISION = "WAIT_VISION"
    GRASP_BOTTLE = "GRASP_BOTTLE"
    GRASP_CUP = "GRASP_CUP"
    OPEN_CAP = "OPEN_CAP"
    POUR = "POUR"
    PLACE_BOTTLE = "PLACE_BOTTLE"
    PLACE_CUP = "PLACE_CUP"
    NEXT_DRINK = "NEXT_DRINK"
    DONE = "DONE"
    ERROR = "ERROR"


class Task1PourService(Node):
    """任务一倒水服务状态机。

    流程:
      IDLE → OBSERVE → WAIT_VISION → GRASP_BOTTLE → GRASP_CUP
      → OPEN_CAP → POUR → PLACE_BOTTLE → PLACE_CUP
      → NEXT_DRINK → (重复) → DONE
    """

    def __init__(self):
        super().__init__("task1_pour_service")

        self.declare_parameter("robot_prefix", "/L")
        self._prefix = self.get_parameter("robot_prefix").value

        # 观测位姿 (mm/度)
        self.declare_parameter("observe_pose", [99.917, -144.210, 542.554, -125.357, 0.0, -100.476])
        self.declare_parameter("velocity", 50.0)
        self.declare_parameter("acceleration", 50.0)

        # 桌面高度 & 物体高度
        self.declare_parameter("desk_height", 360.0)
        self.declare_parameter("bottle_height", 180.0)
        self.declare_parameter("cup_height", 120.0)
        self.declare_parameter("approach_offset_z", 150.0)
        self.declare_parameter("retreat_z", 80.0)

        # 拧瓶盖关节角度
        self.declare_parameter("cap_open_joints_l", [-55.0, -90.0, -120.0, 30.0, 81.272, 0.0])
        self.declare_parameter("cap_open_joints_r", [45.434, -124.551, 128.388, -184.270, 19.218, 0.0])
        self.declare_parameter("pour_j6_angle", 60.0)

        # 放置位姿
        self.declare_parameter("place_bottle_pose", [200.0, -200.0, 200.0, -90.0, 0.0, -90.0])
        self.declare_parameter("place_cup_pose", [250.0, -100.0, 200.0, -90.0, 0.0, -90.0])

        # 夹爪参数
        self.declare_parameter("gripper_id_L", 9)
        self.declare_parameter("gripper_id_R", 10)

        # 处理顺序: 先处理哪个饮品
        self.declare_parameter("drink_order", ["water", "cola"])

        # TF 坐标变换参数
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("vision_frame", "camera_depth_frame")
        self.declare_parameter("tf_timeout", 1.0)

        # 碰撞安全
        self.declare_parameter("collision_safety_distance", 100.0)

        self._observe_pose = self.get_parameter("observe_pose").value
        self._velocity = self.get_parameter("velocity").value
        self._accel = self.get_parameter("acceleration").value
        self._desk_height = self.get_parameter("desk_height").value
        self._bottle_height = self.get_parameter("bottle_height").value
        self._cup_height = self.get_parameter("cup_height").value
        self._approach_offset_z = self.get_parameter("approach_offset_z").value
        self._retreat_z = self.get_parameter("retreat_z").value
        self._cap_open_joints_l = self.get_parameter("cap_open_joints_l").value
        self._cap_open_joints_r = self.get_parameter("cap_open_joints_r").value
        self._pour_j6_angle = self.get_parameter("pour_j6_angle").value
        self._place_bottle_pose = self.get_parameter("place_bottle_pose").value
        self._place_cup_pose = self.get_parameter("place_cup_pose").value
        self._gripper_id_L = self.get_parameter("gripper_id_L").value
        self._gripper_id_R = self.get_parameter("gripper_id_R").value
        self._drink_order = self.get_parameter("drink_order").value
        self._target_frame = self.get_parameter("target_frame").value
        self._vision_frame = self.get_parameter("vision_frame").value
        self._tf_timeout = self.get_parameter("tf_timeout").value
        self._collision_safety_distance = self.get_parameter("collision_safety_distance").value

        self._state = PourState.IDLE
        self._services_ready = False
        self._current_tcp_L: Optional[TCPPose] = None
        self._current_tcp_R: Optional[TCPPose] = None
        self._last_state_L: Optional[RobotState] = None
        self._last_state_R: Optional[RobotState] = None

        # 视觉数据
        self._objects = {}
        self._vision_ready = False

        # TF 坐标变换
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._init_robo_ctrl()

        self._vision_subs = {
            'bottle_water': self.create_subscription(
                PoseStamped, '/vision/bottle_water', lambda msg, n='bottle_water': self._vision_cb(msg, n), 10
            ),
            'bottle_cola': self.create_subscription(
                PoseStamped, '/vision/bottle_cola', lambda msg, n='bottle_cola': self._vision_cb(msg, n), 10
            ),
            'cup_1': self.create_subscription(
                PoseStamped, '/vision/cup_1', lambda msg, n='cup_1': self._vision_cb(msg, n), 10
            ),
            'cup_2': self.create_subscription(
                PoseStamped, '/vision/cup_2', lambda msg, n='cup_2': self._vision_cb(msg, n), 10
            ),
        }

        self._state_sub_L = self.create_subscription(
            RobotState, f'{self._prefix}/robot_state', self._state_cb_L, 10
        )
        self._state_sub_R = self.create_subscription(
            RobotState, '/R/robot_state', self._state_cb_R, 10
        )

        self._init_thread = threading.Thread(target=self._wait_services, daemon=True)
        self._init_thread.start()

        self.get_logger().info("Task1PourService 启动 | 等待服务就绪...")

    # ==================== 初始化 ====================

    def _init_robo_ctrl(self):
        self._move_cart_L = self.create_client(RobotMoveCart, f'{self._prefix}/robot_move_cart')
        self._act_L = self.create_client(RobotAct, f'{self._prefix}/robot_act')
        self._act_j_L = self.create_client(RobotActJ, f'{self._prefix}/robot_act_j')
        self._gripper_L = self.create_client(GripperCommand, 'gripper_command')

        self._move_cart_R = self.create_client(RobotMoveCart, '/R/robot_move_cart')
        self._act_R = self.create_client(RobotAct, '/R/robot_act')
        self._act_j_R = self.create_client(RobotActJ, '/R/robot_act_j')
        self._gripper_R = self.create_client(GripperCommand, 'R_gripper_command')

    def _wait_services(self):
        required = [
            (self._move_cart_L, f'{self._prefix}/robot_move_cart'),
            (self._act_L, f'{self._prefix}/robot_act'),
            (self._act_j_L, f'{self._prefix}/robot_act_j'),
            (self._move_cart_R, '/R/robot_move_cart'),
            (self._act_R, '/R/robot_act'),
            (self._act_j_R, '/R/robot_act_j'),
        ]
        for client, name in required:
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f'服务 {name} 不可用')
                return

        for client, name in [(self._gripper_L, 'gripper_command'), (self._gripper_R, 'R_gripper_command')]:
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f'{name} 不可用, 跳过夹爪')

        self._services_ready = True
        self.get_logger().info('所有服务就绪, 等待视觉数据...')
        self._start_flow()

    # ==================== 视觉回调 ====================

    def _vision_cb(self, msg, name):
        frame_id = msg.header.frame_id
        if frame_id == '':
            frame_id = self._vision_frame

        if frame_id != self._target_frame:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._target_frame,
                    frame_id,
                    msg.header.stamp,
                    timeout=rclpy.duration.Duration(seconds=self._tf_timeout)
                )
                transformed = do_transform_pose_stamped(msg, transform)
                self._objects[name] = {
                    'x': transformed.pose.position.x,
                    'y': transformed.pose.position.y,
                    'z': transformed.pose.position.z,
                }
            except Exception as e:
                self.get_logger().warn(
                    f'TF 变换失败 ({frame_id} → {self._target_frame}): {e}, '
                    f'使用原始坐标'
                )
                self._objects[name] = {
                    'x': msg.pose.position.x,
                    'y': msg.pose.position.y,
                    'z': msg.pose.position.z,
                }
        else:
            self._objects[name] = {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z,
            }

        if not self._vision_ready and len(self._objects) >= 4:
            self._vision_ready = True
            self.get_logger().info('所有物体已检测到:')
            for n, p in self._objects.items():
                self.get_logger().info(f'  {n}: ({p["x"]:.1f}, {p["y"]:.1f}, {p["z"]:.1f})')

    def _state_cb_L(self, msg):
        self._current_tcp_L = msg.tcp_pose
        self._last_state_L = msg

    def _state_cb_R(self, msg):
        self._current_tcp_R = msg.tcp_pose
        self._last_state_R = msg

    # ==================== 服务调用 ====================

    def _call(self, client, req, timeout=30.0, name=''):
        future = client.call_async(req)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error(f'{name} 超时')
                return None
            time.sleep(0.01)
        return future.result()

    def _check_collision_risk(self, target_xyz, moving_side='L'):
        static_side = 'R' if moving_side == 'L' else 'L'
        static_tcp = self._current_tcp_R if static_side == 'R' else self._current_tcp_L
        if static_tcp is None:
            return True
        dx = target_xyz[0] - static_tcp.x
        dy = target_xyz[1] - static_tcp.y
        dz = target_xyz[2] - static_tcp.z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist < self._collision_safety_distance:
            self.get_logger().error(
                f'碰撞风险: {moving_side}臂目标距{static_side}臂TCP仅 {dist:.1f}mm'
                f' (阈值 {self._collision_safety_distance}mm) — 已阻止'
            )
            return False
        return True

    def _move_cart(self, pose, side='L', incremental=False):
        tcp = self._current_tcp_R if side == 'R' else self._current_tcp_L
        if incremental:
            if tcp is not None:
                target_xyz = (tcp.x + pose[0], tcp.y + pose[1], tcp.z + pose[2])
            else:
                target_xyz = (pose[0], pose[1], pose[2])
        else:
            target_xyz = (pose[0], pose[1], pose[2])
        if not self._check_collision_risk(target_xyz, side):
            return False

        req = RobotMoveCart.Request()
        req.tcp_pose = TCPPose(x=float(pose[0]), y=float(pose[1]), z=float(pose[2]),
                                rx=float(pose[3]), ry=float(pose[4]), rz=float(pose[5]))
        req.velocity = self._velocity
        req.acceleration = self._accel
        req.config = -1
        req.blend_time = -1.0
        req.use_increment = incremental
        req.tool = -1
        req.user = -1
        req.ovl = 0.0
        client = self._move_cart_R if side == 'R' else self._move_cart_L
        name = f'{side}/robot_move_cart'
        resp = self._call(client, req, 30.0, name)
        if resp is None:
            return False
        if not resp.success:
            self.get_logger().error(f'{name} 失败: {resp.message}')
            return False
        return True

    def _act_j(self, joints, side='L', incremental=False, point_count=100, message_time=0.01):
        moving_tcp = self._current_tcp_L if side == 'L' else self._current_tcp_R
        static_tcp = self._current_tcp_R if side == 'L' else self._current_tcp_L
        if moving_tcp is not None and static_tcp is not None:
            dx = moving_tcp.x - static_tcp.x
            dy = moving_tcp.y - static_tcp.y
            dz = moving_tcp.z - static_tcp.z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            if dist < self._collision_safety_distance:
                self.get_logger().error(
                    f'碰撞风险: {side}臂与静止臂当前TCP仅 {dist:.1f}mm'
                    f' (阈值 {self._collision_safety_distance}mm) — 已阻止关节运动'
                )
                return False

        req = RobotActJ.Request()
        req.command_type = 0
        req.target_joints = [float(j) for j in joints]
        req.point_count = point_count
        req.message_time = message_time
        req.use_incremental = incremental
        client = self._act_j_R if side == 'R' else self._act_j_L
        name = f'{side}/robot_act_j'
        resp = self._call(client, req, 30.0, name)
        if resp is None:
            return False
        if not resp.success:
            self.get_logger().error(f'{name} 失败: {resp.message}')
            return False
        return True

    def _act_incremental(self, dx, dy, dz, side='L'):
        tcp = self._current_tcp_R if side == 'R' else self._current_tcp_L
        if tcp is not None:
            target_xyz = (tcp.x + dx, tcp.y + dy, tcp.z + dz)
        else:
            self.get_logger().error(f'{side} TCP 未知, 无法检查碰撞')
            return False
        if not self._check_collision_risk(target_xyz, side):
            return False

        req = RobotAct.Request()
        req.command_type = 0
        req.tcp_pose = TCPPose(x=dx, y=dy, z=dz, rx=0.0, ry=0.0, rz=0.0)
        req.point_count = 180
        req.message_time = 0.01
        req.plan_type = 0
        req.use_incremental = True
        client = self._act_R if side == 'R' else self._act_L
        name = f'{side}/robot_act'
        resp = self._call(client, req, 30.0, name)
        if resp is None:
            return False
        if not resp.success:
            self.get_logger().error(f'{name} 失败: {resp.message}')
            return False
        return True

    def _approach_object(self, obj_pos, side='L', target_height=None):
        tcp = self._current_tcp_R if side == 'R' else self._current_tcp_L
        if tcp is None:
            self.get_logger().error(f'{side} TCP 未知')
            return False
        if target_height is None:
            target_height = self._desk_height + self._bottle_height
        dx = obj_pos['x'] - tcp.x
        dy = obj_pos['y'] - tcp.y
        dz = target_height - tcp.z
        self.get_logger().info(
            f'接近 ({side}): dx={dx:.1f} dy={dy:.1f} dz={dz:.1f} | '
            f'TCP=({tcp.x:.1f}, {tcp.y:.1f}, {tcp.z:.1f})'
        )
        return self._act_incremental(dx, dy, dz, side=side)

    def _gripper_cmd(self, position, gripper_id):
        if gripper_id == self._gripper_id_R:
            client = self._gripper_R
        else:
            client = self._gripper_L
        if client is None:
            self.get_logger().warn(f'夹爪 {gripper_id} 不可用')
            return False
        req = GripperCommand.Request()
        req.slave_id = gripper_id
        req.command = 2
        req.position = position
        req.speed = 255
        req.torque = 255
        resp = self._call(client, req, 5.0, f'gripper_{gripper_id}')
        return resp is not None and resp.success

    def _close_gripper(self, gripper_id):
        return self._gripper_cmd(255, gripper_id)

    def _open_gripper(self, gripper_id):
        return self._gripper_cmd(0, gripper_id)

    def _wait_motion(self, secs=2.0):
        time.sleep(secs)

    def _wait_motion_done(self, side='L', timeout=15.0):
        state = self._last_state_L if side == 'L' else self._last_state_R
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = self._last_state_L if side == 'L' else self._last_state_R
            if state is not None and state.motion_done:
                return True
            if state is not None and state.error_code != 0:
                self.get_logger().error(f'{side} 运动错误: error_code={state.error_code}')
                return False
            time.sleep(0.02)
        self.get_logger().error(f'{side} 运动超时 ({timeout}s)')
        return False

    # ==================== 状态管理 ====================

    def _set_state(self, state):
        self._state = state
        self.get_logger().info(f'[状态] {state.value}')

    # ==================== 主流程 ====================

    def _start_flow(self):
        thread = threading.Thread(target=self._run_pour_service, daemon=True)
        thread.start()

    def _run_pour_service(self):
        self.get_logger().info('==== 开始任务一: 倒水服务 ====')

        deadline = time.monotonic() + 30.0
        while len(self._objects) < 4:
            if time.monotonic() > deadline:
                self.get_logger().error('视觉数据等待超时(30s), 使用默认坐标')
                self._objects.setdefault('bottle_water', {'x': 250.0, 'y': -200.0, 'z': 360.0})
                self._objects.setdefault('bottle_cola', {'x': 300.0, 'y': -100.0, 'z': 360.0})
                self._objects.setdefault('cup_1', {'x': 200.0, 'y': -150.0, 'z': 360.0})
                self._objects.setdefault('cup_2', {'x': 280.0, 'y': -250.0, 'z': 360.0})
                break
            self.get_logger().info(f'等待视觉数据... ({len(self._objects)}/4)')
            time.sleep(1.0)

        self.get_logger().info('视觉数据就绪, 开始倒水任务')

        for drink_idx, drink in enumerate(self._drink_order):
            self.get_logger().info(f'==== 处理第 {drink_idx+1} 种饮品: {drink} ====')

            bottle_name = f'bottle_{drink}'
            cup_name = f'cup_{drink_idx + 1}'

            if bottle_name not in self._objects:
                self.get_logger().error(f'未检测到 {bottle_name}')
                continue
            if cup_name not in self._objects:
                self.get_logger().error(f'未检测到 {cup_name}')
                continue

            bottle_pos = self._objects[bottle_name]
            cup_pos = self._objects[cup_name]

            bottle_orig = bottle_pos.copy()
            cup_orig = cup_pos.copy()

            try:
                # ---- Phase 1: OBSERVE ----
                self._set_state(PourState.OBSERVE)
                self.get_logger().info(f'[Phase 1] 前往观测位')
                if not self._move_cart(self._observe_pose, side='L'):
                    self._set_state(PourState.ERROR)
                    return
                self._wait_motion_done(side='L', timeout=5.0)

                # ---- Phase 2: GRASP BOTTLE (L arm) ----
                self._set_state(PourState.GRASP_BOTTLE)
                self.get_logger().info(f'[Phase 2] 左臂抓取瓶子: {bottle_name}')

                if not self._approach_object(bottle_pos, side='L'):
                    self._set_state(PourState.ERROR)
                    return
                self._wait_motion_done(side='L', timeout=5.0)

                if not self._close_gripper(self._gripper_id_L):
                    self.get_logger().warn('左夹爪操作失败')
                self._wait_motion(0.5)

                if not self._move_cart([0, 0, self._retreat_z, 0, 0, 0], side='L', incremental=True):
                    self.get_logger().warn('左臂撤离失败')
                self._wait_motion_done(side='L', timeout=5.0)

                # ---- Phase 3: GRASP CUP (R arm) ----
                self._set_state(PourState.GRASP_CUP)
                self.get_logger().info(f'[Phase 3] 右臂抓取水杯: {cup_name}')

                if not self._approach_object(cup_pos, side='R', target_height=self._desk_height + self._cup_height):
                    self._set_state(PourState.ERROR)
                    return
                self._wait_motion_done(side='R', timeout=5.0)

                if not self._close_gripper(self._gripper_id_R):
                    self.get_logger().warn('右夹爪操作失败')
                self._wait_motion(0.5)

                if not self._move_cart([0, 0, self._retreat_z, 0, 0, 0], side='R', incremental=True):
                    self.get_logger().warn('右臂撤离失败')
                self._wait_motion_done(side='R', timeout=5.0)

                # ---- Phase 4: OPEN CAP ----
                self._set_state(PourState.OPEN_CAP)
                self.get_logger().info(f'[Phase 4] 拧瓶盖')

                if not self._act_j(self._cap_open_joints_l, side='L'):
                    self.get_logger().warn('左臂移往拧盖位失败')
                self._wait_motion_done(side='L', timeout=5.0)

                if not self._act_j(self._cap_open_joints_r, side='R'):
                    self.get_logger().warn('右臂移往接杯位失败')
                self._wait_motion_done(side='R', timeout=5.0)

                if not self._close_gripper(self._gripper_id_L):
                    self.get_logger().warn('左夹爪夹紧瓶盖失败')
                self._wait_motion(0.3)

                for i in range(2):
                    self.get_logger().info(f'拧盖圈 {i+1}/2')
                    if not self._act_j([0, 0, 0, 0, 0, 30], side='L', incremental=True, point_count=100, message_time=0.01):
                        self.get_logger().warn(f'旋转圈 {i+1} 失败')
                    self._wait_motion_done(side='L', timeout=5.0)

                if not self._open_gripper(self._gripper_id_L):
                    self.get_logger().warn('松开瓶盖失败')
                self._wait_motion(0.3)

                if not self._act_j([0, 0, 0, 0, 0, -60], side='L', incremental=True, point_count=100, message_time=0.01):
                    self.get_logger().warn('回退失败')
                self._wait_motion_done(side='L', timeout=5.0)

                # ---- Phase 5: POUR ----
                self._set_state(PourState.POUR)
                self.get_logger().info(f'[Phase 5] 倒水: {drink}')

                if not self._close_gripper(self._gripper_id_L):
                    self.get_logger().warn('左夹爪夹紧瓶子失败')
                self._wait_motion(0.3)

                tilt_angle = self._pour_j6_angle
                self.get_logger().info(f'倾斜 {tilt_angle}° 倒水')
                if not self._act_j([0, 0, 0, 0, 0, tilt_angle], side='L', incremental=True, point_count=70, message_time=0.06):
                    self.get_logger().warn('倾斜倒水失败')
                self._wait_motion_done(side='L', timeout=10.0)

                self.get_logger().info('回正瓶子')
                if not self._act_j([0, 0, 0, 0, 0, -tilt_angle], side='L', incremental=True, point_count=70, message_time=0.06):
                    self.get_logger().warn('回正失败')
                self._wait_motion_done(side='L', timeout=5.0)

                # ---- Phase 6: PLACE BOTTLE（回到原位） ----
                self._set_state(PourState.PLACE_BOTTLE)
                self.get_logger().info(f'[Phase 6] 放回瓶子到原位')

                if not self._approach_object(bottle_orig, side='L', target_height=self._desk_height + self._bottle_height):
                    self.get_logger().warn('回到瓶原位置失败')
                self._wait_motion_done(side='L', timeout=5.0)

                if not self._open_gripper(self._gripper_id_L):
                    self.get_logger().warn('释放瓶子失败')
                self._wait_motion(0.5)

                if not self._move_cart([0, 0, self._retreat_z, 0, 0, 0], side='L', incremental=True):
                    self.get_logger().warn('左臂撤离失败')
                self._wait_motion_done(side='L', timeout=5.0)

                # ---- Phase 7: PLACE CUP（回到原位） ----
                self._set_state(PourState.PLACE_CUP)
                self.get_logger().info(f'[Phase 7] 放回水杯到原位')

                if not self._approach_object(cup_orig, side='R', target_height=self._desk_height + self._cup_height):
                    self.get_logger().warn('回到杯原位置失败')
                self._wait_motion_done(side='R', timeout=5.0)

                if not self._open_gripper(self._gripper_id_R):
                    self.get_logger().warn('释放水杯失败')
                self._wait_motion(0.5)

                if not self._move_cart([0, 0, self._retreat_z, 0, 0, 0], side='R', incremental=True):
                    self.get_logger().warn('右臂撤离失败')
                self._wait_motion_done(side='R', timeout=5.0)

                self.get_logger().info(f'==== {drink} 处理完成 ====')

            except Exception as e:
                self.get_logger().error(f'{drink} 处理异常: {e}')
                self._set_state(PourState.ERROR)
                self._wait_motion(3)
                continue

        self._set_state(PourState.DONE)
        self.get_logger().info('==== 任务一全部完成 ====')


def main(args=None):
    rclpy.init(args=args)
    node = Task1PourService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
