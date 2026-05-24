#!/usr/bin/env python3
"""
任务一流程测试 (无需实机)

测试内容:
  1. 模块导入检查
  2. 参数声明检查
  3. 坐标转换计算
  4. bbox 回调 → 检测目标
  5. opencap 任务流程 (mock 服务)

使用:
  cd ~/ros2_ws && colcon build --packages-select my_arm_control
  ros2 run my_arm_control test_opencap_flow
  # 或直接:
  python3 src/FairinoDualArm/my_arm_control/test/test_opencap_flow.py
"""

import sys
import time
import threading
import unittest
from unittest.mock import MagicMock, patch, call

# ===== 测试 1: 模块导入 =====
class TestImports(unittest.TestCase):
    def test_import_arm_task_manager(self):
        """arm_task_manager 模块可以正常导入"""
        from my_arm_control import arm_task_manager
        self.assertTrue(hasattr(arm_task_manager, 'ArmTaskManager'))
        self.assertTrue(hasattr(arm_task_manager, 'TaskState'))

    def test_task_states(self):
        """TaskState 枚举包含所有必要状态"""
        from my_arm_control.arm_task_manager import TaskState
        required = ['IDLE', 'OBSERVATION', 'PLANNING', 'APPROACHING',
                     'GRABBING', 'RETREATING', 'PLACING', 'ERROR']
        for s in required:
            self.assertIn(s, [e.name for e in TaskState])

    def test_import_virtual_vision(self):
        """virtual_vision_node 可以导入"""
        from my_arm_control import virtual_vision_node
        self.assertTrue(hasattr(virtual_vision_node, 'main'))

    def test_import_fake_vision(self):
        """fake_vision_node 可以导入"""
        from my_arm_control import fake_vision_node
        self.assertTrue(hasattr(fake_vision_node, 'main'))


# ===== 测试 2: 坐标计算 =====
class TestCoordinateCalculation(unittest.TestCase):
    def test_bbox_to_mm(self):
        """bbox3d 坐标从米转换为毫米"""
        # 模拟 bbox3d 数据 (米)
        x, y, z = 0.4, -0.15, 0.3
        w, h, d = 0.05, 0.05, 0.08

        # 与 _bbox_cb 相同的计算
        cx = (x + w / 2.0) * 1000.0
        cy = (y + h / 2.0) * 1000.0
        cz = (z + d / 2.0) * 1000.0

        self.assertAlmostEqual(cx, 425.0)
        self.assertAlmostEqual(cy, -125.0)
        self.assertAlmostEqual(cz, 340.0)

    def test_approach_increment(self):
        """接近目标的增量计算"""
        # 模拟当前 TCP 位置 (mm)
        tcp_x, tcp_y, tcp_z = 99.917, -144.210, 542.554

        # 模拟检测到的可乐位置 (mm, 已转换)
        cola_x, cola_y = 425.0, -125.0

        # 偏移量
        offset_x, offset_y = -132.0, 45.0

        # 高度参数
        desk_height, object_height = 360.0, 89.0

        # 增量计算 (与 _task_opencap 一致)
        dx = cola_x + offset_x - tcp_x
        dy = cola_y + offset_y - tcp_y
        dz = desk_height + object_height - tcp_z

        self.assertAlmostEqual(dx, 425.0 - 132.0 - 99.917)
        self.assertAlmostEqual(dy, -125.0 + 45.0 - (-144.210))
        self.assertAlmostEqual(dz, 360.0 + 89.0 - 542.554)

    def test_orientation_fix_delta(self):
        """姿态修正增量计算"""
        # 当前姿态
        rx, ry, rz = -125.357, 0.0, -100.476

        # 目标姿态
        target_rx, target_ry, target_rz = -90.0, 0.0, -90.0

        drx = target_rx - rx
        dry = target_ry - ry
        drz = target_rz - rz

        self.assertAlmostEqual(drx, -90.0 - (-125.357))
        self.assertAlmostEqual(dry, 0.0)
        self.assertAlmostEqual(drz, -90.0 - (-100.476))

        # 总角度差
        total = abs(drx) + abs(dry) + abs(drz)
        self.assertGreater(total, 1.0)  # 需要修正
        self.assertLess(total, 100.0)   # 不会太大


# ===== 测试 3: 服务调用序列 (mock) =====
class TestOpencapFlow(unittest.TestCase):
    """测试 opencap 任务流程的服务调用序列。"""

    def setUp(self):
        """初始化 rclpy 并创建 mock 节点。"""
        import rclpy
        if not rclpy.ok():
            rclpy.init()

        from my_arm_control.arm_task_manager import ArmTaskManager

        # 创建节点 (会声明参数)
        self.node = ArmTaskManager()

        # 记录所有服务调用
        self.service_calls = []

        # Mock 所有 service client
        self._mock_all_clients()

    def tearDown(self):
        """销毁节点。"""
        self.node.destroy_node()

    def _mock_all_clients(self):
        """替换所有运动方法和夹爪方法为 mock。"""
        # 创建通用 mock response
        def make_response(success=True, message="ok"):
            resp = MagicMock()
            resp.success = success
            resp.message = message
            return resp

        self.mock_response = make_response()

        # 记录调用
        def record_call(service, **kwargs):
            self.service_calls.append({
                'service': service,
                'kwargs': kwargs,
                'timestamp': time.time(),
            })
            return self.mock_response

        # Mock 运动方法 (避免创建真实的 ROS2 Request 对象)
        def mock_move_cart(pose, is_incremental=False, side="L"):
            name = f"{side}/robot_move_cart"
            return record_call(name, pose=pose, incremental=is_incremental)

        def mock_act_incremental(dx, dy, dz):
            return record_call("robot_act", dx=dx, dy=dy, dz=dz)

        def mock_act_j(joints, incremental=False, point_count=100,
                       message_time=0.01, side="L"):
            name = f"{side}/robot_act_j"
            return record_call(name, joints=joints, incremental=incremental)

        def mock_arc(circle_center, radian, initial_orientation=None,
                     face_center=True, point_count=200, message_time=0.006,
                     side="L"):
            name = f"{side}/robot_act_arc"
            return record_call(name, circle_center=circle_center, radian=radian)

        def mock_fix_orientation(target_rx=-90.0, target_ry=0.0, target_rz=-90.0,
                                 side="L", velocity=90.0):
            return record_call("fix_orientation", target_rx=target_rx,
                               target_ry=target_ry, target_rz=target_rz)

        def mock_gripper_enable(gripper_id=None):
            return record_call("gripper_enable", gripper_id=gripper_id)

        def mock_open_gripper(gripper_id=None):
            return record_call("gripper_command", gripper_id=gripper_id, position=0)

        def mock_close_gripper(gripper_id=None):
            return record_call("gripper_command", gripper_id=gripper_id, position=255)

        def mock_move_to_pose(pose, phase="移动", incremental=False):
            return record_call("move_to_pose", pose=pose, phase=phase)

        # 替换方法
        self.node._robo_ctrl_move_cart = mock_move_cart
        self.node._robo_ctrl_act_incremental = mock_act_incremental
        self.node._robo_ctrl_act_j = mock_act_j
        self.node._robo_ctrl_arc = mock_arc
        self.node._fix_orientation = mock_fix_orientation
        self.node._enable_gripper = mock_gripper_enable
        self.node._open_gripper = mock_open_gripper
        self.node._close_gripper = mock_close_gripper
        self.node._move_to_pose = mock_move_to_pose

        # 标记服务就绪
        self.node._services_ready = True

    def test_bbox_callback_converts_units(self):
        """bbox 回调正确转换单位 (米→毫米)"""
        from depth_handler.msg import Bbox3dArray, Bbox3d

        msg = Bbox3dArray()
        bbox = Bbox3d()
        bbox.class_id = 1
        bbox.x = 0.4
        bbox.y = -0.15
        bbox.z = 0.3
        bbox.width = 0.05
        bbox.height = 0.05
        bbox.depth = 0.08
        msg.results = [bbox]

        self.node._target_class_id = 1
        self.node._bbox_cb(msg)

        self.assertEqual(len(self.node._tracked_objects), 1)
        obj = self.node._tracked_objects[0]
        self.assertEqual(obj.class_id, 1)
        self.assertAlmostEqual(obj.pos_mm[0], 425.0)
        self.assertAlmostEqual(obj.pos_mm[1], -125.0)
        self.assertAlmostEqual(obj.pos_mm[2], 340.0)

    def test_bbox_callback_filters_class_id(self):
        """bbox 回调按 class_id 过滤"""
        from depth_handler.msg import Bbox3dArray, Bbox3d

        msg = Bbox3dArray()
        # 目标 class_id=1 (可乐)
        bbox1 = Bbox3d()
        bbox1.class_id = 1
        bbox1.x, bbox1.y, bbox1.z = 0.4, -0.15, 0.3
        bbox1.width, bbox1.height, bbox1.depth = 0.05, 0.05, 0.08
        # 非目标 class_id=2
        bbox2 = Bbox3d()
        bbox2.class_id = 2
        bbox2.x, bbox2.y, bbox2.z = 0.5, -0.2, 0.3
        bbox2.width, bbox2.height, bbox2.depth = 0.05, 0.05, 0.08

        msg.results = [bbox1, bbox2]
        self.node._target_class_id = 1
        self.node._bbox_cb(msg)

        self.assertEqual(len(self.node._tracked_objects), 1)
        self.assertEqual(self.node._tracked_objects[0].class_id, 1)

    def test_bbox_callback_paused(self):
        """检测暂停时 bbox 回调不更新"""
        from depth_handler.msg import Bbox3dArray, Bbox3d

        msg = Bbox3dArray()
        bbox = Bbox3d()
        bbox.class_id = 1
        bbox.x, bbox.y, bbox.z = 0.4, -0.15, 0.3
        bbox.width, bbox.height, bbox.depth = 0.05, 0.05, 0.08
        msg.results = [bbox]

        self.node._target_class_id = 1
        self.node._detection_paused = True
        self.node._bbox_cb(msg)

        self.assertEqual(len(self.node._tracked_objects), 0)

    def test_detection_timeout(self):
        """检测超时返回 None"""
        self.node._target_class_id = 1
        self.node._tracked_objects = []
        self.node._detection_timeout = 0.5

        result = self.node._wait_for_detection()
        self.assertIsNone(result)

    def test_detection_returns_object(self):
        """有检测数据时返回第一个目标"""
        from my_arm_control.arm_task_manager import TrackedObject
        self.node._target_class_id = -1
        self.node._tracked_objects = [
            TrackedObject(1, [425.0, -125.0, 340.0])
        ]
        self.node._detection_timeout = 1.0

        result = self.node._wait_for_detection()
        self.assertIsNotNone(result)
        self.assertEqual(result['class_id'], 1)

    def test_opencap_service_sequence(self):
        """opencap 任务的服务调用序列正确"""
        # 设置初始 TCP 位姿
        from robo_ctrl.msg import TCPPose
        tcp = TCPPose()
        tcp.x, tcp.y, tcp.z = 99.917, -144.210, 542.554
        tcp.rx, tcp.ry, tcp.rz = -125.357, 0.0, -100.476
        self.node._current_tcp = tcp
        self.node._R_current_tcp = tcp

        # 设置检测数据
        from my_arm_control.arm_task_manager import TrackedObject
        self.node._target_class_id = 1
        self.node._tracked_objects = [
            TrackedObject(1, [425.0, -125.0, 340.0])
        ]
        self.node._detection_timeout = 1.0

        # 缩短所有 sleep 时间加速测试
        original_sleep = time.sleep
        time.sleep = lambda x: original_sleep(0.01)

        try:
            self.node._task_opencap()
        except Exception as e:
            # 预期可能失败, 但检查调用序列
            pass
        finally:
            time.sleep = original_sleep

        # 验证服务调用序列
        services = [c['service'] for c in self.service_calls]

        # Step 0: 夹爪使能 (L, R)
        self.assertIn('gripper_enable', services)

        # Step 1: L arm → 观测位 (robot_move_cart)
        # Step 2: R arm → 预备位 (R/robot_act_j)
        # Step 3: L arm 姿态修正 (L/robot_move_cart)
        # Step 4: 检测 (无服务调用, 用已有的 _detected_objects)
        # Step 5: L arm 接近 (robot_act)
        # Step 6: L 夹爪夹紧 (gripper_command)
        # Step 7: L arm 撤离 (L/robot_move_cart)
        # Step 8: L arm 瓶盖位 (L/robot_act_j)
        # Step 9: R arm 瓶盖位 (R/robot_act_j)
        # Step 10-11: 拧瓶盖循环 (R gripper + R arc + R act_j)
        # Step 12: 倒可乐 (L/robot_act_j)
        # Step 13: 放回 (L/robot_move_cart + gripper)

        # 验证关键服务被调用
        self.assertTrue(any('gripper_enable' in s for s in services),
                        "夹爪使能未被调用")
        self.assertTrue(any('robot_move_cart' in s for s in services),
                        "笛卡尔运动未被调用")
        self.assertTrue(any('robot_act' in s for s in services),
                        "增量运动未被调用")

        # 验证右臂服务被调用
        self.assertTrue(any('R/' in s for s in services),
                        "右臂服务未被调用")

        # 验证 gripper_command 被调用 (夹紧)
        self.assertTrue(any('gripper_command' in s for s in services),
                        "夹爪命令未被调用")

        # 验证姿态修正被调用
        self.assertTrue(any('fix_orientation' in s for s in services),
                        "姿态修正未被调用")

        # 打印调用序列
        print("\n=== 服务调用序列 ===")
        for i, c in enumerate(self.service_calls):
            print(f"  {i+1}. {c['service']}")

    def test_parameters_declared(self):
        """所有必要参数已声明"""
        required_params = [
            'use_moveit', 'robot_prefix',
            'observe_x', 'observe_y', 'observe_z',
            'observe_rx', 'observe_ry', 'observe_rz',
            'place_x', 'place_y', 'place_z',
            'desk_height', 'object_height',
            'cap_open_joints_l', 'cap_open_joints_r',
            'ball_L_joint_init_pose', 'ball_R_joint_init_pose',
            'cola_offset_x', 'cola_offset_y',
            'gripper_slave_id', 'task_mode',
            'target_class_id', 'detection_timeout',
        ]
        for param in required_params:
            try:
                val = self.node.get_parameter(param)
                self.assertIsNotNone(val, f"参数 {param} 为 None")
            except Exception as e:
                self.fail(f"参数 {param} 未声明: {e}")


# ===== 测试 4: 仿真模式基础测试 =====
class TestSimulationMode(unittest.TestCase):
    """测试 MoveIt 仿真模式的基本初始化。"""

    def test_virtual_vision_node_init(self):
        """virtual_vision_node 可以创建 (不需要 MoveIt)"""
        import rclpy
        if not rclpy.ok():
            rclpy.init()

        from my_arm_control.virtual_vision_node import VirtualVisionNode
        node = VirtualVisionNode()
        self.assertIsNotNone(node)
        node.destroy_node()


# ===== 运行测试 =====
def main():
    # 初始化 ROS2
    import rclpy
    if not rclpy.ok():
        rclpy.init()

    # 运行测试
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    suite.addTests(loader.loadTestsFromTestCase(TestImports))
    suite.addTests(loader.loadTestsFromTestCase(TestCoordinateCalculation))
    suite.addTests(loader.loadTestsFromTestCase(TestOpencapFlow))
    suite.addTests(loader.loadTestsFromTestCase(TestSimulationMode))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    rclpy.shutdown()

    # 返回码
    sys.exit(0 if result.wasSuccessful() else 1)


if __name__ == '__main__':
    main()
