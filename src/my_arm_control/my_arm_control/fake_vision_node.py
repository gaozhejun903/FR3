"""
假视觉节点 — 发布固定的 Bbox3dArray 用于实机运动测试。

发布 /depth_handler/bbox3d, 使 arm_task_manager 能收到目标并执行运动。
坐标为 Lrobot_base 坐标系, 单位: 米。
"""

import rclpy
from rclpy.node import Node
from depth_handler.msg import Bbox3d, Bbox3dArray
from std_msgs.msg import Header


class FakeVisionNode(Node):

    def __init__(self):
        super().__init__("fake_vision_node")

        self.declare_parameter("target_x", 0.4)    # 米
        self.declare_parameter("target_y", -0.15)   # 米
        self.declare_parameter("target_z", 0.3)     # 米
        self.declare_parameter("width", 0.05)
        self.declare_parameter("height", 0.05)
        self.declare_parameter("depth", 0.1)
        self.declare_parameter("class_id", 1)
        self.declare_parameter("rate", 10.0)        # Hz
        self.declare_parameter("publish_once", False)  # 持续发布

        self._x = self.get_parameter("target_x").value
        self._y = self.get_parameter("target_y").value
        self._z = self.get_parameter("target_z").value
        self._w = self.get_parameter("width").value
        self._h = self.get_parameter("height").value
        self._d = self.get_parameter("depth").value
        self._cid = self.get_parameter("class_id").value
        self._once = self.get_parameter("publish_once").value
        rate = self.get_parameter("rate").value

        self._pub = self.create_publisher(Bbox3dArray, "/depth_handler/bbox3d", 10)

        if self._once:
            # 延迟 2s 发一次, 确保 arm_task_manager 已就绪
            self.create_timer(2.0, self._publish_once)
        else:
            self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f"假视觉节点启动 | 目标=({self._x}, {self._y}, {self._z}) m "
            f"| once={self._once}"
        )

    def _publish_once(self):
        self._publish()
        self.get_logger().info("已发布一次假目标, 停止发布")
        # 取消 timer
        for timer in self._timers if hasattr(self, '_timers') else []:
            timer.cancel()
        # 直接 shutdown
        rclpy.shutdown()

    def _publish(self):
        bbox = Bbox3d()
        bbox.x = self._x - self._w / 2.0
        bbox.y = self._y - self._h / 2.0
        bbox.z = self._z - self._d / 2.0
        bbox.width = self._w
        bbox.height = self._h
        bbox.depth = self._d
        bbox.class_id = self._cid

        msg = Bbox3dArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "Lrobot_base"
        msg.results = [bbox]

        self._pub.publish(msg)
        self.get_logger().info(
            f"发布假目标: ({self._x:.3f}, {self._y:.3f}, {self._z:.3f}) m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FakeVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
