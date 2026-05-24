import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class VirtualVisionTask1(Node):
    """任务一虚拟视觉节点: 发布瓶子和杯子的假坐标。

    发布话题:
      /vision/bottle_water  (PoseStamped) - 怡宝水瓶位置
      /vision/bottle_cola   (PoseStamped) - 可乐瓶位置
      /vision/cup_1         (PoseStamped) - 水杯1位置
      /vision/cup_2         (PoseStamped) - 水杯2位置
    """

    def __init__(self):
        super().__init__('virtual_vision_task1')

        self.declare_parameter('bottle_water_x', 250.0)
        self.declare_parameter('bottle_water_y', -200.0)
        self.declare_parameter('bottle_water_z', 360.0)

        self.declare_parameter('bottle_cola_x', 300.0)
        self.declare_parameter('bottle_cola_y', -100.0)
        self.declare_parameter('bottle_cola_z', 360.0)

        self.declare_parameter('cup_1_x', 200.0)
        self.declare_parameter('cup_1_y', -150.0)
        self.declare_parameter('cup_1_z', 360.0)

        self.declare_parameter('cup_2_x', 280.0)
        self.declare_parameter('cup_2_y', -250.0)
        self.declare_parameter('cup_2_z', 360.0)

        self.declare_parameter('frame_id', 'camera_depth_frame')
        self.declare_parameter('rate', 5.0)

        self._frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('rate').value

        self._objects = {
            'bottle_water': {
                'x': self.get_parameter('bottle_water_x').value,
                'y': self.get_parameter('bottle_water_y').value,
                'z': self.get_parameter('bottle_water_z').value,
            },
            'bottle_cola': {
                'x': self.get_parameter('bottle_cola_x').value,
                'y': self.get_parameter('bottle_cola_y').value,
                'z': self.get_parameter('bottle_cola_z').value,
            },
            'cup_1': {
                'x': self.get_parameter('cup_1_x').value,
                'y': self.get_parameter('cup_1_y').value,
                'z': self.get_parameter('cup_1_z').value,
            },
            'cup_2': {
                'x': self.get_parameter('cup_2_x').value,
                'y': self.get_parameter('cup_2_y').value,
                'z': self.get_parameter('cup_2_z').value,
            },
        }

        self._vision_publishers = {}
        for name in self._objects:
            self._vision_publishers[name] = self.create_publisher(
                PoseStamped, f'/vision/{name}', 10
            )

        self._timer = self.create_timer(1.0 / rate, self._publish_all)

        self.get_logger().info('任务一虚拟视觉节点已启动')
        for name, pos in self._objects.items():
            self.get_logger().info(
                f'  {name}: ({pos["x"]:.1f}, {pos["y"]:.1f}, {pos["z"]:.1f}) mm'
            )

    def _publish_all(self):
        now = self.get_clock().now().to_msg()
        for name, pos in self._objects.items():
            msg = PoseStamped()
            msg.header.stamp = now
            msg.header.frame_id = self._frame_id
            msg.pose.position.x = pos['x']
            msg.pose.position.y = pos['y']
            msg.pose.position.z = pos['z']
            msg.pose.orientation.w = 1.0
            self._vision_publishers[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualVisionTask1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
