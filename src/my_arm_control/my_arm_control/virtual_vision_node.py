import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter


class VirtualVisionNode(Node):
    """模拟视觉节点：沿圆弧轨迹定时发布 target_pose，使机械臂平滑圆弧运动。"""

    def __init__(self):
        super().__init__('virtual_vision_node')

        # ----- 圆弧轨迹参数 -----
        self.declare_parameter('center_x', 0.4)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('center_z', 0.5)
        self.declare_parameter('radius', 0.15)
        self.declare_parameter('angular_velocity', 0.3)   # rad/s
        self.declare_parameter('plane', 'xz')              # xy / xz / yz
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('rate', 10.0)               # 内部发布频率 Hz
        self.declare_parameter('orientation_x', 0.0)
        self.declare_parameter('orientation_y', 0.0)
        self.declare_parameter('orientation_z', 0.0)
        self.declare_parameter('orientation_w', 1.0)

        # ----- 读取参数 -----
        self._cx = self.get_parameter('center_x').value
        self._cy = self.get_parameter('center_y').value
        self._cz = self.get_parameter('center_z').value
        self._radius = self.get_parameter('radius').value
        self._angular_vel = self.get_parameter('angular_velocity').value
        self._plane = self.get_parameter('plane').value
        self._frame_id = self.get_parameter('frame_id').value
        self._ox = self.get_parameter('orientation_x').value
        self._oy = self.get_parameter('orientation_y').value
        self._oz = self.get_parameter('orientation_z').value
        self._ow = self.get_parameter('orientation_w').value
        rate = self.get_parameter('rate').value

        # ----- 角度状态（随时间累积）-----
        self._angle = 0.0
        self._last_time = self.get_clock().now()

        # ----- 创建发布器 & 定时器 -----
        self._publisher = self.create_publisher(PoseStamped, 'target_pose', 10)
        period = 1.0 / rate
        self._timer = self.create_timer(period, self._publish_pose)

        self.get_logger().info(
            f'虚拟视觉节点(圆弧模式)已启动: '
            f'center=({self._cx:.3f}, {self._cy:.3f}, {self._cz:.3f}), '
            f'radius={self._radius:.3f}, '
            f'angular_velocity={self._angular_vel:.3f} rad/s, '
            f'plane={self._plane}, rate={rate} Hz'
        )

    def _on_param_change(self, params):
        """参数动态更新回调"""
        for p in params:
            if p.name == 'center_x':
                self._cx = p.value
            elif p.name == 'center_y':
                self._cy = p.value
            elif p.name == 'center_z':
                self._cz = p.value
            elif p.name == 'radius':
                self._radius = p.value
            elif p.name == 'angular_velocity':
                self._angular_vel = p.value
            elif p.name == 'plane':
                self._plane = p.value
            elif p.name == 'frame_id':
                self._frame_id = p.value
            elif p.name in ('orientation_x', 'orientation_y',
                            'orientation_z', 'orientation_w'):
                setattr(self, f'_o{p.name[-1]}', p.value)
            elif p.name == 'rate':
                period = 1.0 / p.value
                self._timer.timer_period_ns = int(period * 1e9)

        self.get_logger().info(
            f'参数已更新: center=({self._cx:.3f}, {self._cy:.3f}, {self._cz:.3f}), '
            f'radius={self._radius:.3f}, plane={self._plane}'
        )
        return rclpy.node.SetParametersResult(successful=True)

    def _publish_pose(self):
        """定时发布圆弧轨迹上的下一个点"""
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now

        # 角度累积，保持 [0, 2π)
        self._angle += self._angular_vel * dt
        self._angle = math.fmod(self._angle, 2 * math.pi)

        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._frame_id

        # 根据平面计算圆弧上的位置
        c, s = math.cos(self._angle), math.sin(self._angle)
        if self._plane == 'xy':
            msg.pose.position.x = self._cx + self._radius * c
            msg.pose.position.y = self._cy + self._radius * s
            msg.pose.position.z = self._cz
        elif self._plane == 'yz':
            msg.pose.position.x = self._cx
            msg.pose.position.y = self._cy + self._radius * c
            msg.pose.position.z = self._cz + self._radius * s
        else:  # xz plane (默认 — 机器人正前方的水平圆弧)
            msg.pose.position.x = self._cx + self._radius * c
            msg.pose.position.y = self._cy
            msg.pose.position.z = self._cz + self._radius * s

        msg.pose.orientation.x = self._ox
        msg.pose.orientation.y = self._oy
        msg.pose.orientation.z = self._oz
        msg.pose.orientation.w = self._ow

        self._publisher.publish(msg)

        self.get_logger().debug(
            f'发布 target_pose: pos=({msg.pose.position.x:.3f}, '
            f'{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}), '
            f'angle={self._angle:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = VirtualVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('虚拟视觉节点已手动关闭')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
