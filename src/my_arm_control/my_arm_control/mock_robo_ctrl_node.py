import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import time
from robo_ctrl.srv import RobotMoveCart, RobotAct, RobotActJ, RobotServoLine, RobotServo, RobotSetSpeed
from robo_ctrl.msg import RobotState, TCPPose, JointPosition
from epg50_gripper_ros.srv import GripperCommand, GripperStatus
from epg50_gripper_ros.msg import GripperStatus as GripperStatusMsg


class MockRoboCtrlNode(Node):

    def __init__(self):
        super().__init__('mock_robo_ctrl_node')

        self.declare_parameter('robot_name', 'L')
        self._name = self.get_parameter('robot_name').value

        self._tcp_x = 168.0
        self._tcp_y = -102.0
        self._tcp_z = 394.0
        self._tcp_rx = -111.556
        self._tcp_ry = 0.0
        self._tcp_rz = -90.0

        self._motion_end_time = 0.0

        ns = f'/{self._name}' if self._name != '' else ''

        self._act_srv = self.create_service(RobotAct, f'{ns}/robot_act', self._handle_act)
        self._act_j_srv = self.create_service(RobotActJ, f'{ns}/robot_act_j', self._handle_act_j)
        self._move_cart_srv = self.create_service(RobotMoveCart, f'{ns}/robot_move_cart', self._handle_move_cart)
        self._servo_line_srv = self.create_service(RobotServoLine, f'{ns}/robot_servo_line', self._handle_servo_line)
        self._servo_srv = self.create_service(RobotServo, f'{ns}/robot_servo', self._handle_servo)
        self._set_speed_srv = self.create_service(RobotSetSpeed, f'{ns}/robot_set_speed', self._handle_set_speed)

        self._gripper_srv = self.create_service(GripperCommand, 'gripper_command', self._handle_gripper)
        self._gripper_status_srv = self.create_service(GripperStatus, 'gripper_status', self._handle_gripper_status)
        self._R_gripper_srv = self.create_service(GripperCommand, 'R_gripper_command', self._handle_gripper)
        self._R_gripper_status_srv = self.create_service(GripperStatus, 'R_gripper_status', self._handle_gripper_status)

        self._state_pub = self.create_publisher(RobotState, f'{ns}/robot_state', 10)
        self._timer = self.create_timer(0.01, self._publish_state)

        self.get_logger().info(
            f'模拟机器人节点已启动: name={self._name}, '
            f'services at {ns}/robot_act, {ns}/robot_act_j, {ns}/robot_move_cart'
        )

    def _publish_state(self):
        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_position = JointPosition(j1=0.0, j2=-66.994, j3=-51.016, j4=-87.069, j5=81.874, j6=-90.0)
        msg.tcp_pose = TCPPose(x=self._tcp_x, y=self._tcp_y, z=self._tcp_z,
                                rx=self._tcp_rx, ry=self._tcp_ry, rz=self._tcp_rz)
        msg.motion_done = (time.monotonic() >= self._motion_end_time)
        msg.error_code = 0
        self._state_pub.publish(msg)

    def _handle_act(self, request, response):
        if request.use_incremental:
            self._tcp_x += request.tcp_pose.x
            self._tcp_y += request.tcp_pose.y
            self._tcp_z += request.tcp_pose.z
            self._tcp_rx += request.tcp_pose.rx
            self._tcp_ry += request.tcp_pose.ry
            self._tcp_rz += request.tcp_pose.rz
        self.get_logger().info(
            f'RobotAct: cmd={request.command_type} '
            f'tcp=({request.tcp_pose.x:.1f}, {request.tcp_pose.y:.1f}, {request.tcp_pose.z:.1f}) '
            f'incremental={request.use_incremental}'
        )
        self._motion_end_time = time.monotonic() + 0.3
        response.success = True
        response.message = '模拟执行成功'
        return response

    def _handle_act_j(self, request, response):
        self.get_logger().info(
            f'RobotActJ: cmd={request.command_type} joints={[f"{j:.1f}" for j in request.target_joints]}'
        )
        self._motion_end_time = time.monotonic() + 0.3
        response.success = True
        response.message = '模拟执行成功'
        return response

    def _handle_move_cart(self, request, response):
        if request.use_increment:
            self._tcp_x += request.tcp_pose.x
            self._tcp_y += request.tcp_pose.y
            self._tcp_z += request.tcp_pose.z
        else:
            self._tcp_x = request.tcp_pose.x
            self._tcp_y = request.tcp_pose.y
            self._tcp_z = request.tcp_pose.z
            self._tcp_rx = request.tcp_pose.rx
            self._tcp_ry = request.tcp_pose.ry
            self._tcp_rz = request.tcp_pose.rz
        self.get_logger().info(
            f'RobotMoveCart: tcp=({request.tcp_pose.x:.1f}, {request.tcp_pose.y:.1f}, {request.tcp_pose.z:.1f}) '
            f'vel={request.velocity:.0f}% incremental={request.use_increment}'
        )
        self._motion_end_time = time.monotonic() + 0.3
        response.success = True
        response.message = '模拟执行成功'
        return response

    def _handle_servo_line(self, request, response):
        self.get_logger().info(f'RobotServoLine: cmd={request.command_type}')
        response.success = True
        response.message = '模拟执行成功'
        return response

    def _handle_servo(self, request, response):
        self.get_logger().info(f'RobotServo: cmd={request.command_type}')
        response.success = True
        response.message = '模拟执行成功'
        return response

    def _handle_set_speed(self, request, response):
        self.get_logger().info(f'RobotSetSpeed: speed={request.speed}%')
        response.success = True
        response.message = '模拟执行成功'
        return response

    def _handle_gripper(self, request, response):
        self.get_logger().info(
            f'GripperCommand: slave_id={request.slave_id} cmd={request.command} '
            f'pos={request.position} speed={request.speed} torque={request.torque}'
        )
        response.success = True
        response.message = '模拟夹爪执行成功'
        return response

    def _handle_gripper_status(self, request, response):
        response.success = True
        response.status = 0x07D0
        response.gact = True
        response.gmod = False
        response.ggto = False
        response.gsta = 3
        response.gobj = 3
        response.mode = 0
        response.error = 0
        response.position = 0
        response.speed = 0
        response.force = 0
        response.voltage = 24
        response.temperature = 25
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockRoboCtrlNode()
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
