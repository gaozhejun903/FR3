#!/usr/bin/env python3
"""
键盘控制机器人TCP位姿程序
控制方式：
- w/s: 前进/后退 (X轴)
- a/d: 左移/右移 (Y轴)  
- space/shift: 上升/下降 (Z轴)
- q/e: 绕Z轴旋转 (Rz)
- ↑/↓: 绕Y轴旋转 (Ry)
- ←/→: 绕X轴旋转 (Rx)
- 数字键: 设置步进值 (mm/度)
- r: 重置到零位
- ESC: 退出程序
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from robo_ctrl.srv import RobotServoLine
import sys
import termios
import tty
import select
import threading
import time
import math

class KeyboardTCPController(Node):
    def __init__(self):
        super().__init__('keyboard_tcp_controller')
        
        # 创建服务客户端
        self.servo_client = self.create_client(RobotServoLine, '/L/robot_servo_line')
        
        # 等待服务可用
        while not self.servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待机器人服务可用...')
        
        # 当前TCP位姿 [x, y, z, rx, ry, rz]
        self.current_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # 步进值设置 (mm for translation, degrees for rotation)
        self.linear_step = 1.0   # mm
        self.angular_step = 1.0  # degrees
        
        # 运动参数
        self.velocity = 20.0     # 速度百分比
        self.acceleration = 20.0 # 加速度百分比
        self.cmd_time = 0.008    # 指令周期
        self.filter_time = 0.0   # 滤波时间
        
        # 键盘控制状态
        self.running = True
        self.servo_started = False
        
        # 保存终端设置
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('键盘TCP控制器已启动')
        self.print_help()
    
    def __del__(self):
        # 恢复终端设置
        if hasattr(self, 'old_settings'):
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def print_help(self):
        """打印帮助信息"""
        help_text = """
=== 键盘TCP位姿控制器 ===
控制键说明：
  w/s     : 前进/后退 (X轴) 
  a/d     : 左移/右移 (Y轴)
  space/shift : 上升/下降 (Z轴)
  q/e     : 绕Z轴旋转 (Rz)
  ↑/↓     : 绕Y轴旋转 (Ry) 
  ←/→     : 绕X轴旋转 (Rx)
  
设置键：
  1-9     : 设置步进值 (1-9 mm/度)
  0       : 设置步进值为 10 mm/度
  r       : 重置到零位
  ESC     : 退出程序
  
当前设置：
  线性步进: {:.1f} mm
  角度步进: {:.1f} 度
  速度: {:.1f}%
  加速度: {:.1f}%
=============================
        """.format(self.linear_step, self.angular_step, self.velocity, self.acceleration)
        print(help_text)
    
    def get_key(self):
        """获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key
    
    def get_special_key(self):
        """获取特殊键（方向键等）"""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        if ord(key) == 27:  # ESC序列
            key += sys.stdin.read(2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key
    
    def start_servo_mode(self):
        """启动伺服模式"""
        if not self.servo_started:
            request = RobotServoLine.Request()
            request.command_type = 0  # ServoMoveStart
            request.acc = self.acceleration
            request.vel = self.velocity
            request.cmd_time = self.cmd_time
            request.filter_time = self.filter_time
            request.gain = 0.0
            request.point_count = 0
            request.use_incremental = True  # 使用增量模式
            
            # 创建空的位姿数组
            request.cartesian_pose = PoseArray()
            
            future = self.servo_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result().success:
                self.servo_started = True
                self.get_logger().info('伺服模式已启动')
            else:
                self.get_logger().error(f'启动伺服模式失败: {future.result().message}')
    
    def stop_servo_mode(self):
        """停止伺服模式"""
        if self.servo_started:
            request = RobotServoLine.Request()
            request.command_type = 1  # ServoMoveEnd
            
            future = self.servo_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result().success:
                self.servo_started = False
                self.get_logger().info('伺服模式已停止')
            else:
                self.get_logger().error(f'停止伺服模式失败: {future.result().message}')
    
    def send_incremental_pose(self, delta_pose):
        """发送增量位姿指令"""
        if not self.servo_started:
            self.start_servo_mode()
        
        # 创建增量位姿
        pose = Pose()
        pose.position.x = delta_pose[0] / 1000.0  # mm转m
        pose.position.y = delta_pose[1] / 1000.0
        pose.position.z = delta_pose[2] / 1000.0
        
        # 转换成四元数
        roll = math.radians(delta_pose[3])
        pitch = math.radians(delta_pose[4])
        yaw = math.radians(delta_pose[5])
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose.orientation.z = cr * cp * sy - sr * sp * cy
        pose.orientation.w = cr * cp * cy + sr * sp * sy
        
        # 创建请求
        request = RobotServoLine.Request()
        request.command_type = 0  # ServoMoveStart
        request.cartesian_pose = PoseArray()
        request.cartesian_pose.poses = [pose]
        request.acc = self.acceleration
        request.vel = self.velocity
        request.cmd_time = self.cmd_time
        request.filter_time = self.filter_time
        request.gain = 0.0
        request.point_count = 1
        request.use_incremental = True
        
        # 发送请求
        future = self.servo_client.call_async(request)
        
        # 更新当前位姿
        for i in range(6):
            self.current_pose[i] += delta_pose[i]
    
    def reset_pose(self):
        """重置到零位"""
        self.get_logger().info('重置到零位...')
        reset_pose = [-x for x in self.current_pose]
        self.send_incremental_pose(reset_pose)
        self.current_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    def process_key(self, key):
        """处理键盘输入"""
        delta_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # 处理移动键
        if key == 'w':
            delta_pose[0] = self.linear_step  # +X
        elif key == 's':
            delta_pose[0] = -self.linear_step  # -X
        elif key == 'a':
            delta_pose[1] = self.linear_step  # +Y
        elif key == 'd':
            delta_pose[1] = -self.linear_step  # -Y
        elif key == ' ':  # space
            delta_pose[2] = self.linear_step  # +Z
        elif key == 'z':  # 使用z键代替shift键实现下降
            delta_pose[2] = -self.linear_step  # -Z
        
        # 处理旋转键
        elif key == 'q':
            delta_pose[5] = self.angular_step  # +Rz
        elif key == 'e':
            delta_pose[5] = -self.angular_step  # -Rz
        
        # 处理方向键
        elif ord(key) == 27:  # ESC序列开始
            return 'special'
        
        # 处理数字键设置步进值
        elif key.isdigit():
            if key == '0':
                self.linear_step = 10.0
                self.angular_step = 10.0
            else:
                step_value = float(key)
                self.linear_step = step_value
                self.angular_step = step_value
            self.get_logger().info(f'步进值设置为: {self.linear_step} mm / {self.angular_step} 度')
            return None
        
        # 处理特殊键
        elif key == 'r':
            self.reset_pose()
            return None
        elif ord(key) == 27:  # ESC
            return 'quit'
        else:
            return None
        
        # 发送运动指令
        if any(abs(x) > 0.001 for x in delta_pose):
            self.send_incremental_pose(delta_pose)
            self.get_logger().info(f'当前位姿: X={self.current_pose[0]:.1f}, Y={self.current_pose[1]:.1f}, Z={self.current_pose[2]:.1f}, Rx={self.current_pose[3]:.1f}, Ry={self.current_pose[4]:.1f}, Rz={self.current_pose[5]:.1f}')
        
        return None
    
    def handle_arrow_keys(self):
        """处理方向键"""
        key_sequence = self.get_special_key()
        delta_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        if len(key_sequence) == 3 and key_sequence[0] == '\x1b' and key_sequence[1] == '[':
            if key_sequence[2] == 'A':  # ↑
                delta_pose[4] = self.angular_step  # +Ry
            elif key_sequence[2] == 'B':  # ↓
                delta_pose[4] = -self.angular_step  # -Ry
            elif key_sequence[2] == 'C':  # →
                delta_pose[3] = self.angular_step  # +Rx
            elif key_sequence[2] == 'D':  # ←
                delta_pose[3] = -self.angular_step  # -Rx
        elif key_sequence == '\x1b':  # 单独的ESC键
            return 'quit'
        
        # 发送运动指令
        if any(abs(x) > 0.001 for x in delta_pose):
            self.send_incremental_pose(delta_pose)
            self.get_logger().info(f'当前位姿: X={self.current_pose[0]:.1f}, Y={self.current_pose[1]:.1f}, Z={self.current_pose[2]:.1f}, Rx={self.current_pose[3]:.1f}, Ry={self.current_pose[4]:.1f}, Rz={self.current_pose[5]:.1f}')
        
        return None
    
    def run(self):
        """主运行循环"""
        try:
            while self.running and rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = self.get_key()
                    
                    result = self.process_key(key)
                    if result == 'quit':
                        break
                    elif result == 'special':
                        result = self.handle_arrow_keys()
                        if result == 'quit':
                            break
                
                # 处理ROS事件
                rclpy.spin_once(self, timeout_sec=0.01)
        
        except KeyboardInterrupt:
            pass
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        self.get_logger().info('正在退出...')
        self.stop_servo_mode()
        self.running = False
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = KeyboardTCPController()
        controller.run()
    except Exception as e:
        print(f"程序运行出错: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
