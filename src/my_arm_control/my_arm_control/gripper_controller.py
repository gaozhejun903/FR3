"""
EPG50 夹爪 Python 控制器 (Modbus RTU / 115200 8N1)

直接通过串口发送 Modbus 帧控制夹爪开合，
同时可选通过 ROS 话题监测抓取状态。

用法::

    # 纯串口模式 (无需 ROS)
    ctrl = GripperController("/dev/ttyACM0", slave_id=0x09)
    ctrl.open()
    ctrl.wait_until_grabbed()
    ctrl.close()
    ctrl.close_serial()

    # ROS 节点内使用 (import 本模块)
    ctrl = GripperController("/dev/ttyACM0", slave_id=0x09)
    ctrl.open()
    # 使用 ROS 订阅监测
    GripperController.wait_until_grabbed_ros(node, topic="gripper_status_stream")
    ctrl.close()
"""

import struct
import logging
import threading
import time as _time
from typing import Optional

import serial

# ============================================================
# Logger —— 终端彩色输出，方便用户观察
# ============================================================
logger = logging.getLogger("Gripper")
logger.setLevel(logging.DEBUG)
if not logger.handlers:
    _h = logging.StreamHandler()
    _h.setFormatter(logging.Formatter(
        "[%(name)s] %(asctime)s.%(msecs)03d %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    ))
    logger.addHandler(_h)


# ============================================================
# Modbus CRC16 (多项式 0xA001, 同 C++ 实现)
# ============================================================
def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


# ============================================================
# 状态位解析 (同 C++ GripperStatusBits)
# ============================================================
class GripperStatusBits:
    """从 status 低字节解析出的各个状态位。"""

    __slots__ = ("gact", "gmod", "ggto", "gsta", "gobj")

    def __init__(self, status_byte: int):
        self.gact: bool = bool(status_byte & 0x01)   # bit0: 使能
        self.gmod: bool = bool(status_byte & 0x04)   # bit2: 工作模式
        self.ggto: bool = bool(status_byte & 0x08)   # bit3: 动作状态
        self.gsta: int  = (status_byte & 0x30) >> 4  # bits4-5: 夹爪状态
        self.gobj: int  = (status_byte & 0xC0) >> 6  # bits6-7: 目标检测

    @property
    def gripper_status_str(self) -> str:
        parts = [
            "已使能" if self.gact else "未使能/复位中",
            "无输入参数控制模式" if self.gmod else "参数控制模式",
            "前往目标位置" if self.ggto else "停止/执行激活或巡检",
        ]
        parts.append({0: "复位或巡检状态", 1: "正在激活",
                       2: "未使用状态", 3: "激活完成"}.get(self.gsta, "未知"))
        return ", ".join(parts)

    @property
    def object_status_str(self) -> str:
        return {
            0: "手指正向指定位置移动",
            1: "手指张开过程中接触到物体并停止",
            2: "手指闭合过程中接触到物体并停止",
            3: "已到达指定位置，但未检测到物体或物体已脱落",
        }.get(self.gobj, "未知状态")

    def __repr__(self) -> str:
        return (f"gact={int(self.gact)} gmod={int(self.gmod)} "
                f"ggto={int(self.ggto)} gsta={self.gsta} gobj={self.gobj}")


class EPG50Status:
    """单次读取返回的完整夹爪状态 (8 个字段)。"""

    __slots__ = ("status", "mode", "error", "position", "speed",
                 "force", "voltage", "temperature", "bits")

    def __init__(self, raw: list[int]):
        # 0x07D0 / 0x07D1 / 0x07D2 / 0x07D3  各寄存器的低高字节
        self.status: int      = raw[0]   # 夹爪状态
        self.mode: int        = raw[1]   # 留空
        self.error: int       = raw[2]   # 故障错误
        self.position: int    = raw[3]   # 位置
        self.speed: int       = raw[4]   # 速度
        self.force: int       = raw[5]   # 力(即时电流)
        self.voltage: int     = raw[6]   # 母线电压
        self.temperature: int = raw[7]   # 环境温度
        self.bits = GripperStatusBits(self.status & 0xFF)

    @property
    def error_message(self) -> str:
        e = self.error & 0xFF
        for bit, msg in [(0x01, "通讯异常"), (0x02, "控制指令错误"),
                         (0x04, "过温故障"), (0x08, "电压异常"),
                         (0x10, "过流故障")]:
            if e & bit:
                return msg
        return "正常"

    def __repr__(self) -> str:
        return (f"pos={self.position:3d} speed={self.speed:3d} "
                f"force={self.force:3d} volt={self.voltage} "
                f"temp={self.temperature} | {self.bits}")


# ============================================================
# 主控制器
# ============================================================
class GripperController:
    """EPG50 夹爪控制器 (Modbus RTU)。

    参数
    ----------
    port : str
        串口路径，默认 ``/dev/ttyACM0``。
    slave_id : int
        默认从站 ID，默认 ``0x09``。
    """

    WRITE_REG = 0x03E8   # 写寄存器首地址
    READ_REG  = 0x07D0   # 读寄存器首地址

    def __init__(self, port: str = "/dev/ttyACM0", slave_id: int = 0x09):
        self._port = port
        self._slave_id = slave_id
        self._serial: Optional[serial.Serial] = None
        self._open_serial()
        logger.info("============================================")
        logger.info("  GripperController 初始化完成")
        logger.info(f"  端口:      {port}")
        logger.info(f"  从站 ID:   0x{slave_id:02X}")
        logger.info(f"  波特率:    115200 8N1")
        logger.info("============================================")

    # -----------------------------------------------------------------
    # 串口管理
    # -----------------------------------------------------------------
    def _open_serial(self) -> None:
        if self._serial and self._serial.is_open:
            return
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.5,
                write_timeout=0.5,
                xonxoff=False,
                rtscts=False,
            )
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            logger.info(f"串口 {self._port} 打开成功")
        except serial.SerialException as e:
            logger.error(f"打开串口失败: {e}")
            raise

    def close_serial(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()
            logger.info("串口已关闭")

    # -----------------------------------------------------------------
    # Modbus 底层
    # -----------------------------------------------------------------
    def _send_cmd(self, cmd: bytes) -> Optional[bytes]:
        logger.debug(f"  >> TX: {cmd.hex(' ')}")
        try:
            self._serial.reset_input_buffer()
            self._serial.write(cmd)
            self._serial.flush()
        except serial.SerialException as e:
            logger.error(f"串口写入失败: {e}")
            return None
        resp = self._serial.read(256)
        if not resp:
            logger.warning("响应超时 (500ms)")
            return None
        logger.debug(f"  << RX: {resp.hex(' ')}")
        return resp

    def _fc16(self, reg: int, values: list[int]) -> bytes:
        """FC16 写多个寄存器。"""
        n = len(values)
        buf = bytearray([self._slave_id, 0x10,
                         (reg >> 8) & 0xFF, reg & 0xFF,
                         (n >> 8) & 0xFF, n & 0xFF,
                         n * 2])
        for v in values:
            buf.append((v >> 8) & 0xFF)
            buf.append(v & 0xFF)
        crc = _crc16(bytes(buf))
        buf.append(crc & 0xFF)
        buf.append((crc >> 8) & 0xFF)
        return bytes(buf)

    def _fc03(self, reg: int, count: int) -> bytes:
        """FC03 读多个寄存器。"""
        buf = bytearray([self._slave_id, 0x03,
                         (reg >> 8) & 0xFF, reg & 0xFF,
                         (count >> 8) & 0xFF, count & 0xFF])
        crc = _crc16(bytes(buf))
        buf.append(crc & 0xFF)
        buf.append((crc >> 8) & 0xFF)
        return bytes(buf)

    def _check_fc16_ack(self, resp: Optional[bytes], reg: int,
                        count: int) -> bool:
        if resp is None or len(resp) < 6:
            logger.error("FC16 响应长度不足")
            return False
        expected = bytearray([self._slave_id, 0x10,
                              (reg >> 8) & 0xFF, reg & 0xFF,
                              (count >> 8) & 0xFF, count & 0xFF])
        crc = _crc16(bytes(expected))
        expected.append(crc & 0xFF)
        expected.append((crc >> 8) & 0xFF)
        if resp != bytes(expected):
            logger.error(f"FC16 响应不匹配: 期望={expected.hex(' ')}, "
                         f"收到={resp.hex(' ')}")
            return False
        return True

    def _with_sid(self, slave_id: Optional[int], fn):
        """临时切换 slave_id 执行操作后恢复。"""
        old = self._slave_id
        if slave_id is not None:
            self._slave_id = slave_id
        try:
            return fn()
        finally:
            self._slave_id = old

    # -----------------------------------------------------------------
    # 命令
    # -----------------------------------------------------------------
    def enable(self, slave_id: Optional[int] = None) -> bool:
        """使能夹爪 (写 0x03E8 = 0x0001)。"""
        return self._with_sid(slave_id, lambda: self._enable())

    def _enable(self) -> bool:
        logger.info(f"使能夹爪 [0x{self._slave_id:02X}] ...")
        cmd = self._fc16(self.WRITE_REG, [0x0001])
        ok = self._check_fc16_ack(self._send_cmd(cmd), self.WRITE_REG, 1)
        logger.info(f"  夹爪 0x{self._slave_id:02X} {'使能成功' if ok else '使能失败'}")
        return ok

    def disable(self, slave_id: Optional[int] = None) -> bool:
        """禁用夹爪 (写 0x03E8 = 0x0000)。"""
        return self._with_sid(slave_id, lambda: self._disable())

    def _disable(self) -> bool:
        logger.info(f"禁用夹爪 [0x{self._slave_id:02X}] ...")
        cmd = self._fc16(self.WRITE_REG, [0x0000])
        ok = self._check_fc16_ack(self._send_cmd(cmd), self.WRITE_REG, 1)
        logger.info(f"  夹爪 0x{self._slave_id:02X} {'已禁用' if ok else '禁用失败'}")
        return ok

    def set_params(self, position: int, speed: int = 255, torque: int = 255,
                   slave_id: Optional[int] = None) -> bool:
        """设置位置/速度/力矩 (FC16 写 3 个寄存器)。

        position : 0-255  (0=全开, 255=全闭)
        """
        return self._with_sid(slave_id, lambda: self._set_params(position, speed, torque))

    def _set_params(self, pos: int, speed: int, torque: int) -> bool:
        logger.info(f">>> 设置 0x{self._slave_id:02X}: pos={pos} speed={speed} torque={torque}")
        # 同 C++: regs = [0x0009, position, (speed<<8)|torque]
        cmd = self._fc16(self.WRITE_REG, [0x0009, pos, (speed << 8) | torque])
        ok = self._check_fc16_ack(self._send_cmd(cmd), self.WRITE_REG, 3)
        if ok:
            logger.info(f"  ✓ 参数设置成功: pos={pos}, speed={speed}, torque={torque}")
        else:
            logger.error(f"  ✗ 参数设置失败: pos={pos}, speed={speed}, torque={torque}")
        return ok

    def open(self, slave_id: Optional[int] = None) -> bool:
        """打开夹爪 (position=0)。"""
        sid = slave_id or self._slave_id
        logger.info(f">>>>> 打开夹爪 [0x{sid:02X}] <<<<<")
        ok = self.set_params(0, 255, 255, slave_id=sid)
        logger.info(f">>>>> 夹爪 0x{sid:02X} {'已打开' if ok else '打开失败'} <<<<<")
        return ok

    def close(self, slave_id: Optional[int] = None) -> bool:
        """闭合夹爪 (position=255)。"""
        sid = slave_id or self._slave_id
        logger.info(f">>>>> 闭合夹爪 [0x{sid:02X}] <<<<<")
        ok = self.set_params(255, 255, 255, slave_id=sid)
        logger.info(f">>>>> 夹爪 0x{sid:02X} {'已闭合' if ok else '闭合失败'} <<<<<")
        return ok

    def read_status(self, slave_id: Optional[int] = None) -> Optional[EPG50Status]:
        """读取夹爪状态 (FC03 读 4 个寄存器 → 8 个字段)。"""
        return self._with_sid(slave_id, self._read_status)

    def _read_status(self) -> Optional[EPG50Status]:
        cmd = self._fc03(self.READ_REG, 4)
        resp = self._send_cmd(cmd)
        if resp is None or len(resp) < 13:
            logger.debug("读状态: 响应无效")
            return None
        # 解析顺序同 C++ serial.hpp
        raw = [
            resp[4],   # status  (低字节 reg0)
            resp[3],   # mode    (高字节 reg0)
            resp[6],   # error   (低字节 reg1)
            resp[5],   # position(高字节 reg1)
            resp[8],   # speed   (低字节 reg2)
            resp[7],   # force   (高字节 reg2)
            resp[10],  # voltage (低字节 reg3)
            resp[9],   # temp    (高字节 reg3)
        ]
        return EPG50Status(raw)

    # -----------------------------------------------------------------
    # 等待抓取 — 串口轮询版
    # -----------------------------------------------------------------
    def wait_until_grabbed(self, timeout: float = 5.0,
                           slave_id: Optional[int] = None) -> bool:
        """轮询串口, 等待夹爪抓住物体或超时。

        判定逻辑:
          - ``gobj == 2`` (闭合过程中接触物体) → 成功
          - ``gobj == 3`` (到达位置无物体)     → 失败
          - ``force >= 30``                     → 成功 (辅助判据)

        返回 ``True`` 表示抓取成功。
        """
        sid = slave_id or self._slave_id
        logger.info(f"=== 等待抓取 0x{sid:02X}  timeout={timeout}s ===")
        deadline = _time.monotonic() + timeout

        while _time.monotonic() < deadline:
            st = self.read_status(slave_id=sid)
            if st is None:
                _time.sleep(0.05)
                continue

            logger.debug(
                f"  pos={st.position:3d} force={st.force:3d}  "
                f"ggto={int(st.bits.ggto)} gobj={st.bits.gobj}")

            if st.bits.gobj == 2 or st.force >= 30:
                logger.info(
                    f"✓  抓取成功!  gobj={st.bits.gobj}  "
                    f"force={st.force}  pos={st.position}")
                return True
            if st.bits.gobj == 3:
                logger.warning(
                    f"✗  未抓到物体 gobj=3  force={st.force}  "
                    f"pos={st.position}")
                return False

            _time.sleep(0.05)

        logger.warning(f"✗  等待超时 ({timeout}s)")
        return False

    # -----------------------------------------------------------------
    # 等待抓取 — ROS 订阅版 (需要 rclpy + epg50_gripper_ros 消息)
    # -----------------------------------------------------------------
    @staticmethod
    def wait_until_grabbed_ros(
        node,
        topic: str = "gripper_status_stream",
        timeout: float = 5.0,
    ) -> bool:
        """通过 ROS 订阅夹爪状态话题等待抓取。

        需在已初始化的 rclpy 节点中调用。

        参数
        ----------
        node : rclpy.node.Node
        topic : str
            状态话题名 (epg50_gripper_ros 发布 ``gripper_status_stream``)。
        timeout : float
            超时秒数。

        返回 ``True`` 表示抓取成功。
        """
        from epg50_gripper_ros.msg import GripperStatus

        event = threading.Event()
        result = {"ok": False, "reason": ""}

        def cb(msg):
            g, f = msg.gobj, msg.force
            logger.debug(f"  [ROS] gobj={g} force={f} pos={msg.position}")
            if g == 2 or f >= 30:
                result["ok"] = True
                result["reason"] = f"gobj={g} force={f}"
                event.set()
            elif g == 3:
                result["ok"] = False
                result["reason"] = "未检测到物体 (gobj=3)"
                event.set()

        sub = node.create_subscription(GripperStatus, topic, cb, 10)
        logger.info(f"=== ROS 订阅等待抓取 topic={topic} timeout={timeout}s ===")

        grabbed = event.wait(timeout=timeout)
        node.destroy_subscription(sub)

        if grabbed and result["ok"]:
            logger.info(f"✓ [ROS] 抓取成功! {result['reason']}")
        else:
            logger.warning(f"✗ [ROS] 抓取失败/超时: {result['reason']}")

        return result["ok"]

    # -----------------------------------------------------------------
    def __del__(self):
        self.close_serial()
