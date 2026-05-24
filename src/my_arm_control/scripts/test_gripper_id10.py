#!/usr/bin/env python3
"""测试 ID=10 (0x0A) 夹爪是否可以启动。

用法:
    python3 test_gripper_id10.py [/dev/ttyACM0]
"""

import sys
import os

# 把 my_arm_control 加入路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from my_arm_control.gripper_controller import GripperController


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    slave_id = 0x0A  # ID = 10

    print(f"=== 测试夹爪 ID=10 (0x0A)  端口: {port} ===\n")

    # 1. 初始化控制器 (使用 ID=9 作为默认，因为共享串口)
    try:
        ctrl = GripperController(port=port, slave_id=0x09)
    except Exception as e:
        print(f"[FAIL] 无法打开串口: {e}")
        return

    # 2. 读取 ID=10 的状态
    print("[1] 读取 ID=10 状态...")
    status = ctrl.read_status(slave_id=slave_id)
    if status is None:
        print("[FAIL] ID=10 无响应 — 请检查:")
        print("  - 夹爪是否上电")
        print("  - RS485/USB 线缆是否连接")
        print("  - ID 是否确实是 10 (可用重命名工具确认)")
        ctrl.close_serial()
        return

    print(f"[OK] ID=10 响应成功!")
    print(f"  位置={status.position}  速度={status.speed}  力={status.force}")
    print(f"  电压={status.voltage}  温度={status.temperature}")
    print(f"  状态位: {status.bits}")
    print(f"  使能(gact)={status.bits.gact}  激活状态(gsta)={status.bits.gsta}")
    print(f"  错误: {status.error_message}")

    # 3. 尝试使能
    print("\n[2] 尝试使能 ID=10...")
    ok = ctrl.enable(slave_id=slave_id)
    if ok:
        print("[OK] ID=10 使能成功!")
    else:
        print("[FAIL] ID=10 使能失败")
        ctrl.close_serial()
        return

    # 4. 再次读取状态确认使能
    import time
    time.sleep(0.5)
    status2 = ctrl.read_status(slave_id=slave_id)
    if status2:
        print(f"\n[3] 使能后状态:")
        print(f"  使能(gact)={status2.bits.gact}  激活状态(gsta)={status2.bits.gsta}")
        print(f"  {status2.bits.gripper_status_str}")
        if status2.bits.gact and status2.bits.gsta == 3:
            print("\n=== ID=10 夹爪启动成功，已激活就绪! ===")
        elif status2.bits.gact:
            print("\n=== ID=10 已使能，正在激活中... ===")
        else:
            print("\n=== ID=10 使能后状态异常 ===")

    # 5. 打开夹爪测试
    print("\n[4] 测试打开夹爪...")
    ctrl.open(slave_id=slave_id)
    time.sleep(1.0)
    status3 = ctrl.read_status(slave_id=slave_id)
    if status3:
        print(f"  打开后位置: {status3.position}")

    ctrl.close_serial()
    print("\n=== 测试完成 ===")


if __name__ == "__main__":
    main()
