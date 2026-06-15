# AI-Deep: 一体化启动文件 + 状态检查
# 同时启动 depth_processor.launch.py 和 depth_tf.launch.py 的内容
# 注: depth_processor.launch.py 已包含 depth_tf.launch.py 的全部内容 (静态TF + 深度处理)
#      因此本文件以 depth_processor.launch.py 为基础，加入启动后系统状态检查

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # AI-Deep: 夹爪设备权限
    password = '123'
    for dev in ['/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2']:
        if os.path.exists(dev):
            subprocess.run(
                ['sudo', '-S', 'chmod', '777', dev],
                input=password + '\n', encoding='utf-8',
                capture_output=True
            )

    # ═══════════════════════════════════════════════════════════════
    # AI-Deep: 清理残留进程，防止占用机械臂SDK连接
    # 如果上次启动未正常退出 (kill -9 或崩溃)，robo_ctrl_node 仍持有
    # Fairino RPC 连接，导致新进程被控制器拒绝 (Connection reset by peer)
    # ═══════════════════════════════════════════════════════════════
    STALE_PROCS = [
        'robo_ctrl_node',
        'high_level_node',
        'epg50_gripper_node',
        'fake_gripper_tf_publisher_node',
        'static_tf_publisher_node',
        'detector_node_exe',
        'depth_processor_node',
        'camera_info_interceptor_node',
        'joint_state_merger',
        'static_transform_publisher',
    ]
    for proc in STALE_PROCS:
        subprocess.run(['pkill', '-9', '-f', proc], capture_output=True)
    # 等待内核回收资源，确保 TCP 连接完全释放
    subprocess.run(['sleep', '1.5'], capture_output=True)

    # ═══════════════════════════════════════════════════════════════
    # 左臂参数 (192.168.58.2)
    # ═══════════════════════════════════════════════════════════════
    L_ip_arg = DeclareLaunchArgument(
        'L_robot_ip', default_value='192.168.58.2',
        description='左臂控制器IP地址'
    )
    # ═══════════════════════════════════════════════════════════════
    # 右臂参数 (192.168.58.3)
    # ═══════════════════════════════════════════════════════════════
    R_ip_arg = DeclareLaunchArgument(
        'R_robot_ip', default_value='192.168.58.3',
        description='右臂控制器IP地址'
    )
    # ═══════════════════════════════════════════════════════════════
    # 夹爪端口参数 (by-path 持久化路径，重启不变)
    # ═══════════════════════════════════════════════════════════════
    R_gripper_port_arg = DeclareLaunchArgument(
        'R_gripper_port',
        default_value='/dev/serial/by-path/pci-0000:05:00.4-usb-0:1.2:1.0-port0',
        description='右夹爪串口 (by-path) — USB拓扑变更后更新 2026-06-06'
    )
    L_gripper_port_arg = DeclareLaunchArgument(
        'L_gripper_port',
        default_value='/dev/serial/by-path/pci-0000:05:00.4-usb-0:1.4:1.0-port0',
        description='左夹爪串口 (by-path) — USB拓扑变更后更新 2026-06-06'
    )

    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.4',
        description='目标检测置信度阈值'
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/mihu/FR3_again/src/detector/best2.engine',
        description='YOLO模型路径'
    )

    # ═══════════════════════════════════════════════════════════════
    # 相机 + 静态TF
    # ═══════════════════════════════════════════════════════════════
    orbbec_launch_dir = get_package_share_directory('orbbec_camera')
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_launch_dir, 'launch', 'gemini_330_series.launch.py')
        )
    )

    tools_launch_dir = get_package_share_directory('tools')
    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tools_launch_dir, 'launch', 'static_tf_multiple.launch.py')
        )
    )

    # ═══════════════════════════════════════════════════════════════
    # 左臂 (L) 控制节点
    # ═══════════════════════════════════════════════════════════════
    L_robo_ctrl_node = Node(
        package='robo_ctrl',
        executable='robo_ctrl_node',
        name='Lrobo_ctrl',
        parameters=[{
            'robot_ip': LaunchConfiguration('L_robot_ip'),
            'robot_name': 'L',
        }],
        output='screen',
    )
    L_high_level_node = Node(
        package='robo_ctrl',
        executable='high_level_node',
        name='Lhigh_level',
        parameters=[{
            'robot_ip': LaunchConfiguration('L_robot_ip'),
            'robot_name': 'L',
        }],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    # 右臂 (R) 控制节点
    # ═══════════════════════════════════════════════════════════════
    R_robo_ctrl_node = Node(
        package='robo_ctrl',
        executable='robo_ctrl_node',
        name='Rrobo_ctrl',
        parameters=[{
            'robot_ip': LaunchConfiguration('R_robot_ip'),
            'robot_name': 'R',
        }],
        output='screen',
    )
    R_high_level_node = Node(
        package='robo_ctrl',
        executable='high_level_node',
        name='Rhigh_level',
        parameters=[{
            'robot_ip': LaunchConfiguration('R_robot_ip'),
            'robot_name': 'R',
        }],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    # 左夹爪节点
    # ═══════════════════════════════════════════════════════════════
    L_gripper_node = Node(
        package='epg50_gripper_ros',
        executable='epg50_gripper_node',
        name='L_gripper_node',
        parameters=[{
            'port': LaunchConfiguration('L_gripper_port'),
            'robot_name': 'L',
        }],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    # 右夹爪节点
    # ═══════════════════════════════════════════════════════════════
    R_gripper_node = Node(
        package='epg50_gripper_ros',
        executable='epg50_gripper_node',
        name='R_gripper_node',
        parameters=[{
            'port': LaunchConfiguration('R_gripper_port'),
            'robot_name': 'R',
        }],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    # AI-Deep: 虚假夹爪TF发布节点 — 为high_level轨迹规划提供水平参考系
    # ═══════════════════════════════════════════════════════════════
    L_fake_gripper_tf_node = Node(
        package='tools',
        executable='fake_gripper_tf_publisher_node',
        name='Lfake_gripper_tf_publisher',
        parameters=[{
            'gripper_frame': 'Lgripper_tip',
            'base_frame': 'Lrobot_base',
            'fake_frame': 'Lfake_gripper_frame',
            'reference_frame': 'world',
            'robot_name': 'L',
            'rate': 50.0,
        }],
        output='screen',
    )

    R_fake_gripper_tf_node = Node(
        package='tools',
        executable='fake_gripper_tf_publisher_node',
        name='Rfake_gripper_tf_publisher',
        parameters=[{
            'gripper_frame': 'Rgripper_tip',
            'base_frame': 'Rrobot_base',
            'fake_frame': 'Rfake_gripper_frame',
            'reference_frame': 'world',
            'robot_name': 'R',
            'rate': 50.0,
        }],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    # 视觉节点
    # ═══════════════════════════════════════════════════════════════
    detector_node = Node(
        package='detector',
        executable='detector_node_exe',
        name='detector_node',
        parameters=[{
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'model_path': LaunchConfiguration('model_path'),
        }],
        output='screen',
    )

    depth_node = Node(
        package='depth_handler',
        executable='depth_processor_node',
        name='depth_handler_node',
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════
    # AI-Deep: 启动后系统状态检查
    #
    # 改进说明 (2026-06-15):
    #   1. subprocess 继承 ROS 2 环境变量 (os.environ.copy())
    #   2. 每次 check 都重新查询 ros2 node/topic list (不再缓存一次性快照)
    #   3. check_node / check_topic / check_topic_data 均带重试
    #   4. 失败时打印当前节点列表和 stderr 帮助排查
    #   5. 总超时约 60 秒 (15s 初始 + 3轮*2s 重试 + topic data 超时)
    # ═══════════════════════════════════════════════════════════════
    status_checker_script = '''
import subprocess, sys, time, os

OK    = "[  OK  ]"
FAIL  = "[ FAIL ]"
WARN  = "[ WARN ]"

# ═══════════════════════════════════════════════════════════════
# 初始等待: 相机 connection_delay=10s + 机械臂TCP握手 + 模型加载
# ═══════════════════════════════════════════════════════════════
time.sleep(15)

# 继承父进程的完整环境变量，确保 ros2 命令能找到正确的 DDS / domain
_ROS_ENV = os.environ.copy()

def _ros_cmd(cmd, timeout=8):
    """执行 ros2 命令，返回 (stdout, stderr, ok)"""
    try:
        r = subprocess.run(
            cmd, shell=True, capture_output=True, text=True,
            timeout=timeout, env=_ROS_ENV
        )
        return r.stdout, r.stderr, r.returncode == 0
    except subprocess.TimeoutExpired:
        return "", "timeout", False
    except Exception as e:
        return "", str(e), False

def _get_nodes():
    """返回当前所有节点名的 set (每次调用都重新查询)"""
    out, err, ok = _ros_cmd("ros2 node list 2>&1")
    if not ok:
        print(f"  {WARN}  ros2 node list 失败: {err.strip()[:120]}")
        return set()
    return {n.strip() for n in out.strip().split("\\n") if n.strip()}

def _get_topics():
    """返回当前所有 topic 名的 set (每次调用都重新查询)"""
    out, err, ok = _ros_cmd("ros2 topic list 2>&1")
    if not ok:
        print(f"  {WARN}  ros2 topic list 失败: {err.strip()[:120]}")
        return set()
    return {t.strip() for t in out.strip().split("\\n") if t.strip()}

def check_node(name, retries=3, delay=2.0):
    """
    检查节点是否在线。失败时重新 ros2 node list 并重试。
    retries=0 表示只查一次不重试 (用于夹爪失败后的二次确认)
    """
    for attempt in range(1, max(retries, 1) + 1):
        nodes = _get_nodes()
        if name in nodes:
            msg = f"  {OK}  node : {name}"
            if attempt > 1:
                msg += f" (第{attempt}次重试成功)"
            print(msg)
            return True
        if attempt < retries:
            print(f"  ...  node {name} 未出现，{delay:.0f}s 后刷新重试 ({attempt}/{retries})")
            time.sleep(delay)
    # 最终失败 — 打印当前节点列表方便定位
    nodes = _get_nodes()
    print(f"  {FAIL}  node : {name}  <-- 未找到!")
    if nodes:
        print(f"         | 当前在线节点({len(nodes)}): {', '.join(sorted(nodes)[:25])}")
    else:
        print(f"         | ros2 node list 返回为空! 检查 ROS_DOMAIN_ID / 网络 / 是否 source 了 setup.bash")
    return False

def check_topic(name, retries=2, delay=2.0):
    """检查 topic 是否已注册 (仅检查注册，不检查数据)"""
    for attempt in range(1, max(retries, 1) + 1):
        topics = _get_topics()
        if name in topics:
            msg = f"  {OK}  topic: {name}"
            if attempt > 1:
                msg += f" (第{attempt}次重试成功)"
            print(msg)
            return True
        if attempt < retries:
            time.sleep(delay)
    print(f"  {FAIL}  topic: {name}  <-- 未找到!")
    return False

def check_serial_port(port_path):
    """检查串口设备文件是否存在"""
    if os.path.exists(port_path):
        print(f"  {OK}  port: {port_path}")
        return True
    else:
        print(f"  {FAIL}  port: {port_path}  <-- 设备不存在!")
        return False

def get_gripper_node_log(name):
    """如果夹爪节点未启动，抓取其最近的错误日志"""
    try:
        result = subprocess.run(
            f"ros2 topic echo --once /rosout 2>/dev/null | grep -i -A2 'gripper\\|epg50\\|serial\\|tty' | tail -20",
            shell=True, capture_output=True, text=True, timeout=5, env=_ROS_ENV
        )
        if result.stdout.strip():
            for line in result.stdout.strip().split("\\n")[-5:]:
                print(f"         | {line.strip()}")
        else:
            print(f"         | (无相关日志输出 — 节点可能启动时崩溃)")
    except:
        pass

def check_topic_data(name, timeout_sec=5, retries=2, delay=2.0):
    """检查 topic 是否有实际数据流出 (ros2 topic echo --once)，带重试"""
    for attempt in range(1, max(retries, 1) + 1):
        # 每轮都确认 topic 已注册
        topics = _get_topics()
        if name not in topics:
            if attempt < retries:
                print(f"  ...  topic {name} 尚未注册，{delay:.0f}s 后重试 ({attempt}/{retries})")
                time.sleep(delay)
                continue
            print(f"  {FAIL}  data : {name}  <-- 未注册!")
            return False
        try:
            result = subprocess.run(
                f"timeout {timeout_sec} ros2 topic echo {name} --once 2>/dev/null",
                shell=True, capture_output=True, text=True,
                timeout=timeout_sec + 3, env=_ROS_ENV
            )
            if result.returncode == 0 and result.stdout.strip():
                msg = f"  {OK}  data : {name}"
                if attempt > 1:
                    msg += f" (第{attempt}次重试成功)"
                print(msg)
                return True
            if attempt < retries:
                print(f"  ...  topic {name} 已注册但无数据，{delay:.0f}s 后重试 ({attempt}/{retries})")
                time.sleep(delay)
        except:
            if attempt < retries:
                time.sleep(delay)
    print(f"  {FAIL}  data : {name}  <-- 无数据!")
    return False

def section(title):
    print(f"\\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")

print(f"\\n{'#'*60}")
print(f"#           系统启动状态检查")
print(f"{'#'*60}")

pass_count = 0
fail_count = 0
warn_count = 0
failed_components = []
warned_components = []

# 1) 左臂
section("左臂机械臂 (192.168.58.2)")
left_ok = True
check_node("Lrobo_ctrl") or (left_ok := False)
check_node("Lhigh_level") or (left_ok := False)
check_topic_data("/L/joint_states", timeout_sec=3) or (left_ok := False)
if left_ok:
    print(f"  ==> 左臂: [SUCCESS]")
    pass_count += 1
else:
    print(f"  ==> 左臂: [FAILED]")
    fail_count += 1
    failed_components.append("左臂机械臂")

# 2) 右臂
section("右臂机械臂 (192.168.58.3)")
right_ok = True
check_node("Rrobo_ctrl") or (right_ok := False)
check_node("Rhigh_level") or (right_ok := False)
check_topic_data("/R/joint_states", timeout_sec=3) or (right_ok := False)
if right_ok:
    print(f"  ==> 右臂: [SUCCESS]")
    pass_count += 1
else:
    print(f"  ==> 右臂: [FAILED]")
    fail_count += 1
    failed_components.append("右臂机械臂")

# 3) 左夹爪 — 夹爪可能未连接，失败仅报警告不阻断
section("左夹爪")
lg_ok = True
check_serial_port("/dev/serial/by-path/pci-0000:05:00.4-usb-0:1.4:1.0-port0") or (lg_ok := False)
check_node("L_gripper_node") or (lg_ok := False)
check_topic("/L/gripper/status_stream") or (lg_ok := False)
if lg_ok:
    print(f"  ==> 左夹爪: [SUCCESS]")
    pass_count += 1
else:
    # 二次确认节点是否真的不在线 (不做重试，只看当前状态)
    nodes = _get_nodes()
    if "L_gripper_node" not in nodes:
        print(f"  {WARN}  左夹爪节点未启动 — 可能串口打开失败，抓取最近日志:")
        get_gripper_node_log("L_gripper_node")
    print(f"  {WARN}  左夹爪状态获取失败 — 检查串口连接/供电")
    print(f"  ==> 左夹爪: [WARN]  (非致命，继续运行)")
    warn_count += 1
    warned_components.append("左夹爪")

# 4) 右夹爪 — 夹爪可能未连接，失败仅报警告不阻断
section("右夹爪")
rg_ok = True
check_serial_port("/dev/serial/by-path/pci-0000:05:00.4-usb-0:1.2:1.0-port0") or (rg_ok := False)
check_node("R_gripper_node") or (rg_ok := False)
check_topic("/R/gripper/status_stream") or (rg_ok := False)
if rg_ok:
    print(f"  ==> 右夹爪: [SUCCESS]")
    pass_count += 1
else:
    nodes = _get_nodes()
    if "R_gripper_node" not in nodes:
        print(f"  {WARN}  右夹爪节点未启动 — 可能串口打开失败，抓取最近日志:")
        get_gripper_node_log("R_gripper_node")
    print(f"  {WARN}  右夹爪状态获取失败 — 检查串口连接/供电")
    print(f"  ==> 右夹爪: [WARN]  (非致命，继续运行)")
    warn_count += 1
    warned_components.append("右夹爪")

# 5) 目标检测
section("目标检测 (YOLO)")
det_ok = True
check_node("detector_node") or (det_ok := False)
check_topic("/detector/detections") or (det_ok := False)
if det_ok:
    print(f"  ==> 目标检测: [SUCCESS]")
    pass_count += 1
else:
    print(f"  ==> 目标检测: [FAILED]")
    fail_count += 1
    failed_components.append("目标检测(YOLO)")

# 6) 深度处理
section("深度处理")
dep_ok = True
check_node("depth_handler_node") or (dep_ok := False)
# bbox3d / pointcloud 依赖相机图像，相机未就绪时无数据属于正常
has_bbox3d = check_topic_data("/depth_handler/bbox3d", timeout_sec=3)
has_pc = check_topic_data("/depth_handler/pointcloud", timeout_sec=3)
if not has_bbox3d and not has_pc:
    print(f"  {WARN}  深度处理 topic 无数据 — 可能相机未就绪")
# 深度处理节点存活即算 OK (数据依赖上游相机)
if dep_ok:
    print(f"  ==> 深度处理: [SUCCESS]")
    pass_count += 1
else:
    print(f"  ==> 深度处理: [FAILED]")
    fail_count += 1
    failed_components.append("深度处理")

# 7) TF发布
section("TF发布")
tf_ok = True
check_node("camera_static_tf_publisher") or (tf_ok := False)
check_node("gripper_static_tf_publisher") or (tf_ok := False)
check_node("Lfake_gripper_tf_publisher") or (tf_ok := False)
check_node("Rfake_gripper_tf_publisher") or (tf_ok := False)
if tf_ok:
    print(f"  ==> TF发布: [SUCCESS]")
    pass_count += 1
else:
    print(f"  ==> TF发布: [FAILED]")
    fail_count += 1
    failed_components.append("TF发布")

# 8) 相机 — 必须有实际图像数据才算通过
section("相机 (Orbbec Gemini 335)")
cam_ok = True
(check_node("ob_camera_node") or check_node("camera_container")) or (cam_ok := False)
print(f"  等待相机推流 (最长 10 秒)...")
check_topic_data("/camera/color/image_raw", timeout_sec=10) or (cam_ok := False)
check_topic_data("/camera/depth/image_raw", timeout_sec=10) or (cam_ok := False)
if cam_ok:
    print(f"  ==> 相机: [SUCCESS]")
    pass_count += 1
else:
    print(f"  ==> 相机: [FAILED]  <-- 无图像数据，检查USB线缆/USB口")
    fail_count += 1
    failed_components.append("相机(Orbbec)")

# 汇总
total = pass_count + fail_count + warn_count
print(f"\\n{'#'*60}")
print(f"#           最终检查结果")
print(f"{'#'*60}")
print(f"  共检查 {total} 项 : 成功 {pass_count} / 失败 {fail_count} / 警告 {warn_count}")
print()
if fail_count == 0 and warn_count == 0:
    print(f"  >>> 所有组件启动成功! <<<")
else:
    if fail_count > 0:
        print(f"  >>> 以下 {fail_count} 个组件启动失败，请检查日志: <<<")
        for i, comp in enumerate(failed_components, 1):
            print(f"      {i}. [FAIL] {comp}")
    if warn_count > 0:
        print(f"  >>> 以下 {warn_count} 个组件有警告 (非致命)，请检查连接: <<<")
        for i, comp in enumerate(warned_components, 1):
            print(f"      {i}. [WARN] {comp}")
print()
'''

    status_checker = ExecuteProcess(
        cmd=['python3', '-c', status_checker_script],
        output='screen',
        name='system_status_checker',
    )

    return LaunchDescription([
        L_ip_arg,
        R_ip_arg,
        L_gripper_port_arg,
        R_gripper_port_arg,
        confidence_threshold_arg,
        model_path_arg,
        camera_launch,
        static_tf_launch,
        L_robo_ctrl_node,
        L_high_level_node,
        R_robo_ctrl_node,
        R_high_level_node,
        L_gripper_node,
        R_gripper_node,
        L_fake_gripper_tf_node,
        R_fake_gripper_tf_node,
        detector_node,
        depth_node,
        status_checker,
    ])
