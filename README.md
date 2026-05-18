# FairinoDualArm

## 概述
FairinoDualArm 是一个基于 ROS2 Humble 的双臂协作机器人控制包，集成了感知、运动规划与执行功能，可用于实现复杂的抓取、搬运与协作任务。

## 主要功能
- 传感器接入：支持深度相机、激光雷达输入
- 运动规划：基于 MoveIt2 实现双臂协同规划
- 抓取与放置：自定义抓取策略与末端工具
- 可视化调试：RViz 工具链支持

## 软件结构
```plaintext
src/
├── camera_info_interceptor    # 相机信息拦截与转换节点
├── depth_handler              # 深度图像处理节点
├── detector                   # 目标检测与定位节点
├── dualarm                    # 双臂运动规划与执行核心
├── epg50_gripper_ros          # EPG50 电磁夹爪驱动
├── robo_ctrl                  # 机器人整体控制管理节点
├── tools                      # 辅助脚本与工具
└── tf_node                    # TF 坐标变换广播节点
```

## 安装与构建
```bash
# 安装依赖
sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-moveit2 ros-humble-rviz2

# 克隆仓库
cd ~/ros2_ws/src
git clone <仓库地址> FairinoDualArm

# 构建
cd ~/ros2_ws
trt
colcon build --symlink-install
source install/setup.bash
```

# ！！！！！! 要跑的节点 ！！！！！！！！
```
ros2 launch robo_ctrl robo_ctrl.launch.py
ros2 run robo_ctrl high_level_node
ros2 launch orbbec_camera gemini_330_series.launch.py
ros2 run depth_handler depth_processor_node
ros2 run detector detector_node_exe
ros2 run tools tf_point_query_node
ros2 launch tools static_tf_publisher.launch.py
ros2 launch tools fake_gripper_tf_publisher.launch.py
ros2 run epg50_gripper_ros epg50_gripper_node
```

foxglove观察用（可选）
```
ros2 run camera_info_interceptor camera_info_interceptor_node
```

记得检查下夹爪的供电情况，大疆电池有休眠

## 需要的service和话题
机器人控制
```
/robot_state
/depth_handler/visualization 的 center_point
/transform_point
/epg50_gripper/command
/robot_move_cart
/robot_act
```