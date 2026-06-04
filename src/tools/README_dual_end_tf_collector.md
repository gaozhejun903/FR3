# Dual End TF Collector

双臂末端TF数据收集器，用于从TF中订阅Rend和Lend的位置信息，并将它们成对保存用于双臂协作标定。

## 功能特点

- 从TF中实时获取Rend和Lend的位置信息
- 支持手动触发和自动采集模式
- 保存包含位置、姿态和欧拉角的完整位姿数据
- 计算双臂末端之间的相对位置关系
- 支持JSON格式的数据输出

## 使用方法

### 1. 启动节点

```bash
# 使用默认参数启动
ros2 launch tools dual_end_tf_collector.launch.py

# 使用自定义参数启动
ros2 launch tools dual_end_tf_collector.launch.py \
    save_path:=/home/user/dual_end_data \
    auto_capture:=true \
    auto_capture_interval:=3000 \
    rend_frame_id:=right_end_effector \
    lend_frame_id:=left_end_effector
```

### 2. 手动触发数据采集

```bash
ros2 service call /dual_end_tf_collector/capture_dual_end_sample std_srvs/srv/Trigger
```

### 3. 自动采集模式

设置 `auto_capture:=true` 并指定 `auto_capture_interval` 即可启用自动采集。

## 参数说明

- `save_path`: 数据保存路径，默认为 `/tmp/dual_end_tf_data`
- `auto_capture`: 是否启用自动采集，默认为 `false`
- `auto_capture_interval`: 自动采集间隔（毫秒），默认为 `2000`
- `tf_update_interval`: TF数据更新间隔（毫秒），默认为 `100`
- `rend_frame_id`: 右臂末端坐标系名称，默认为 `Rend`
- `lend_frame_id`: 左臂末端坐标系名称，默认为 `Lend`
- `base_frame_id`: 基座坐标系名称，默认为 `robot_base_link`

## 输出数据格式

数据保存为JSON格式，包含以下信息：

```json
{
  "timestamp": 1625097600000000000,
  "sample_index": 0,
  "base_frame_id": "robot_base_link",
  "rend_pose": {
    "frame_id": "robot_base_link",
    "position": {"x": 0.5, "y": 0.0, "z": 0.3},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "euler": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
  },
  "lend_pose": {
    "frame_id": "robot_base_link",
    "position": {"x": -0.5, "y": 0.0, "z": 0.3},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "euler": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
  },
  "relative_info": {
    "distance": 1.0,
    "delta_x": -1.0,
    "delta_y": 0.0,
    "delta_z": 0.0
  }
}
```

## 服务接口

- `/dual_end_tf_collector/capture_dual_end_sample` (std_srvs/srv/Trigger): 触发单次数据采集

## 依赖项

- ROS2 
- tf2_ros
- geometry_msgs
- std_srvs
- nlohmann_json

## 编译

```bash
cd /path/to/your/workspace
colcon build --packages-select tools
```

## 注意事项

1. 确保TF中存在指定的Rend和Lend坐标系
2. 确保TF变换链路完整，能够从base_frame_id到end_frame_id的变换
3. 在自动采集模式下，建议适当调整采集间隔以避免过于频繁的数据保存
4. 数据文件按照 `dual_pose_XXXX.json` 格式命名，其中XXXX为4位数字的样本编号
