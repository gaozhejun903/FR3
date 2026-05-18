# TF点位置查询工具

这个工具提供了一个ROS2服务，用于查询一个坐标系下的点在另一个坐标系下的位置。适用于手动调试和测试坐标系变换。

## 功能特点

- 提供服务，查询一个坐标系下的点在另一个坐标系下的位置
- 支持可配置的超时时间
- 提供命令行客户端工具，方便使用
- 可通过ROS2 launch文件快速启动

## 安装和构建

确保您已经将本仓库克隆到您的ROS2工作空间，然后执行以下命令来构建：

```bash
cd /path/to/your/workspace
colcon build --packages-select tools
source install/setup.bash
```

## 使用方法

### 1. 启动服务节点

使用launch文件启动服务节点：

```bash
ros2 launch tools tf_point_query.launch.py
```

### 2. 使用命令行客户端

使用提供的Python客户端脚本查询坐标变换：

```bash
# 基本用法
ros2 run tools tf_query_client.py <源坐标系> <目标坐标系> <x> <y> <z>

# 例如，查询点(1.0, 2.0, 3.0)从"base_link"坐标系到"camera_link"坐标系的变换
ros2 run tools tf_query_client.py base_link camera_link 1.0 2.0 3.0

# 指定超时时间（默认为1秒）
ros2 run tools tf_query_client.py base_link camera_link 1.0 2.0 3.0 --timeout 2.0
```

### 3. 通过ROS2服务API调用

您也可以在您的ROS2节点中直接调用服务：

```python
# Python示例
from tools.srv import TransformPoint
from geometry_msgs.msg import Point

# 创建客户端
client = node.create_client(TransformPoint, 'transform_point')

# 准备请求
request = TransformPoint.Request()
request.source_frame = 'base_link'
request.target_frame = 'camera_link'
request.point = Point(x=1.0, y=2.0, z=3.0)
request.timeout = 1.0

# 发送请求
future = client.call_async(request)
```

```cpp
// C++示例
#include "tools/srv/transform_point.hpp"
#include <geometry_msgs/msg/point.hpp>

// 创建客户端
auto client = node->create_client<tools::srv::TransformPoint>("transform_point");

// 准备请求
auto request = std::make_shared<tools::srv::TransformPoint::Request>();
request->source_frame = "base_link";
request->target_frame = "camera_link";
request->point.x = 1.0;
request->point.y = 2.0;
request->point.z = 3.0;
request->timeout = 1.0;

// 发送请求
auto future = client->async_send_request(request);
```

## 服务定义

服务定义如下：

```
# 请求部分：包含源坐标系、目标坐标系和要转换的点
string source_frame           # 源坐标系
string target_frame           # 目标坐标系
geometry_msgs/Point point     # 源坐标系中的点
float64 timeout               # 等待变换可用的超时时间（秒），默认为1.0
---
# 响应部分：包含转换后的点和是否成功的标志
geometry_msgs/PointStamped transformed_point  # 转换后的点（带时间戳和坐标系id）
bool success                                 # 转换是否成功
string error_msg                             # 如果失败，错误信息
```

## 常见问题

- **服务无响应**：确保TF树中存在从源坐标系到目标坐标系的变换路径。
- **变换超时**：增加超时时间或检查TF树中是否缺少坐标系的发布。
- **坐标系名称错误**：确保使用了正确的坐标系名称，注意大小写和前导斜杠。