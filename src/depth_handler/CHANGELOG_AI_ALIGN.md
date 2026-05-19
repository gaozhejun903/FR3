# AI-Deep 图像对齐修改记录

## 修改概述

为 `depth_handler` 包添加**彩色图像到深度图像的像素级对齐**功能，解决原来简单分辨率缩放无法补偿不同相机内参和外参差异的问题。

## 修改动机

- 彩色相机和深度相机的内参不同（fx、fy、cx、cy 均不同）
- 两相机之间存在物理基线（外参 R、t），产生视差
- 原代码仅按分辨率比例缩放 2D 检测框（如 `1280→848`），未考虑内参和外参差异
- 导致 2D 检测框映射到深度图时对应错误的像素位置，3D 坐标产生偏差

## 修改文件

### 1. `include/depth_handler/depth_processor_node.hpp`

| 位置 | 修改内容 |
|------|----------|
| 第54-60行 | 添加彩色相机内参成员变量（`color_camera_info_sub_`、`fx_c_`/`fy_c_`/`cx_c_`/`cy_c_`），以及深度相机内参提取变量（`fx_d_`/`fy_d_`/`cx_d_`/`cy_d_`） |
| 第70-71行 | 添加 `color_camera_info_topic_` 参数（默认 `/camera/color/camera_info`） |
| 第100-103行 | 添加外参的 Eigen 矩阵形式 `R_c2d_`、`t_c2d_`（`P_d = R * P_c + t`）和初始化标志 `extrinsics_initialized_` |
| 第126-136行 | 在深度 `camera_info_callback` 中提取内参并初始化外参 Eigen 矩阵 |
| 第146-161行 | 添加彩色 `camera_info_callback`，订阅一次后自动取消 |
| 第258-269行 | 声明 `alignBboxToDepth()` 函数 |

### 2. `src/depth_processor_node.cpp`

| 位置 | 修改内容 |
|------|----------|
| 第32-37行 | 构造函数中订阅 `/camera/color/camera_info` |
| 第69行 | 更新注释，说明通过 `alignBboxToDepth` 做像素级对齐 |
| 第78-87行 | 将 `cv::Mat depth_img` 构造移到 ROI 计算之前（对齐需要深度值） |
| 第91-97行 | 用 `alignBboxToDepth` 替代原有 `scale_bbox`，保留 `scale_bbox` 作为回退方案 |
| 第269-360行 | 实现 `alignBboxToDepth()` 函数 |
| 第568、595、620行 | 添加 `color_camera_info_topic` 参数的声明、读取和日志输出 |

## 对齐算法（`alignBboxToDepth`）

```
输入: color图像上的2D检测框 (x, y, width, height)
输出: depth图像上对齐后的ROI

步骤:
  1. 用分辨率比例估算bbox中心在depth图中的位置
  2. 从depth图读取该位置的粗略深度值
  3. 用color内参将bbox四个角点反投影为3D点 (在color相机坐标系)
     P_c = [ (u-cx_c)/fx_c * z,  (v-cy_c)/fy_c * z,  z ]
  4. 通过外参变换到depth相机坐标系
     P_d = R_c2d * P_c + t_c2d
  5. 用depth内参投影回depth图像平面
     u_d = (P_d.x / P_d.z) * fx_d + cx_d
     v_d = (P_d.y / P_d.z) * fy_d + cy_d
  6. 取投影角点的外接矩形 + 10像素边距作为对齐后的ROI

回退策略:
  - 若color camera_info未就绪 → 回退到简单分辨率缩放 (scale_bbox)
  - 若所有角点投影失败(在depth相机后方) → 回退到简单缩放
```

## 依赖关系

- 新依赖话题: `/camera/color/camera_info`（`sensor_msgs/msg/CameraInfo`）
- 其余依赖不变

## 新增参数

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `color_camera_info_topic` | string | `/camera/color/camera_info` | 彩色相机内参话题 |

## 外参说明

外参 `r[9]` 和 `t[3]` 定义在头文件中（第96-99行），表示 **color→depth** 坐标变换：
```
P_depth = R_c2d * P_color + t_c2d
```

当前为硬编码值，如需更改请修改头文件中的 `r` 和 `t` 数组。
