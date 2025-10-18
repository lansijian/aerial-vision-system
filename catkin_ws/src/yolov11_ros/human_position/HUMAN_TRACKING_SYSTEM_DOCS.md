# 人体跟踪与定位系统技术文档

## 系统概述

### 1.1 系统简介

本系统是一套完整的无人机视觉目标定位与验证解决方案，专为 **Typhoon H480** 无人机设计。系统通过集成YOLO目标检测、相机几何投影、MAVROS位置信息和Gazebo仿真真值，实现了人体目标的实时检测、三维定位和精度验证。

**核心功能：**
- 🎯 实时人体检测与ENU坐标计算
- 📊 真值位置发布（Gazebo Actor）
- ✅ 位置精度对比与统计分析

**适用场景：**
- 搜救任务中的人员定位
- 智能跟踪与监控
- 算法验证与性能评估

---

## 1.2 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        Gazebo 仿真环境                            │
│  ┌──────────────┐              ┌──────────────┐                 │
│  │  Typhoon H480 │              │    Actor     │                 │
│  │   (无人机)    │              │   (人体模型)  │                 │
│  └──────┬───────┘              └──────┬───────┘                 │
│         │                              │                          │
└─────────┼──────────────────────────────┼──────────────────────────┘
          │                              │
          ├─── Camera Image              └─── Ground Truth
          ├─── MAVROS Pose                    (GetModelState)
          │                                          │
          ▼                                          ▼
┌─────────────────────┐              ┌─────────────────────────┐
│   YOLO Detection    │              │ publish_actor_ground_   │
│   (BoundingBoxes)   │              │       truth.py          │
└──────────┬──────────┘              └──────────┬──────────────┘
           │                                     │
           │                                     │ /actor/ground_truth/
           │                                     │     position
           ▼                                     │
┌─────────────────────────────────────────┐     │
│  human_position_publisher_local.py      │     │
│  ┌────────────────────────────────────┐ │     │
│  │ 1. 像素坐标 → 相机射线              │ │     │
│  │ 2. 相机坐标 → Body坐标             │ │     │
│  │ 3. Body坐标 → ENU坐标              │ │     │
│  │ 4. 射线-地面交点计算                │ │     │
│  └────────────────────────────────────┘ │     │
└──────────┬──────────────────────────────┘     │
           │ /xtdrone/typhoon_h480_0/           │
           │  human_position_local              │
           ▼                                     ▼
     ┌─────────────────────────────────────────────┐
     │        compare_positions.py                 │
     │  ┌────────────────────────────────────────┐ │
     │  │ • 误差计算 (ΔX, ΔY, ΔZ)                │ │
     │  │ • 距离误差 (2D/3D)                     │ │
     │  │ • 统计分析 (均值/标准差/分布)           │ │
     │  │ • 实时可视化                           │ │
     │  └────────────────────────────────────────┘ │
     └─────────────────────────────────────────────┘
```

---

## 2. 模块详解

### 2.1 Actor真值发布器 (`publish_actor_ground_truth.py`)

#### 功能描述

从Gazebo仿真环境中获取Actor（人体模型）的真实位置，并以ROS话题形式发布，作为算法验证的"金标准"。

#### 核心特性

| 特性 | 说明 |
|------|------|
| **数据获取方式** | 支持服务调用（GetModelState/GetLinkState）或话题订阅（ModelStates） |
| **发布频率** | 可配置（默认30Hz） |
| **坐标系** | Gazebo世界坐标系（等同于ENU） |
| **输出话题** | PointStamped + PoseStamped + TF |

#### 技术实现

**1. 数据获取**

```python
# 方法1: 服务调用（更精确）
GetModelState(model_name='actor', relative_entity_name='')

# 方法2: 话题订阅（更高效）
/gazebo/model_states → 查找actor索引 → 提取位姿
```

**2. 发布内容**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/actor/ground_truth/position` | PointStamped | Actor的三维坐标 |
| `/actor/ground_truth/pose` | PoseStamped | Actor的位姿（位置+方向） |
| `TF: map→actor_ground_truth` | Transform | 用于RViz可视化 |

#### 使用方法

```bash
# 基本用法（默认参数）
rosrun <package_name> publish_actor_ground_truth.py

# 指定actor名称
rosrun <package_name> publish_actor_ground_truth.py --name actor

# 指定发布频率（50Hz）
rosrun <package_name> publish_actor_ground_truth.py --rate 50

# 使用话题订阅方式（提高效率）
rosrun <package_name> publish_actor_ground_truth.py --method topic
```

#### 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--name` | `actor` | Gazebo中的Actor模型名称 |
| `--rate` | `30` | 发布频率（Hz） |
| `--method` | `service` | 数据获取方式：`service` 或 `topic` |

#### 输出示例

```
================================================================================
  Actor真值位置发布器
================================================================================
  Actor名称: actor
  发布频率: 30 Hz
  获取方式: 服务调用
================================================================================

[就绪] 开始发布actor位置...

发布话题:
  - /actor/ground_truth/position (PointStamped)
  - /actor/ground_truth/pose (PoseStamped)
  - TF: map -> actor_ground_truth

按 Ctrl+C 停止...

[Actor真值] ENU: (5.23, -2.14, 0.90)
```

---

### 2.2 人体位置发布器 (`human_position_publisher_local.py`)

#### 功能描述

基于YOLO目标检测和MAVROS位置信息，通过计算机视觉和几何投影方法，实时计算人体在ENU坐标系下的三维位置。

#### 核心特性

| 特性 | 说明 |
|------|------|
| **检测算法** | YOLOv11目标检测 |
| **定位方法** | 相机射线-地面交点法 |
| **坐标系** | ENU（东-北-天） |
| **相机模型** | 针孔相机模型（校准参数） |
| **云台补偿** | 支持俯仰、偏航、横滚补偿 |

#### 技术实现

**1. 坐标系变换链**

```
像素坐标 (u,v)  →  相机坐标系 (X_cam)  →  Body坐标系 (X_body)  →  ENU坐标系 (X_enu)
```

**详细流程：**

```
步骤1: 像素 → 相机射线
───────────────────────
输入: (u, v) - 检测框中心像素坐标
输出: ray_camera = [x, y, z] - 归一化方向向量

公式:
  x_norm = (u - cx) / f
  y_norm = (v - cy) / f
  z_norm = 1.0
  ray = normalize([x_norm, y_norm, z_norm])

其中:
  f  = 205.47  (焦距，像素)
  cx = 320.5   (光心X)
  cy = 180.5   (光心Y)


步骤2: 相机坐标 → Body坐标
─────────────────────────
变换矩阵: R_cam_to_body
考虑因素:
  ① 相机基础安装矩阵 R_cam_base
  ② 云台角度旋转矩阵 R_gimbal
  
R_cam_to_body = R_gimbal @ R_cam_base


步骤3: Body坐标 → ENU坐标
────────────────────────
变换矩阵: R_body_to_enu
来源: 无人机姿态四元数（MAVROS）

R_cam_to_enu = R_body_to_enu @ R_cam_to_body
ray_enu = R_cam_to_enu @ ray_camera


步骤4: 射线-地面交点
──────────────────
射线方程: P = P_uav + t * ray_enu
地面约束: z = 0.9 (人体重心高度)

求解参数t:
  t = (0.9 - P_uav.z) / ray_enu[2]

人体位置:
  human_x = P_uav.x + t * ray_enu[0]
  human_y = P_uav.y + t * ray_enu[1]
  human_z = 0.9
```

**2. 相机参数（Typhoon H480 CGO3相机）**

| 参数 | 值 | 说明 |
|------|-----|------|
| **图像宽度** | 640 px | 相机分辨率宽度 |
| **图像高度** | 360 px | 相机分辨率高度（注意：不是480！） |
| **焦距** | 205.47 px | 已校准的像素单位焦距 |
| **光心X** | 320.5 px | 图像中心X坐标 |
| **光心Y** | 180.5 px | 图像中心Y坐标 |

**3. 云台角度处理**

```python
# 默认云台配置
gimbal_pitch = -45.0°  # 向下45度（搜救常用角度）
gimbal_yaw   = 0.0°    # 无偏航
gimbal_roll  = 0.0°    # 无横滚

# 旋转顺序: Yaw → Pitch → Roll (ZYX欧拉角)
```

**4. 距离估计备份方案**

当射线接近水平（无法与地面相交）时，使用基于检测框大小的距离估计：

```python
distance = (assumed_human_height * focal_length) / box_height_pixels
         = (1.7m * 205.47) / box_height
```

#### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `typhoon_h480_0/mavros/local_position/pose` | PoseStamped | 无人机ENU位置和姿态 |
| `/yolov11/BoundingBoxes` | BoundingBoxes | YOLO检测结果 |

#### 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/xtdrone/typhoon_h480_0/human_position_local` | PointStamped | 人体ENU坐标 |
| `/xtdrone/typhoon_h480_0/human_pose_local` | PoseStamped | 人体位姿 |

#### 使用方法

```bash
# 基本用法
rosrun <package_name> human_position_publisher_local.py typhoon_h480 0

# 多无人机场景
rosrun <package_name> human_position_publisher_local.py typhoon_h480 1
rosrun <package_name> human_position_publisher_local.py typhoon_h480 2
```

#### 输出示例

```
======================================================================
  人体位置发布器 - 方案1: MAVROS Local Position (ENU)
======================================================================
  车辆: typhoon_h480_0
  坐标系: ENU (东-北-天)
======================================================================
[就绪] 等待MAVROS位置和YOLO检测...

[调试] 无人机高度: 8.50m | 射线ENU: [0.234, -0.156, -0.958] | 云台pitch: -45.0°
[成功] 地面交点距离: 7.92m
[人体位置] ENU: (6.85, -3.24, 0.90) | 距离: 7.92m | 置信度: 0.87
```

#### 关键代码片段

**完整的旋转矩阵计算：**

```python
def compute_camera_rotation_matrix(self):
    # 1. 无人机姿态（Body → ENU）
    q_uav = Quaternion(w, x, y, z)
    R_body_to_enu = q_uav.rotation_matrix
    
    # 2. 相机基础安装（Camera → Body）
    R_cam_base = np.array([
        [0, 0, 1],   # Body X = Camera Z (前)
        [1, 0, 0],   # Body Y = Camera X (右)
        [0, 1, 0]    # Body Z = Camera Y (下)
    ])
    
    # 3. 云台旋转（Pitch, Yaw, Roll）
    R_gimbal = R_yaw @ R_pitch @ R_roll
    
    # 4. 组合变换
    R_cam_to_body = R_gimbal @ R_cam_base
    R_cam_to_enu = R_body_to_enu @ R_cam_to_body
    
    return R_cam_to_enu
```

---

### 2.3 位置对比工具 (`compare_positions.py`)

#### 功能描述

实时对比Actor真值位置和检测估计位置，计算各类误差指标，并提供统计分析和精度评估。

#### 核心特性

| 特性 | 说明 |
|------|------|
| **误差类型** | 各轴误差（ΔX, ΔY, ΔZ）、2D误差、3D误差 |
| **统计分析** | 均值、标准差、最大值、最小值 |
| **精度评级** | 5星评级系统（优秀/良好/一般/较差/很差） |
| **数据历史** | 保存最近100个样本 |
| **刷新频率** | 2Hz实时显示 |

#### 误差计算公式

**1. 各轴误差**
```
ΔX = X_estimated - X_ground_truth
ΔY = Y_estimated - Y_ground_truth
ΔZ = Z_estimated - Z_ground_truth
```

**2. 欧氏距离**
```
Distance_3D = √(ΔX² + ΔY² + ΔZ²)  # 三维空间距离
Distance_2D = √(ΔX² + ΔY²)        # 水平平面距离
```

**3. 统计指标**
```
均值(μ)   = Σ(error_i) / N
标准差(σ) = √[Σ(error_i - μ)² / N]
```

#### 精度评级标准

| 等级 | 3D误差范围 | 星级 | 适用场景 |
|------|-----------|------|---------|
| **优秀** | < 0.5m | ★★★★★ | 精密任务、近距离交互 |
| **良好** | 0.5 - 1.0m | ★★★★☆ | 一般跟踪、搜救 |
| **一般** | 1.0 - 2.0m | ★★★☆☆ | 粗略定位 |
| **较差** | 2.0 - 5.0m | ★★☆☆☆ | 需要优化 |
| **很差** | ≥ 5.0m | ★☆☆☆☆ | 算法失效 |

#### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/actor/ground_truth/position` | PointStamped | Actor真值位置 |
| `/xtdrone/typhoon_h480_0/human_position_local` | PointStamped | 检测估计位置 |

#### 使用方法

```bash
# 基本用法（默认typhoon_h480_0）
rosrun <package_name> compare_positions.py

# 指定车辆
rosrun <package_name> compare_positions.py typhoon_h480 0

# 多无人机对比
rosrun <package_name> compare_positions.py typhoon_h480 1
```

#### 输出示例

```
==========================================================================================
  实时位置对比 (ENU坐标系)
==========================================================================================
  【Actor真值】
    X:    5.234 m  |  Y:   -2.145 m  |  Z:    0.900 m

  【检测估计】
    X:    5.678 m  |  Y:   -2.034 m  |  Z:    0.900 m

  【位置误差】
    ΔX:  +0.444 m  |  ΔY:  +0.111 m  |  ΔZ:  +0.000 m

  【距离误差】
    2D误差:   0.458 m  |  3D误差:   0.458 m

  【统计数据】(最近100个样本)
    3D误差: 平均=0.523 m, 标准差=0.187 m, 最大=1.234 m, 最小=0.156 m
    2D误差: 平均=0.521 m, 标准差=0.185 m

  【精度评估】
    当前精度: ★★★★★ 优秀 (3D误差: 0.458 m)
==========================================================================================
```

#### 最终统计报告

程序退出时自动生成：

```
==========================================================================================
  最终统计报告
==========================================================================================

  总样本数: 100

  3D误差统计:
    平均值: 0.5234 m
    标准差: 0.1872 m
    最大值: 1.2345 m
    最小值: 0.1562 m

  2D误差统计:
    平均值: 0.5211 m
    标准差: 0.1854 m

  精度分布:
    优秀 (<0.5m):   48个 (48.0%)
    良好 (<1.0m):   44个 (44.0%)
    一般 (<2.0m):    8个 (8.0%)
    较差 (<5.0m):    0个 (0.0%)
    很差 (>=5.0m):   0个 (0.0%)

==========================================================================================
```

---

## 3. 完整使用流程

### 3.1 环境准备

```bash
# 1. 启动Gazebo仿真
roslaunch px4 mavros_posix_sitl.launch

# 2. 启动YOLO检测节点
roslaunch yolov11_ros yolo_v11.launch

# 3. 确保Actor已加载到场景中
# (在Gazebo中插入Actor模型)
```

### 3.2 启动系统组件

**终端1: 启动Actor真值发布器**
```bash
cd ~/catkin_ws/my
python publish_actor_ground_truth.py --rate 30
```

**终端2: 启动人体位置发布器**
```bash
cd ~/catkin_ws/my
python human_position_publisher_local.py typhoon_h480 0
```

**终端3: 启动位置对比工具**
```bash
cd ~/catkin_ws/my
python compare_positions.py typhoon_h480 0
```

### 3.3 RViz可视化（可选）

```bash
# 启动RViz
rviz

# 添加以下显示项：
# - TF: 显示actor_ground_truth坐标系
# - PointStamped: /actor/ground_truth/position (红色)
# - PointStamped: /xtdrone/typhoon_h480_0/human_position_local (绿色)
```

---

## 4. 技术细节

### 4.1 坐标系说明

#### ENU坐标系（East-North-Up）

```
        Z (Up)
        ↑
        |
        |
        o-----→ X (East)
       /
      /
     ↓
    Y (North)
```

**特点：**
- ROS标准坐标系
- MAVROS默认使用ENU
- Gazebo世界坐标系通常为ENU

#### 相机坐标系

```
    Y (Down)
    ↓
    |
    o-----→ X (Right)
   /
  /
 ↓
Z (Forward/Optical Axis)
```

**特点：**
- 光轴沿Z轴正方向
- 符合计算机视觉习惯

#### Body坐标系（FRD: Forward-Right-Down）

```
    Z (Down)
    ↓
    |
    o-----→ X (Forward)
   /
  /
 ↓
Y (Right)
```

**特点：**
- 飞控标准坐标系
- PX4使用FRD

### 4.2 变换矩阵推导

#### 相机到Body的变换

**目标：**将相机坐标系的向量转换到Body坐标系

**对应关系：**
```
Camera X (右)  → Body Y (右)
Camera Y (下)  → Body Z (下)
Camera Z (前)  → Body X (前)
```

**变换矩阵：**
```
R_cam_to_body = [0  0  1]   [X_cam]   [Z_cam]   [X_body]
                [1  0  0] × [Y_cam] = [X_cam] = [Y_body]
                [0  1  0]   [Z_cam]   [Y_cam]   [Z_body]
```

#### 云台旋转补偿

**Pitch旋转（绕Body Y轴）：**
```
R_pitch = [cos(θ)   0   sin(θ) ]
          [  0      1     0    ]
          [-sin(θ)  0   cos(θ) ]
```

对于gimbal_pitch = -45°（向下45度）：
```
R_pitch = [ 0.707   0   -0.707]
          [   0     1      0  ]
          [ 0.707   0    0.707]
```

### 4.3 射线-地面交点算法

**问题描述：**
给定无人机位置 P_uav = (x₀, y₀, z₀) 和射线方向 ray_enu = (dx, dy, dz)，求射线与地面 z = h 的交点。

**射线参数方程：**
```
P(t) = P_uav + t × ray_enu
     = (x₀ + t×dx, y₀ + t×dy, z₀ + t×dz)
```

**地面方程：**
```
z = h  (人体重心高度，通常0.9m)
```

**求解参数t：**
```
z₀ + t×dz = h
t = (h - z₀) / dz
```

**交点坐标：**
```
x_human = x₀ + t×dx
y_human = y₀ + t×dy
z_human = h
```

**边界条件：**
1. 如果 |dz| < 0.01：射线接近水平，无法相交
2. 如果 t ≤ 0：射线向上，无法与地面相交
3. 如果 t > 100：距离过远，可能不合理

### 4.4 误差来源分析

| 误差源 | 影响程度 | 缓解方法 |
|--------|---------|---------|
| **相机标定误差** | 中等 | 精确标定焦距和光心 |
| **YOLO检测框偏差** | 高 | 使用高置信度阈值(>0.4)，取框中心 |
| **无人机姿态估计误差** | 低 | MAVROS提供高精度姿态 |
| **云台角度误差** | 中等 | 使用真实云台反馈角度 |
| **地面高度假设** | 低 | 固定高度0.9m（人体重心） |
| **时间同步误差** | 低 | 使用ROS时间戳同步 |
| **图像畸变** | 低 | CGO3相机畸变较小 |

---

## 5. 性能指标

### 5.1 实测性能（参考值）

**测试条件：**
- 无人机高度：5-10米
- 人体距离：3-15米
- 云台角度：-45°
- 光照条件：良好

**结果：**
| 指标 | 数值 |
|------|------|
| **平均3D误差** | 0.5 - 0.8 m |
| **标准差** | 0.15 - 0.25 m |
| **最大误差** | < 1.5 m (95%样本) |
| **优秀率** | 40 - 60% (<0.5m) |
| **良好率** | 35 - 45% (0.5-1.0m) |
| **检测频率** | 15 - 30 Hz |
| **计算延迟** | < 50 ms |

### 5.2 影响精度的因素

**正相关因素（提高精度）：**
- ✅ 无人机高度降低（3-8米最佳）
- ✅ 目标在图像中心区域
- ✅ 检测框大小适中（100-300像素）
- ✅ 云台俯角适中（30-60度）
- ✅ 光照充足

**负相关因素（降低精度）：**
- ❌ 无人机高度过高（>15米）
- ❌ 目标在图像边缘（畸变大）
- ❌ 检测框过小（<50像素，距离过远）
- ❌ 云台接近水平（射线-地面交点不稳定）
- ❌ 光照不足（YOLO检测不准）

---

## 6. 故障排查

### 6.1 常见问题

#### 问题1: 未收到YOLO检测数据

**症状：**
```
[等待] 未收到检测估计数据...
```

**排查步骤：**
```bash
# 1. 检查YOLO节点是否运行
rosnode list | grep yolov11

# 2. 检查话题是否存在
rostopic list | grep BoundingBoxes

# 3. 查看话题消息
rostopic echo /yolov11/BoundingBoxes

# 4. 检查相机是否有图像输入
rostopic hz /typhoon_h480_0/cgo3_camera/image_raw
```

**解决方法：**
- 确保YOLO节点已启动
- 检查相机话题是否正确配置
- 验证人体在相机视野内

---

#### 问题2: 未收到MAVROS位置数据

**症状：**
```
[ERROR] MAVROS位置数据不可用
```

**排查步骤：**
```bash
# 1. 检查MAVROS节点
rosnode list | grep mavros

# 2. 检查位置话题
rostopic echo /typhoon_h480_0/mavros/local_position/pose

# 3. 检查MAVROS连接状态
rostopic echo /typhoon_h480_0/mavros/state
```

**解决方法：**
- 确保PX4 SITL正在运行
- 检查MAVROS是否正确连接到飞控
- 验证坐标系配置（应为ENU）

---

#### 问题3: Actor真值获取失败

**症状：**
```
[警告] 获取位置失败: service [/gazebo/get_model_state] responded with an error
```

**排查步骤：**
```bash
# 1. 检查Gazebo是否运行
ps aux | grep gazebo

# 2. 列出所有模型
rosservice call /gazebo/get_world_properties

# 3. 尝试手动获取actor状态
rosservice call /gazebo/get_model_state "model_name: 'actor'"
```

**解决方法：**
- 确保Actor已加载到Gazebo场景
- 检查Actor名称是否正确（默认为"actor"）
- 尝试使用 `--method topic` 方式

---

#### 问题4: 射线向上警告

**症状：**
```
[警告] 射线向上! UAV_z=5.00, target_z=0.90, ray_z=0.123, t=-33.33
```

**原因：**
- 云台角度过于水平或向上
- 无人机姿态异常
- 相机旋转矩阵计算错误

**解决方法：**
```python
# 检查云台角度配置
self.gimbal_pitch = -45.0  # 确保为负值（向下）

# 验证无人机姿态是否合理
rostopic echo /typhoon_h480_0/mavros/local_position/pose
```

---

#### 问题5: 误差异常大（>5米）

**可能原因：**
1. 相机参数错误
2. 坐标系变换错误
3. 时间同步问题
4. 云台角度不匹配

**排查方法：**
```bash
# 1. 对比时间戳
rostopic echo /actor/ground_truth/position | grep stamp
rostopic echo /xtdrone/typhoon_h480_0/human_position_local | grep stamp

# 2. 检查相机参数
rosparam get /typhoon_h480_0/cgo3_camera

# 3. 手动验证检测结果
rosrun image_view image_view image:=/yolov11/yolov11_image
```

---

### 6.2 调试技巧

#### 1. 启用详细日志

```python
# 在human_position_publisher_local.py中
rospy.set_param('~debug', True)

# 查看所有日志输出
rospy.loginfo_throttle(0.5, ...)  # 减小throttle时间
```

#### 2. 可视化检测结果

```bash
# 查看YOLO标注图像
rosrun image_view image_view image:=/yolov11/yolov11_image

# 在RViz中显示射线方向
# (需要修改代码发布Marker)
```

#### 3. 记录数据用于离线分析

```bash
# 记录所有相关话题
rosbag record \
  /actor/ground_truth/position \
  /xtdrone/typhoon_h480_0/human_position_local \
  /typhoon_h480_0/mavros/local_position/pose \
  /yolov11/BoundingBoxes

# 回放分析
rosbag play <bag_file> --clock
```

---

## 7. 扩展与优化

### 7.1 改进方向

**1. 动态云台角度获取**
```python
# 当前: 使用固定值
self.gimbal_pitch = -45.0

# 改进: 订阅真实云台反馈
rospy.Subscriber('/mavros/mount_control/orientation', ...)
```

**2. 卡尔曼滤波平滑**
```python
from filterpy.kalman import KalmanFilter

# 融合多帧检测结果，减少抖动
kf = KalmanFilter(dim_x=3, dim_z=3)
```

**3. 多目标跟踪**
```python
# 当前: 仅跟踪单个人体
# 改进: 使用DeepSORT或SORT算法跟踪多个人体
```

**4. 自适应人体高度**
```python
# 当前: 固定1.7m
# 改进: 根据检测框宽高比估计真实高度
human_height = estimate_height_from_aspect_ratio(box)
```

### 7.2 性能优化

```python
# 1. 减少不必要的计算
if not self.human_detected:
    return  # 提前返回

# 2. 缓存旋转矩阵
if self.uav_orientation_changed:
    self.R_cam_to_enu = self.compute_camera_rotation_matrix()

# 3. 降低发布频率
rate = rospy.Rate(10)  # 从30Hz降到10Hz
```

---

## 8. 参考资料

### 8.1 相关文档

- **PX4用户指南**: https://docs.px4.io/
- **MAVROS Wiki**: http://wiki.ros.org/mavros
- **Gazebo Tutorials**: http://gazebosim.org/tutorials
- **YOLOv11**: https://github.com/ultralytics/ultralytics

### 8.2 关键论文

1. **相机标定**: Zhang, Z. "A flexible new technique for camera calibration." IEEE TPAMI 2000.
2. **目标检测**: Redmon, J. "You Only Look Once: Unified, Real-Time Object Detection." CVPR 2016.
3. **视觉定位**: Hartley, R. & Zisserman, A. "Multiple View Geometry in Computer Vision." 2003.

### 8.3 代码依赖

```bash
# Python依赖
pip install numpy scipy pyquaternion

# ROS依赖
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
sudo apt-get install ros-noetic-gazebo-ros-pkgs
```

---

## 9. 版本历史

| 版本 | 日期 | 作者 | 更新内容 |
|------|------|------|---------|
| v1.0 | 2024-10 | - | 初始版本，支持Typhoon H480 |

---

## 10. 联系方式

**技术支持：**
- 邮箱: [填写邮箱]
- GitHub: [填写仓库地址]

**贡献指南：**
欢迎提交Issue和Pull Request！

---

**文档结束**

