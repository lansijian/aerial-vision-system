# compare_positions_integrated.py 技术文档

## 文件概述

**功能**: 集成式位置对比工具，自动获取Actor真值并与检测估计位置实时对比  
**语言**: Python 2.7/3.x  
**依赖**: ROS, rospy, numpy, gazebo_msgs, tf2_ros

---

## 主要功能

### 核心能力
- 自动获取Gazebo Actor真值位置（无需手动启动真值发布器）
- 实时对比真值与估计位置
- 计算3D和2D位置误差
- 统计误差分布和精度评估
- 低误差持续时间跟踪
- 数据时延监控

### 使用场景
- 位置估计算法验证
- 跟踪系统性能评估
- 实时精度监控
- 算法调试与优化

---

## 架构设计

### 双类架构

```
compare_positions_integrated.py
├── ActorGroundTruthPublisher (后台运行)
│   ├── 从Gazebo获取Actor位置
│   ├── 发布真值话题
│   └── 发布TF变换
│
└── PositionComparator (主控制器)
    ├── 订阅真值话题
    ├── 订阅估计话题
    ├── 计算误差
    ├── 统计分析
    └── 终端显示
```

---

## 核心功能详解

### 1. ActorGroundTruthPublisher（真值发布器）

#### 数据获取方式
支持两种方式获取Actor位置：
- **服务方式**（默认）：通过Gazebo服务获取
  - `/gazebo/get_model_state`
  - `/gazebo/get_link_state`（优先）
- **话题方式**：订阅`/gazebo/model_states`

#### 发布内容
| 话题/变换 | 类型 | 说明 |
|----------|------|------|
| `/actor/ground_truth/position` | PointStamped | Actor位置（点） |
| `/actor/ground_truth/pose` | PoseStamped | Actor位姿 |
| `actor_ground_truth` TF | TransformStamped | TF变换 |

**特点**: 后台静默运行，无终端输出

---

### 2. PositionComparator（位置对比器）

#### 误差计算
```python
# 各轴误差
dx = estimated_x - ground_truth_x
dy = estimated_y - ground_truth_y
dz = estimated_z - ground_truth_z

# 欧氏距离
distance_2d = √(dx² + dy²)
distance_3d = √(dx² + dy² + dz²)
```

#### 统计指标
- **实时误差**: dx, dy, dz, 2D误差, 3D误差
- **统计数据**（最近100个样本）:
  - 平均误差
  - 标准差
  - 最大/最小误差
  - 精度分布

#### 精度评估标准
| 3D误差范围 | 评级 | 星级 |
|-----------|------|------|
| < 0.5米 | 优秀 | ★★★★★ |
| 0.5-1.0米 | 良好 | ★★★★☆ |
| 1.0-2.0米 | 一般 | ★★★☆☆ |
| 2.0-5.0米 | 较差 | ★★☆☆☆ |
| ≥ 5.0米 | 很差 | ★☆☆☆☆ |

---

## 使用方法

### 命令格式
```bash
python compare_positions_integrated.py [vehicle_type] [vehicle_id] [actor_name]
```

### 使用示例

#### 1. 使用默认参数
```bash
python compare_positions_integrated.py
```
默认: `typhoon_h480 0 actor`

#### 2. 指定飞行器
```bash
python compare_positions_integrated.py typhoon_h480 0
```

#### 3. 指定Actor名称
```bash
python compare_positions_integrated.py typhoon_h480 0 my_actor
```

---

## 订阅话题

| 话题 | 类型 | 说明 |
|-----|------|-----|
| `/actor/ground_truth/position` | PointStamped | Actor真值位置（自动发布） |
| `/xtdrone/<vehicle>/human_position_local` | PointStamped | 检测估计位置 |

---

## 输出格式

### 终端输出示例

```
==================================================================================
  实时位置对比 (ENU坐标系)
==================================================================================
  【数据状态】
    估计数据时延: 0.015 秒 ✓

  【Actor真值位置】
    X:    5.234 米  |  Y:   -2.145 米  |  Z:    0.900 米

  【检测估计位置】
    X:    5.456 米  |  Y:   -2.367 米  |  Z:    0.900 米

  【位置误差】
    ΔX:  +0.222 米  |  ΔY:  -0.222 米  |  ΔZ:  +0.000 米

  【距离误差】
    2D误差:   0.314 米  |  3D误差:   0.314 米

  【统计数据】(最近100个样本)
    3D误差: 平均=0.358 米, 标准差=0.125 米, 最大=0.652 米, 最小=0.142 米
    2D误差: 平均=0.358 米, 标准差=0.125 米

  【精度评估】
    当前精度: ★★★★★ 优秀 (3D误差: 0.314 米)

  【低误差跟踪】(|ΔX|<1.0米 且 |ΔY|<1.0米)
    状态: ✓ 持续低误差
    持续时间: 45.3秒
    当前误差: ΔX=0.222米, ΔY=-0.222米
==================================================================================
```

---

## 核心类说明

### ActorGroundTruthPublisher

**初始化参数**:
- `actor_name`: Actor模型名称（默认'actor'）
- `publish_rate`: 发布频率（默认30Hz）
- `use_service`: 使用服务方式（默认True）

**关键方法**:
- `setup_service()`: 初始化Gazebo服务
- `setup_subscriber()`: 初始化话题订阅
- `get_actor_pose_from_service()`: 从服务获取位姿
- `publish_actor_pose()`: 发布真值位置

---

### PositionComparator

**初始化参数**:
- `vehicle_type`: 飞行器类型（如'typhoon_h480'）
- `vehicle_id`: 飞行器ID（如'0'）
- `actor_name`: Actor名称（默认'actor'）

**关键方法**:
- `ground_truth_callback(msg)`: 接收真值位置
- `estimated_callback(msg)`: 接收估计位置
- `calculate_error()`: 计算位置误差
- `calculate_statistics()`: 计算统计数据
- `print_comparison()`: 打印对比结果
- `update_truth_publisher()`: 更新真值发布器

---

## 特色功能

### 1. 低误差持续跟踪
- **阈值**: |ΔX| < 1.0米 且 |ΔY| < 1.0米
- **功能**: 自动记录持续低误差的时间
- **显示**: 实时显示持续时间（秒/分钟/小时）
- **用途**: 评估系统稳定性

### 2. 数据时延监控
- 实时监控估计数据的时间戳
- 显示数据新鲜度（时延）
- 标识数据状态：
  - ✓ 良好（<0.2秒）
  - ⚠ 警告（0.2-0.5秒）
  - ✗ 过期（>0.5秒）
- 超时检测（>1.0秒视为数据过期）

### 3. 最终统计报告
退出时自动生成：
- 总样本数
- 完整误差统计（均值、标准差、最大/最小值）
- 精度等级分布（百分比）
- 低误差持续时间总结

---

## 配置参数

### 可调参数
| 参数 | 默认值 | 说明 |
|-----|--------|------|
| data_timeout | 1.0秒 | 数据过期阈值 |
| error_history.maxlen | 100 | 误差历史记录数 |
| low_error_threshold | 1.0米 | 低误差阈值 |
| display_interval | 15次 | 显示刷新间隔（约0.5秒） |
| publish_rate | 30Hz | 真值发布频率 |

---

## 运行流程

```
1. 启动
   ├─ 初始化ROS节点
   ├─ 创建ActorGroundTruthPublisher（后台）
   └─ 创建PositionComparator

2. 运行循环 (30Hz)
   ├─ 更新真值发布器
   ├─ 接收真值和估计位置
   ├─ 计算误差
   ├─ 更新统计数据
   ├─ 检查低误差状态
   └─ 定期显示结果（0.5秒一次）

3. 退出
   ├─ 保存低误差持续时间
   ├─ 打印最终统计报告
   └─ 显示精度分布
```

---

## 注意事项

1. **运行前提**
   - Gazebo仿真必须正在运行
   - Actor模型必须存在于仿真中
   - 位置估计器必须正在发布数据

2. **性能建议**
   - 运行频率30Hz，平衡性能与CPU占用
   - 显示频率约2Hz，避免终端刷新过快
   - 历史记录100个样本，适合短期统计

3. **常见问题**
   - 如果显示"Gazebo服务不可用"，检查仿真是否启动
   - 如果长时间"等待数据"，检查话题是否正确
   - 数据过期警告表示位置发布器可能停止

---

## 输出文件

本工具仅在终端显示，不生成日志文件。如需记录数据，建议：
```bash
python compare_positions_integrated.py > output.log 2>&1
```

---

## 扩展建议

### 可能的改进方向
1. 添加CSV数据导出功能
2. 实时绘制误差曲线
3. 支持多目标对比
4. 添加可视化界面（如rviz marker）
5. 支持自定义误差阈值
6. 添加误差报警功能

---

## 作者与维护

**更新日期**: 2025  
**维护状态**: 活跃维护  
**适用场景**: 位置估计算法验证与性能评估

