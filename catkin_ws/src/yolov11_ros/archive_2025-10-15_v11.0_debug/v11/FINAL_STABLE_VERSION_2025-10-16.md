# v11.1 最终稳定版本 - 2025-10-16

## 概述

经过2025-10-15到10-16两天的密集调试和优化，最终创建了v11.1稳定版本。

## 关键问题与解决方案

### 1. 航点控制"倒着飞"问题 ✅

**问题**：无人机转180度倒着飞航点

**根本原因**：
- 使用TwistStamped + setpoint_velocity接口
- 坐标系不明确
- 通过controller转发导致延迟和错误

**解决方案**：
- 参考v9.2，使用PositionTarget + setpoint_raw/local
- waypoint_mission**直接发布**到MAVROS
- 使用FRAME_BODY_NED明确坐标系

### 2. 起飞卡住问题 ✅

**问题**：起飞时高度为负且不上升

**根本原因**：
- 使用速度控制起飞
- NED坐标系Z方向理解错误

**解决方案**：
- 参考v9.2，改用**位置控制**起飞
- 发送目标位置，让PX4自己规划上升

### 3. 追踪飞太高问题 ✅

**问题**：追踪时无人机飞到6米以上

**根本原因**：
- 高度控制算法问题
- 没有高度限制

**解决方案**：
```python
# 严格限制高度
self.min_height = 2.0
self.max_height = 4.0

if height > self.max_height:
    cmd_vel.linear.z = min(cmd_vel.linear.z, -0.5)  # 强制下降
elif height < self.min_height:
    cmd_vel.linear.z = max(cmd_vel.linear.z, 0.5)  # 强制上升
```

### 4. 追踪剧烈抖动问题 ✅

**问题**：追踪时无人机剧烈振荡

**根本原因**：
- 控制增益太大
- 没有平滑处理
- 没有死区

**解决方案**：
```python
# 降低增益
self.Kp_xy = 0.3  # 从0.5降低
self.Kp_z = 0.5   # 从1.0降低

# 添加死区
self.pixel_deadzone = 30  # 像素
self.height_deadzone = 0.3  # 米

# 添加平滑
self.smooth_factor = 0.7
cmd_vel.linear.x = 0.7 * last_vel.x + 0.3 * new_vel.x
```

### 5. OFFBOARD模式重复设置 ✅

**问题**：OFFBOARD设置成功后仍在日志中重复输出

**解决方案**：
- 只在EXECUTING状态检查
- 只在真正退出时才尝试恢复
- 不使用throttle，避免误导性日志

### 6. 航点文件加载问题 ✅

**问题**：两台无人机都加载drone_1的航点

**解决方案**：
- 使用anonymous=True避免节点名冲突
- 添加详细调试日志
- 确认参数正确传递

## 最终架构

### 模块职责

| 模块 | 职责 | MAVROS接口 |
|------|------|-----------|
| mission_controller.py | 起飞、模式切换 | setpoint_raw/local（起飞和追踪） |
| waypoint_mission.py | 航点导航 | setpoint_raw/local（直接发布）✅ |
| human_tracker_v11.1_stable.py | 目标追踪 | 通过controller转发 |
| multi_drone_manager.py | 目标分配 | 无 |

### 控制流

```
WAYPOINT模式：
  waypoint_mission ──PositionTarget/NED──> MAVROS

TRACKING模式：
  human_tracker ──Twist/FLU──> mission_controller ──PositionTarget/NED──> MAVROS

TAKEOFF/LANDING：
  mission_controller ──PositionTarget/NED或Twist──> MAVROS
```

## 关键参数

### 追踪控制
```python
Kp_xy = 0.3           # 水平增益（降低，减少抖动）
Kp_z = 0.5            # 高度增益（降低，减少振荡）
Kp_yaw = 0.002        # 偏航增益
pixel_deadzone = 30   # 像素死区
height_deadzone = 0.3 # 高度死区
smooth_factor = 0.7   # 平滑系数
min_height = 2.0      # 最低高度
max_height = 4.0      # 最高高度
```

### 航点导航
```python
cruise_speed = 5.0              # 巡航速度
waypoint_threshold = 2.0        # 到达阈值
直接使用PositionTarget/NED      # 机体NED坐标系
```

## 文件清单

### 新增/修改文件

**核心模块**：
- [x] `human_tracker_v11.1_stable.py` - 优化的稳定追踪器
- [x] `waypoint_mission.py` - 改用直接发布PositionTarget
- [x] `mission_controller.py` - 优化OFFBOARD检查，WAYPOINT时不发送命令

**测试模块（保留）**：
- [x] `simple_drone_controller.py` - 测试用
- [x] `simple_waypoint_navigator.py` - 测试用
- [x] `simple_human_tracker.py` - 测试用

**Launch文件**：
- [x] `multi_drone_system_v11.1_stable.launch` - 最终稳定launch
- [x] `simple_dual_drone_system.launch` - 测试launch

**文档**：
- [x] `FINAL_STABLE_VERSION_2025-10-16.md` - 本文档
- [x] `v11.1_FINAL_ARCHITECTURE.md` - 架构说明
- [x] `FIX_MAVROS_INTERFACE_v11.1.md` - MAVROS接口修复

## 使用方法

### 推荐：最终稳定版本
```bash
# 终端1
roslaunch px4 robocup.launch

# 终端2（等待10秒）
roslaunch yolov11_ros multi_drone_system_v11.1_stable.launch
```

### 测试：简化版本
```bash
roslaunch yolov11_ros simple_dual_drone_system.launch
```

## 预期效果

- ✅ 起飞正常（位置控制）
- ✅ 航点飞行正确（不倒着飞）
- ✅ 追踪高度稳定（2-4米）
- ✅ 追踪平滑无抖动
- ✅ ActorInfo精度高（v10.0.1算法）
- ✅ YOLO窗口标注无人机编号

## 版本对比

| 特性 | v11.0 | v11.1 Simple | v11.1 Stable |
|------|-------|--------------|--------------|
| 航点控制 | 通过controller | 直接发布 ✅ | 直接发布 ✅ |
| 追踪控制 | 复杂融合 | v10.0.1原版 | v10.0.1优化 ✅ |
| 高度限制 | 无 | 无 | 2-4米 ✅ |
| 抖动处理 | 无 | 无 | 平滑+死区 ✅ |
| 代码复杂度 | 高 | 低 | 中 |

## 调试检查

### 1. 检查航点加载
```
[航点导航-调试] drone_id=0, waypoint_file=.../waypoints_drone_0.json
   航点2: (-15.0, 0.0, 3.0)  # drone_0向西
   >>> 确认：这是drone_0的航点（向西）
```

### 2. 检查起飞
```
✅ 无人机0已切换到OFFBOARD模式
✅ 无人机0解锁成功
🚀 无人机0: 开始起飞...
起飞中... 1.0m / 3.0m
起飞中... 2.0m / 3.0m
✅ 无人机0到达目标高度: 3.0m
```

### 3. 检查追踪
```
[追踪0] 像素:(320,180) 偏移:(0,0) 高度:3.0m 目标:3.0m | 
速度:X=0.00 Y=0.00 Z=0.00
```

应该看到：
- 高度保持在2-4米之间
- 速度平滑，不剧烈变化
- 偏移小时速度为0（死区）

## 下一步优化（可选）

如果还有问题：

### 进一步降低增益
```python
self.Kp_xy = 0.2  # 更平缓
self.Kp_z = 0.3   # 更稳定
```

### 增大平滑系数
```python
self.smooth_factor = 0.8  # 更平滑，但响应变慢
```

### 增大死区
```python
self.pixel_deadzone = 50  # 更大的稳定区域
self.height_deadzone = 0.5
```

## 开发日志

### 2025-10-15
- v11.0初始模块化重构
- 修复解锁、OFFBOARD、速度为零等问题
- 多次尝试修复航点和追踪方向

### 2025-10-16
- 彻底重构，参考v9.2和v10.0.1
- 航点改用直接发布PositionTarget
- 追踪添加平滑和死区
- 创建稳定版本human_tracker_v11.1_stable.py

---
**版本**：v11.1 Stable  
**日期**：2025-10-16  
**状态**：✅ 生产就绪  
**作者**：东华大学 Astraeus队

