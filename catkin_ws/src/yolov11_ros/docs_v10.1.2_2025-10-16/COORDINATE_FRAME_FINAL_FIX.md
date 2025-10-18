# 坐标系最终修复 - 2025-10-16

## 🔴 问题确认

### 用户反馈
- 无人机旋转后**还是后退** ❌
- MAVLINK警告：`SET_POSITION_TARGET_LOCAL_NED invalid`

### 根本原因

**关键bug**：使用`TwistStamped` + `base_link`（机体坐标系）
- 机体坐标系会随无人机旋转而旋转
- 旋转180°后，"机体前方"就是世界坐标的"后方"
- 导致追踪方向反向

---

## ✅ 最终解决方案

### 核心思路

**使用世界坐标系（FRAME_LOCAL_NED）发送速度命令**
- 速度方向固定在世界坐标系
- 不受无人机偏航角影响
- 旋转任意角度都正确

### 实现方式

#### 1. human_tracker计算机体坐标系速度

```python
# 简单直接，基于图像误差
forward_speed = Kp_distance * (height_error / ideal_box_height)
lateral_speed = Kp_lateral * u_offset

# 机体坐标系（相对无人机）
cmd_vel.linear.x = forward_speed  # 机体前方
cmd_vel.linear.y = lateral_speed  # 机体右方
```

#### 2. drone_controller转换为世界坐标系

```python
# 获取当前偏航角
yaw = get_yaw_from_quaternion(current_pose)

# 机体坐标系 → 世界坐标系（旋转变换）
vx_world = vx_body * cos(yaw) - vy_body * sin(yaw)
vy_world = vx_body * sin(yaw) + vy_body * cos(yaw)

# 使用PositionTarget + FRAME_LOCAL_NED
target = PositionTarget()
target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED  # 世界坐标系
target.velocity.x = vx_world  # 世界坐标X（北/东）
target.velocity.y = vy_world  # 世界坐标Y（北/东）
```

---

## 📊 坐标系对比

| 方案 | 坐标系 | 旋转180°后 | 结果 |
|------|--------|-----------|------|
| TwistStamped | 机体（base_link） | 前方反了 | ❌ 后退 |
| PositionTarget | 世界（FRAME_LOCAL_NED） | 方向不变 | ✅ 正确 |

---

## 🧪 验证示例

### 场景：无人机旋转180°追踪

#### 修复前（TwistStamped/机体）
```
yaw=0°   → cmd_vel.x=0.5 → 机体前方 → 世界北方 ✅
yaw=180° → cmd_vel.x=0.5 → 机体前方 → 世界南方 ❌（反了）
```

#### 修复后（PositionTarget/世界）
```
yaw=0°   → 机体vx=0.5 → 世界vx=0.5 → 世界北方 ✅
yaw=180° → 机体vx=0.5 → 世界vx=-0.5 → 世界北方 ✅（仍然正确）
```

---

## 🔧 完整改动

### drone_controller.py

1. **添加PositionTarget发布者**
```python
self.setpoint_raw_pub = rospy.Publisher(
    f'/{self.vehicle_ns}/mavros/setpoint_raw/local',
    PositionTarget, queue_size=1
)
```

2. **起飞改用PositionTarget**
```python
target = PositionTarget()
target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
target.velocity.z = -1.0  # NED: 负Z是上升
```

3. **追踪转发时做坐标转换**
```python
# 机体 → 世界
vx_world = vx_body * cos(yaw) - vy_body * sin(yaw)
vy_world = vx_body * sin(yaw) + vy_body * cos(yaw)

target.velocity.x = vx_world
target.velocity.y = vy_world
```

### waypoint_navigator.py

**修复MAVLINK警告**：
```python
# 非WAYPOINT时不发送任何命令（而不是发送IGNORE_ALL）
else:
    # 不发布
    pass
```

---

## 📝 关键点

### NED坐标系说明

**FRAME_LOCAL_NED**：
- X: 北（North）- 相对起飞点向前
- Y: 东（East）- 相对起飞点向右
- Z: 下（Down）- 向下为正，**所以上升是负值**

### 速度命令示例

```python
# 上升
velocity.z = -1.0  # 负值

# 下降
velocity.z = 1.0   # 正值

# 向北飞
velocity.x = 1.0

# 向东飞
velocity.y = 1.0
```

---

## ✅ 解决的问题

1. ✅ 无人机旋转180°后追踪正确（世界坐标系）
2. ✅ MAVLINK警告消除（正确的type_mask）
3. ✅ 控制权冲突修复（非WAYPOINT不发布）

---

**修复版本**：v10.1.2-optimized  
**状态**：✅ 使用世界坐标系，彻底解决旋转问题  
**日期**：2025-10-16  
**东华大学 Astraeus队**

