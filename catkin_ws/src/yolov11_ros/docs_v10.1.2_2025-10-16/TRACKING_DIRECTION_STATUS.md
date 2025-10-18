# 追踪控制方向状态说明 - 2025-10-16

## 当前实现（v10.1.2稳定做法）

### 架构
```
human_tracker（计算速度）
    ↓ Twist（机体坐标系）
drone_controller（直接转发）
    ↓ TwistStamped + frame_id="base_link"
MAVROS (/setpoint_velocity/cmd_vel)
```

### 速度定义（机体坐标系）
```python
# human_tracker发送
cmd_vel.linear.x  # 机体X轴（前）
cmd_vel.linear.y  # 机体Y轴（右？左？）
cmd_vel.linear.z  # 机体Z轴（下）
cmd_vel.angular.z # 偏航率
```

---

## ⚠️ 潜在问题：无人机0旋转180°后

### 问题现象
- **正常飞行**（偏航角0°）：追踪正确 ✅
- **旋转180°后**（偏航角180°）：前后可能反向 ⚠️

### 可能原因
1. MAVROS对`/setpoint_velocity/cmd_vel`的解释可能依赖偏航角
2. 不同偏航角下，`base_link`的定义可能改变

---

## 🧪 测试验证

### 关键测试场景

1. **无人机0正常飞行时追踪**
```
[追踪优化] 框高:120 速度:(vx=0.05,vy=0.10,yaw=-0.12)
[动作] ↑靠近 →右移 ↻转向
# 验证：无人机应该向前靠近目标 ✅
```

2. **无人机0旋转180°后追踪**（关键！）
```
[追踪优化] 框高:120 速度:(vx=0.05,vy=0.10,yaw=-0.12)
[动作] ↑靠近 →右移 ↻转向
# 验证：无人机应该向前靠近目标
# 如果向后远离，则说明vx方向反了 ❌
```

### 观察要点

```
# 目标在前方，框太小（距离远）
框高:50 → height_error>0 → vx>0 → 应该向前靠近

# 目标在前方，框太大（距离近）  
框高:200 → height_error<0 → vx<0 → 应该向后远离

# 如果行为相反，说明vx符号错了
```

---

## 🔧 如果方向仍然有问题

### 解决方案A：添加方向修正参数（临时）

回到之前的velocity_invert方案：

```xml
<!-- launch文件 -->
<param name="velocity_invert_x" value="true" />  <!-- 只针对无人机0 -->
```

```python
# drone_controller.py
cmd.linear.x = -msg.linear.x if self.velocity_invert_x else msg.linear.x
```

### 解决方案B：检查human_tracker速度符号

检查速度计算逻辑：
```python
# 前后速度
height_error = ideal_box_height - box_height
forward_speed = Kp_distance * (height_error / ideal_box_height)

# 检查：
# box小（远）→ height_error>0 → vx>0 → 应该向前 ✅
# box大（近）→ height_error<0 → vx<0 → 应该向后 ✅
```

### 解决方案C：使用PositionTarget（根本解决）

如果需要彻底解决，使用明确的坐标系：
```python
target = PositionTarget()
target.coordinate_frame = PositionTarget.FRAME_BODY_NED
# 但需要正确的FLU→NED转换
```

**问题**：需要确认human_tracker发送的是FLU还是body frame

---

## 📝 速度计算检查

### human_tracker当前实现

```python
# 横向速度
lateral_speed = -Kp_lateral * u_offset
# u_offset = u - cx
# u > cx（目标在右）→ u_offset > 0 → lateral_speed < 0 → 向左？
# 检查：如果目标在右边，应该向右飞（vy > 0）
# 所以符号可能对或错，需要测试验证

# 偏航率
yaw_rate = -Kp_yaw * u_offset  
# u > cx（目标在右）→ u_offset > 0 → yaw_rate < 0 → 向左转
# 检查：如果目标在右边，应该向右转（yaw_rate > 0）
# 符号可能错了？
```

### 建议检查

添加调试日志确认符号：
```python
if u_offset > 50:  # 目标明显在右边
    # lateral_speed应该>0（向右）
    # yaw_rate应该>0（向右转）
    rospy.loginfo(f"目标在右 u_offset={u_offset} → vy={vy} yaw={yaw}")
```

---

## 🎯 当前状态

### 已完成
- ✅ 高度保持：vz=0（PX4自动保持）
- ✅ 追踪优化：Kalman + 自适应Kp + PI控制
- ✅ 参数ROS化：12个参数可调
- ✅ 100Hz控制频率

### 待验证
- ⚠️ 无人机0旋转180°后方向是否正确
- ⚠️ 横向和偏航符号是否正确

---

## 💡 测试建议

### 1. 分步测试

**Step 1**：无人机1（正常的）
- 测试追踪是否正确
- 确认速度符号逻辑

**Step 2**：无人机0（正常飞行）
- 测试追踪是否正确
- 对比无人机1

**Step 3**：无人机0（旋转180°后）
- 这是关键测试
- 如果反向，需要修正

### 2. 观察日志

```
# 目标在右边
[追踪优化] 偏移:(u=100,v=0) ...
[动作] →右移 ↻转向
# 验证：无人机应该向右移动和转向

# 目标太远
[追踪优化] 框高:50 ...
[动作] ↑靠近
# 验证：无人机应该向前飞

# 目标太近
[追踪优化] 框高:200 ...
[动作] ↓远离
# 验证：无人机应该向后退
```

---

**状态**：参考v10.1.2稳定做法，待测试验证  
**日期**：2025-10-16  
**东华大学 Astraeus队**

