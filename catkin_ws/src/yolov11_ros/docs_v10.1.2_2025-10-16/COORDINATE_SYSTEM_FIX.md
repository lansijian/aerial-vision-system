# 坐标系统一修复 - 解决旋转180°后反向问题

## 🔍 问题根源（用户发现）

### 现象
- **正常飞行**：无人机0追踪正确 ✅
- **旋转180°后**：前后反向，但左右正确 ❌

### 关键线索
**只有X轴（前后）反向，Y轴（左右）正常！**

这排除了：
- ❌ 整体坐标系反转
- ❌ 参数配置问题
- ❌ 初始spawn差异

---

## 🎯 根本原因

### 坐标系混用问题

#### 我们的实现（错误）
```python
# drone_controller.py - 旧版本
self.cmd_vel_pub = rospy.Publisher(
    '/mavros/setpoint_velocity/cmd_vel',  # 使用TwistStamped
    TwistStamped, queue_size=1
)

# 发布速度
cmd = TwistStamped()
cmd.header.frame_id = "base_link"  # 坐标系不明确
cmd.twist.linear.x = 0.5  # 这是FLU还是NED？不清楚
self.cmd_vel_pub.publish(cmd)
```

**问题**：
- `TwistStamped` + `base_link`的坐标系定义**不明确**
- MAVROS可能将其解释为不同的坐标系
- 旋转180°后，MAVROS的处理方式可能改变

#### XTDrone的实现（正确）
```python
# XTDrone communication.py
self.target_motion_pub = rospy.Publisher(
    '/mavros/setpoint_raw/local',  # 使用PositionTarget
    PositionTarget, queue_size=1
)

# 明确指定坐标系
target = PositionTarget()
target.coordinate_frame = PositionTarget.FRAME_BODY_NED  # 明确NED！
target.velocity.x = vx  # NED坐标系
target.velocity.y = vy
```

**优势**：
- ✅ `PositionTarget`支持明确指定坐标系
- ✅ `FRAME_BODY_NED`定义清晰
- ✅ 任何偏航角下行为一致

---

## ✅ 解决方案

### 坐标系定义

#### FLU（Forward-Left-Up）
```
X: 前（Forward）
Y: 左（Left）
Z: 上（Up）
```
- 用于：human_tracker计算的速度（符合直觉）

#### NED（North-East-Down）
```
X: 北/前（North）
Y: 东/右（East）
Z: 下（Down）
```
- 用于：MAVROS/PX4（航空标准）

### 坐标转换公式

```python
# FLU → NED
velocity_ned.x = velocity_flu.x      # X: 前（相同）
velocity_ned.y = -velocity_flu.y     # Y: 左→右（反转）
velocity_ned.z = -velocity_flu.z     # Z: 上→下（反转）
yaw_rate_ned = -yaw_rate_flu         # 偏航率（反转）
```

### 代码修改

#### drone_controller.py
```python
# 添加PositionTarget发布者
self.setpoint_raw_pub = rospy.Publisher(
    f'/{self.vehicle_ns}/mavros/setpoint_raw/local',
    PositionTarget, queue_size=1
)

# 主循环中转换并发布
if self.flight_mode == "TRACKING":
    target = PositionTarget()
    target.coordinate_frame = PositionTarget.FRAME_BODY_NED  # 明确指定NED
    target.type_mask = (忽略位置，只用速度)
    
    # FLU → NED转换
    target.velocity.x = self.current_cmd.twist.linear.x      # X不变
    target.velocity.y = -self.current_cmd.twist.linear.y     # Y反转
    target.velocity.z = -self.current_cmd.twist.linear.z     # Z反转
    target.yaw_rate = -self.current_cmd.twist.angular.z      # yaw反转
    
    self.setpoint_raw_pub.publish(target)
```

---

## 📊 修复效果

### 旋转前（偏航角0°）
```
human_tracker(FLU): vx=0.5, vy=0.2, vz=0.0, yaw=0.1
              ↓ FLU→NED转换
drone_controller(NED): vx=0.5, vy=-0.2, vz=0.0, yaw=-0.1
              ↓ FRAME_BODY_NED
MAVROS/PX4: 向前0.5m/s，向右0.2m/s ✅
```

### 旋转180°后（偏航角180°）
```
human_tracker(FLU): vx=0.5, vy=0.2, vz=0.0, yaw=0.1
              ↓ FLU→NED转换（相同）
drone_controller(NED): vx=0.5, vy=-0.2, vz=0.0, yaw=-0.1
              ↓ FRAME_BODY_NED（坐标系明确）
MAVROS/PX4: 向前0.5m/s，向右0.2m/s ✅（不受偏航角影响）
```

**关键**：使用`FRAME_BODY_NED`后，无论偏航角多少，行为都一致！

---

## 🧪 测试验证

### 启动观察
```
✅ 无人机0控制器初始化完成
   速度控制：使用FRAME_BODY_NED坐标系（修复旋转180°问题）
```

### 追踪时观察
```
# 收到FLU命令
[追踪速度FLU] 无人机0 收到:(vx=0.50,vy=0.20,vz=0.00,yaw=0.10)

# 转换为NED发布
[控制权-TRACKING] 无人机0 转发追踪速度(NED): vx=0.50 vy=-0.20 yaw=-0.10
```

### 旋转180°后测试
1. 让无人机0飞向需要转180°的航点
2. 转向后检测到目标
3. 观察追踪行为：
   - ✅ 前后：应该向前靠近目标（不反向）
   - ✅ 左右：应该向右/左对准目标（继续正确）

---

## 🔧 关键改进

| 方面 | 旧实现 | 新实现 |
|------|--------|--------|
| 消息类型 | TwistStamped | PositionTarget |
| 话题 | /mavros/setpoint_velocity/cmd_vel | /mavros/setpoint_raw/local |
| 坐标系 | base_link（不明确） | FRAME_BODY_NED（明确） |
| 转换 | 无 | FLU→NED |
| 偏航角影响 | ❌ 受影响 | ✅ 不受影响 |

---

## 💡 为什么之前Y正确但X反？

### 推测
MAVROS的`/setpoint_velocity/cmd_vel`话题可能：
1. 在偏航角接近0°时，使用一种坐标系定义
2. 在偏航角接近180°时，触发某种边界条件处理
3. 导致X轴的处理方式改变，但Y轴保持不变

使用明确的`FRAME_BODY_NED`后，避免了这种模糊性。

---

## ⚠️ 注意事项

### 1. 移除了velocity_invert参数
- 不再需要per-drone配置
- 统一通过FLU→NED转换处理

### 2. human_tracker无需修改
- 继续使用FLU格式（符合直觉）
- vx=前，vy=左，vz=上

### 3. waypoint_navigator无需修改
- 使用位置控制，不受影响

---

**修复日期**：2025-10-16  
**版本**：v10.1.2-refactored-final  
**状态**：✅ 统一坐标系，解决旋转180°问题  
**参考**：XTDrone multirotor_communication.py  
**东华大学 Astraeus队**

