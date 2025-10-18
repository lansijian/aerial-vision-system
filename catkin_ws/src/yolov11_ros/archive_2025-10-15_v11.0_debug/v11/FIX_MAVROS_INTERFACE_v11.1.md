# MAVROS接口问题修复 - v11.1

**日期**：2025-01-15  
**版本**：v11.1  
**问题**：无人机倒着飞，使用了错误的MAVROS接口

## 问题诊断

### 根本原因
**使用了错误的MAVROS话题和消息类型**

#### 错误的实现（v11.0）
```python
# 使用 TwistStamped 和 setpoint_velocity/cmd_vel
self.cmd_vel_pub = rospy.Publisher(
    f'/{vehicle_ns}/mavros/setpoint_velocity/cmd_vel',
    TwistStamped, queue_size=1
)

# 直接发送
cmd = TwistStamped()
cmd.twist.linear.x = 5.0  # FLU坐标系
self.cmd_vel_pub.publish(cmd)
```

**问题**：
- 坐标系定义不明确
- 可能与PX4的坐标系不匹配
- 导致控制方向混乱

#### 正确的实现（v9.2）
```python
# 使用 PositionTarget 和 setpoint_raw/local
self.setpoint_pub = rospy.Publisher(
    f'/{vehicle_ns}/mavros/setpoint_raw/local',
    PositionTarget, queue_size=1
)

# 明确指定坐标系
target = PositionTarget()
target.coordinate_frame = PositionTarget.FRAME_BODY_NED  # 机体NED系
target.type_mask = (...)  # 指定使用速度控制
target.velocity.x = 5.0   # NED系
```

## 关键差异

### 1. 消息类型

| 方面 | TwistStamped | PositionTarget |
|------|--------------|----------------|
| 消息类型 | geometry_msgs | mavros_msgs |
| 坐标系 | 不明确 | 明确指定 |
| 控制模式 | 仅速度 | 位置/速度/加速度 |
| type_mask | 无 | 精确控制使用哪些 |

### 2. 坐标系转换

#### FLU (Forward-Left-Up) - ROS标准
- X: 前
- Y: 左
- Z: 上

#### NED (North-East-Down) - PX4标准
- X: 北（前）
- Y: 东（右）
- Z: 下

#### 转换公式（参考v9.2）
```python
# FLU到NED转换
target.velocity.x = cmd.linear.x      # X前不变
target.velocity.y = -cmd.linear.y     # Y左→右（反转）
target.velocity.z = -cmd.linear.z     # Z上→下（反转）
target.yaw_rate = -cmd.angular.z      # 偏航率反转
```

### 3. MAVROS话题

| 话题 | 消息类型 | 用途 |
|------|----------|------|
| `/mavros/setpoint_velocity/cmd_vel` | TwistStamped | 速度控制（坐标系模糊） |
| `/mavros/setpoint_raw/local` | PositionTarget | 原始控制（坐标系明确） |

## 修复内容

### simple_drone_controller.py

#### 1. 导入PositionTarget
```python
from mavros_msgs.msg import State, PositionTarget
```

#### 2. 修改发布者
```python
# 修改前
self.cmd_vel_pub = rospy.Publisher(
    f'/{self.vehicle_ns}/mavros/setpoint_velocity/cmd_vel',
    TwistStamped, queue_size=1
)

# 修改后
self.setpoint_pub = rospy.Publisher(
    f'/{self.vehicle_ns}/mavros/setpoint_raw/local',
    PositionTarget, queue_size=1
)
```

#### 3. 修改主循环速度转发
```python
# 主循环中
with self.lock:
    # 选择速度命令
    if self.flight_mode == "WAYPOINT":
        cmd = self.waypoint_cmd  # Twist格式
    elif self.flight_mode == "TRACKING":
        cmd = self.tracking_cmd  # Twist格式
    else:
        cmd = Twist()
    
    # 转换为PositionTarget（FLU→NED）
    target = PositionTarget()
    target.coordinate_frame = PositionTarget.FRAME_BODY_NED
    target.type_mask = (IGNORE位置和加速度，只用速度和偏航率)
    
    # 坐标系转换
    target.velocity.x = cmd.linear.x      # X不变
    target.velocity.y = -cmd.linear.y     # Y反转
    target.velocity.z = -cmd.linear.z     # Z反转
    target.yaw_rate = -cmd.angular.z      # yaw反转
    
    # 发布
    self.setpoint_pub.publish(target)
```

#### 4. 修改起飞和初始化的setpoint
所有发送速度命令的地方都改用PositionTarget格式。

## type_mask说明

```python
# 使用速度控制，忽略位置和加速度
type_mask = (
    PositionTarget.IGNORE_PX +      # 忽略位置X
    PositionTarget.IGNORE_PY +      # 忽略位置Y
    PositionTarget.IGNORE_PZ +      # 忽略位置Z
    PositionTarget.IGNORE_AFX +     # 忽略加速度X
    PositionTarget.IGNORE_AFY +     # 忽略加速度Y
    PositionTarget.IGNORE_AFZ +     # 忽略加速度Z
    PositionTarget.IGNORE_YAW +     # 忽略偏航角（使用yaw_rate）
    PositionTarget.IGNORE_YAW_RATE  # 或忽略偏航率（使用yaw）
)
```

## 为什么这样修复有效

### 1. 坐标系明确
- `FRAME_BODY_NED`明确告诉PX4使用NED坐标系
- 避免了坐标系歧义导致的方向错误

### 2. 正确的转换
- Y和Z需要反转（FLU→NED）
- v9.2已经验证过这个转换是正确的

### 3. PX4原生支持
- `setpoint_raw/local`是PX4的原生接口
- 更稳定，更可靠

## 预期效果

修复后：
- ✅ 无人机朝向正确（不会转180度）
- ✅ 航点飞行方向正确
- ✅ 追踪方向正确
- ✅ 完全符合PX4的坐标系定义

## 测试验证

重新测试，应该看到：
1. 无人机0起飞后，向西飞行（朝向-X方向）
2. 无人机1起飞后，向东飞行（朝向+X方向）
3. 云台始终朝向飞行方向
4. 追踪时，人在前方→无人机向前飞

---
**记录人**：东华大学 Astraeus队  
**状态**：关键修复，参考v9.2稳定实现

