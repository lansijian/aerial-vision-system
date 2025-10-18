# v11.1 最终控制架构 - 2025-10-16

## 核心原则

**航点和追踪完全隔离，绝不冲突**

## 控制流程

### WAYPOINT模式（航点巡检）

```
waypoint_mission.py
  ├── is_active = True
  ├── 计算航点速度（NED机体坐标）
  ├── 构造PositionTarget（速度控制）
  └──> 直接发布到 /typhoon_h480_X/mavros/setpoint_raw/local

target_tracker.py
  ├── flight_mode = "waypoint"
  └── 不发布任何命令 ✅

mission_controller.py
  ├── flight_mode = WAYPOINT
  └── 不发布任何命令（should_publish = False）✅
```

### TRACKING模式（目标追踪）

```
waypoint_mission.py
  ├── is_active = False
  └── 停止发布 ✅

target_tracker.py
  ├── flight_mode = "tracking"
  ├── 计算追踪速度（FLU坐标）
  ├── 构造Twist消息
  └──> 发布到 /drone_X/target_tracker/cmd_vel

mission_controller.py
  ├── flight_mode = TRACKING
  ├── 订阅 /drone_X/target_tracker/cmd_vel
  ├── 转换Twist（FLU）→ PositionTarget（NED）
  └──> 发布到 /typhoon_h480_X/mavros/setpoint_raw/local
```

## 关键检查点

### 1. waypoint_mission的激活逻辑
```python
def _flight_mode_callback(self, msg):
    self.flight_mode = msg.data
    # 关键：只有在waypoint模式且有航点时才激活
    self.is_active = (self.flight_mode == "waypoint" and bool(self.waypoints))
```

**验证**：
- waypoint模式 → is_active=True → 开始发布
- tracking模式 → is_active=False → 停止发布 ✅

### 2. target_tracker的发布逻辑
```python
def _control_loop(self, event):
    # 只在tracking模式时工作
    if self.flight_mode != "tracking":
        return  # 完全不发布 ✅
    
    # 计算追踪速度
    cmd_vel = self._compute_tracking_velocity(target)
    
    # 发布到controller
    self.cmd_vel_pub.publish(cmd_vel)
```

### 3. mission_controller的发布逻辑
```python
should_publish = False

if mission_state == EXECUTING and flight_mode == TRACKING:
    should_publish = True  # 追踪模式发布
elif mission_state in [TAKEOFF, LANDING, ...]:
    should_publish = True  # 其他状态发布

if should_publish:
    # 转换并发布
    self.setpoint_pub.publish(target)
```

**关键**：
- EXECUTING+WAYPOINT → should_publish=False → 不发布 ✅
- EXECUTING+TRACKING → should_publish=True → 发布追踪速度 ✅

## 模式切换流程

### 航点→追踪

```
1. target_tracker检测到目标
2. 发布request_tracking_mode
3. mission_controller接收，切换flight_mode=TRACKING
4. 发布flight_mode="tracking"
5. waypoint_mission接收，is_active=False，停止发布
6. target_tracker接收，开始计算并发布追踪速度
7. mission_controller转换并发布到MAVROS
```

### 追踪→航点

```
1. target_tracker目标丢失1.5秒
2. 发布request_waypoint_mode
3. mission_controller接收，切换flight_mode=WAYPOINT
4. 发布flight_mode="waypoint"
5. target_tracker接收，停止发布
6. waypoint_mission接收，is_active=True，恢复发布
```

## 调试日志应该显示

### WAYPOINT模式
```
[航点任务] 激活航点模式，执行20个航点
[航点0] WP2:(-15,0,3.0) ...速度NED:(5.00,0.00,-0.00)
```

### TRACKING模式
```
🎯 无人机0: 切换到追踪模式
⏸️  无人机0: 航点模式暂停  ← waypoint_mission停止
[追踪0] 目标像素:(320,180) ...速度:X=0.00 Y=0.00 Z=0.00
```

### 返回WAYPOINT
```
目标丢失，请求返回航点
🔄 无人机0: 返回航点模式
🚁 无人机0: 航点模式激活  ← waypoint_mission恢复
```

## 问题排查

如果追踪还是被打断：

### 检查1：waypoint是否真正停止
```bash
rostopic hz /typhoon_h480_0/mavros/setpoint_raw/local
```

在TRACKING模式时，应该看到频率保持在30Hz（来自controller），不应该有20Hz的waypoint信号。

### 检查2：controller是否在TRACKING时发布
查看日志，应该看到`[任务控制器] 执行中 模式:tracking 速度:X=X.XX ...`

### 检查3：模式切换日志
应该清楚看到模式切换的顺序。

---
**版本**：v11.1 Final  
**日期**：2025-10-16  
**状态**：✅ 隔离完成，绝不冲突

