# 速度为零问题修复记录

**日期**：2025-01-15  
**版本**：v11.0  
**问题**：无人机成功起飞后，航点任务和追踪任务速度一直为0，无人机悬停不动

## 问题现象

从终端日志看到：
```
[INFO] 起飞完成，开始航点任务
[INFO] 当前飞行模式: waypoint
[INFO] 已加载21个航点
[INFO] 执行中 模式:waypoint 速度:X=0.00 Y=0.00 Z=0.00
[航点任务] 切换到waypoint模式，暂停航点任务
```

```
[目标追踪] 申请追踪red色目标
[多机管理器] 分配red色目标给无人机0
[多机管理器] 释放无人机0的目标red
[多机管理器] 分配red色目标给无人机0
[多机管理器] 释放无人机0的目标red
...（频繁分配和释放）
[任务控制器] 执行中 模式:tracking 速度:X=0.00 Y=0.00 Z=0.00
```

## 根本原因

### 1. 航点任务未激活
**问题**：任务控制器在初始化时发布航点，但此时航点任务模块还未准备好接收

**时序问题**：
1. 任务控制器启动，加载航点并发布（航点任务可能还未订阅）
2. 航点任务模块启动，订阅话题（但已错过航点发布）
3. 起飞完成，切换到航点模式
4. 航点任务模块收到模式切换，但 `self.waypoints` 为空
5. 检查 `self.is_active = (self.flight_mode == "waypoint" and bool(self.waypoints))` 
6. 因为 `bool([]) == False`，所以 `is_active = False`
7. 控制循环中发送零速度

### 2. 目标分配频繁变化
**问题**：多机管理器过早释放目标

**问题逻辑**：
```python
# 旧逻辑
if mode != 'tracking':
    # 不在追踪模式，释放目标
    inactive_drones.append(drone_id)
```

**时序冲突**：
1. 无人机在航点模式下检测到目标，申请追踪
2. 多机管理器分配目标
3. 但此时 `flight_mode = 'waypoint'`（还未切换）
4. 多机管理器检查发现不在tracking模式，立即释放目标
5. 无人机切换到tracking模式
6. 但目标已被释放，`is_tracking_approved = False`
7. 因为未批准，不计算追踪速度

### 3. 追踪批准状态不稳定
**问题**：目标追踪器在分配表中消失时立即取消批准

```python
# 旧逻辑
else:
    # 未被分配目标
    if self.is_tracking_approved:
        self.is_tracking_approved = False
```

导致：
- 目标分配和释放频繁变化
- 批准状态跟着频繁变化
- 速度计算被频繁打断

## 修复内容

### 1. 修复航点发布时序问题
**文件**：`mission_controller.py`

**方案**：在起飞完成后再发布航点，确保航点任务模块已经准备好

```python
# 修改前（第169-171行）
if self.waypoints:
    self._publish_waypoints()  # 初始化时发布

# 修改后
# 航点将在起飞完成后发布（避免时序问题）

# 在起飞完成处添加（第587-591行）
if self.waypoints:
    for i in range(3):
        self._publish_waypoints()  # 多次发布确保接收
        rospy.sleep(0.1)
```

### 2. 修复OFFBOARD模式设置
**文件**：`mission_controller.py`  
**函数**：`_set_offboard_mode()`（第335-400行）

**方案**：参考旧版本v10.1.2的实现

```python
# 关键改进：
1. 先发送100次setpoint（5秒），确保PX4准备就绪
2. 在发送10次后才开始尝试切换模式
3. 检查是否已切换成功，成功后再多发送20次确保稳定
4. 每秒输出一次进度
```

### 3. 修复多机管理器释放逻辑
**文件**：`multi_drone_manager.py`  
**位置**：第165-181行

```python
# 修改前
if mode != 'tracking':
    # 不在追踪模式，释放目标
    inactive_drones.append(drone_id)

# 修改后
# 只有当无人机返回航点模式且没有目标申请时才释放
if mode == 'waypoint' and drone_id not in self.target_claims:
    inactive_drones.append(drone_id)
```

**改进点**：
- 给无人机时间切换到tracking模式
- 如果还有目标申请，不释放目标
- 只在明确返回航点模式时才释放

### 4. 修复目标追踪器重复申请
**文件**：`target_tracker.py`  
**位置**：第470-477行

```python
# 修改前
if target:
    if not self.tracking_target or target.Class != self.tracking_color:
        ...
        self._claim_target(target.Class)  # 每次都申请

# 修改后
if target:
    if not self.tracking_target or target.Class != self.tracking_color:
        ...
        self._claim_target(target.Class)  # 只在目标改变时申请
    else:
        # 同一目标，只更新tracking_target
        self.tracking_target = target
```

### 5. 修复追踪批准取消逻辑
**文件**：`target_tracker.py`  
**函数**：`_target_assignment_callback()`（第149-174行）

```python
# 修改前
else:
    # 未被分配目标
    if self.is_tracking_approved:
        self.is_tracking_approved = False  # 立即取消

# 修改后
else:
    # 未被分配目标，但只在明确切换回航点模式时才取消批准
    # 不要立即取消，避免模式切换期间的问题
    if self.flight_mode == 'waypoint' and self.is_tracking_approved:
        self.is_tracking_approved = False
```

### 6. 其他改进

**控制循环频率**（mission_controller.py 第174行）：
```python
# 修改前：50Hz
self.control_timer = rospy.Timer(rospy.Duration(0.02), self._control_loop)

# 修改后：30Hz（与旧版本一致）
self.control_timer = rospy.Timer(rospy.Duration(0.033), self._control_loop)
```

**OFFBOARD模式保持**（mission_controller.py 第525-532行）：
```python
# 新增：如果已经解锁但不在OFFBOARD模式，尝试重新进入
if self.is_armed and not self.is_offboard and self.mission_state not in [MissionState.IDLE, MissionState.LANDED]:
    rospy.logwarn_throttle(1.0, f"[任务控制器] 检测到OFFBOARD模式退出，当前模式: {self.mavros_state.mode}")
    try:
        self.set_mode_service(custom_mode="OFFBOARD")
    except:
        pass
```

## 预期效果

修复后系统应该：
1. ✅ 航点任务正确激活，无人机按航点飞行
2. ✅ 检测到目标后平滑切换到追踪模式
3. ✅ 目标分配稳定，不频繁释放
4. ✅ 追踪速度正常计算，无人机跟踪目标
5. ✅ 目标丢失后返回航点巡检

## 测试建议

1. 观察航点任务日志：
   - 应该看到 `[航点任务] 接收到21个航点`
   - 应该看到 `[航点任务] 激活航点模式，执行21个航点`

2. 观察速度指令：
   - 航点模式下速度不应该全为0
   - 追踪模式下速度应该根据目标位置计算

3. 观察目标分配：
   - 不应该频繁分配和释放同一个目标
   - 分配后应该保持稳定

## 相关文档

- [FIX_ARMING_ISSUE_2025-01-15.md](./FIX_ARMING_ISSUE_2025-01-15.md) - 解锁失败问题修复
- [V11_REFACTOR_SUMMARY.md](./V11_REFACTOR_SUMMARY.md) - v11.0重构总结

---
**记录人**：东华大学 Astraeus队  
**审核状态**：已完成

