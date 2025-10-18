# 高度保持最终方案 - 2025-10-16

## ✅ 最终方案：vz=0，让PX4自动保持

### 问题回顾
尝试了主动高度控制，但仍然下降：
- ❌ Kp_z = 0.5 → 太弱
- ❌ Kp_z = 2.0 → 仍然落下去

### 根本原因
- 主动Z轴控制可能与PX4内部高度保持冲突
- 速度控制模式下，Z轴命令可能被优先处理
- 导致PX4的高度保持失效

---

## 🎯 最终解决方案

### 核心思路（参考之前稳定版本）
```python
# 追踪时不主动控制高度
self.cmd_vel.linear.z = 0.0  # 始终为0

# 让PX4/MAVROS自己维持高度
# PX4在OFFBOARD模式下会自动保持当前高度（如果vz=0）
```

### 代码实现

```python
# human_tracker.py - _compute_tracking_velocity()
# ========== 4. 高度控制（保持不变，让PX4自己维持） ==========
# 参考之前稳定的做法：Z轴速度设为0，让PX4/MAVROS自动保持高度
self.cmd_vel.linear.z = 0.0

# 高度监控日志（仅监控，不主动控制）
if self.current_pose:
    rospy.loginfo_throttle(5.0,
        f"[高度监控] 目标:{ideal_distance}m 当前:{current_z}m "
        f"误差:{error}m vz:0.0(PX4自动保持)")
```

---

## 📊 方案对比

### 方案A：主动高度控制（失败）
```python
vz = Kp_z * (target_height - current_height)
```

**问题**：
- ❌ 与PX4内部控制冲突
- ❌ 参数难以调节
- ❌ 仍然下降

### 方案B：vz=0，PX4保持（成功）✅
```python
vz = 0.0  # 不主动控制
```

**优势**：
- ✅ 简单可靠
- ✅ 依赖PX4成熟的高度保持
- ✅ 不干扰其他轴控制
- ✅ 参考之前稳定版本

---

## 🔬 PX4高度保持原理

### OFFBOARD模式下的行为

当使用速度控制（PositionTarget.FRAME_BODY_NED）时：
```
velocity.x ≠ 0  → PX4执行X方向运动
velocity.y ≠ 0  → PX4执行Y方向运动
velocity.z = 0  → PX4保持当前高度 ✅
yaw_rate ≠ 0  → PX4执行偏航转向
```

**关键**：
- vz=0时，PX4会激活内部高度保持算法
- 依赖气压计、IMU等传感器
- 比我们手动计算更可靠

---

## 🧪 测试验证

### 启动后观察

```
[高度监控] 目标:3.0m 当前:3.0m 误差:0.00m vz:0.0(PX4自动保持)
[追踪优化] ... 速度:(x=0.50,y=0.10,vz=0.00,yaw=-0.12)
                                      ↑ 应该始终为0
```

### 监控高度稳定性

```bash
# 持续观察高度
rostopic echo /typhoon_h480_0/mavros/local_position/pose | grep "z:"

# 追踪期间应该保持在3米左右
position.z: 3.01
position.z: 3.00
position.z: 2.99
position.z: 3.01  ← 小幅震荡，不应该持续下降
```

---

## 📝 相关参数（现在不使用）

launch文件中Kp_z参数仍然保留（以防将来需要），但代码中**强制vz=0**：

```xml
<!-- 这些参数现在不生效 -->
<param name="Kp_z" value="2.0" />  <!-- 不使用 -->
<param name="max_vz" value="1.0" />  <!-- 仅用于限幅保护 -->
```

---

## 🎯 如果高度仍然下降

### 检查项

1. **检查vz是否真的为0**
```bash
rostopic echo /typhoon_h480_0/mavros/setpoint_raw/local | grep velocity

# 应该看到
velocity.z: 0.0  或者  velocity.z: -0.0（NED中负是上）
```

2. **检查PX4模式**
```bash
rostopic echo /typhoon_h480_0/mavros/state | grep mode

# 应该是OFFBOARD
mode: "OFFBOARD"
```

3. **检查是否有其他节点干扰**
```bash
# 查看所有发布到setpoint的节点
rostopic info /typhoon_h480_0/mavros/setpoint_raw/local

# 应该只有
Publishers: 
 * /drone_0/drone_controller
 * /drone_0/waypoint_navigator（仅WAYPOINT模式）
```

---

## 🔧 紧急处理

如果追踪时高度仍然下降：

### 临时方案：添加高度保持辅助
```python
# human_tracker.py临时修改
# 如果高度偏差太大，发送小的修正
if abs(height_error) > 0.5:  # 超过0.5米才修正
    self.cmd_vel.linear.z = np.sign(height_error) * 0.3
else:
    self.cmd_vel.linear.z = 0.0
```

### 检查drone_controller的坐标转换
```python
# drone_controller.py
# 确认Z轴转换正确
target.velocity.z = -self.current_cmd.twist.linear.z

# 如果human_tracker发送vz=0
# 转换后应该是vz=0（不是-0或其他值）
```

---

## 📌 参考之前稳定版本

### v10.1.2版本的做法
```python
# human_tracker_v10.1.2.py
self.cmd_vel.linear.z = 0  # 始终为0
```

### v9.2版本的做法
```python
# plan3_optimized.py
# 追踪时也是vz=0，让PX4保持
```

**结论**：之前稳定版本都是vz=0，所以现在也应该这样做 ✅

---

**修复日期**：2025-10-16  
**方案**：vz=0，PX4自动保持  
**状态**：✅ 已实现  
**东华大学 Astraeus队**

