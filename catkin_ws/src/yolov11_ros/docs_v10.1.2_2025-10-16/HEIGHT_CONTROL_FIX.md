# 高度保持问题修复 - 2025-10-16

## 🔴 问题现象

无人机进入追踪模式后**逐渐下降**，没有保持3米高度。

---

## 🔍 问题诊断

### 原配置（太弱）
```python
Kp_z = 0.5    # 高度控制增益
max_vz = 0.5  # 垂直速度上限
```

### 计算示例
```
当前高度：2.7m
目标高度：3.0m
高度误差：0.3m

vz = Kp_z * error = 0.5 * 0.3 = 0.15 m/s
限制后vz = clip(0.15, -0.5, 0.5) = 0.15 m/s
```

**问题**：
- ❌ 0.15 m/s上升速度太慢
- ❌ 如果无人机同时在下降（比如追踪时前倾），可能抵消不了

---

## ✅ 解决方案

### 增强高度控制

```xml
<!-- launch文件配置 -->
<param name="Kp_z" value="2.0" />    <!-- 从0.5提高到2.0（4倍） -->
<param name="max_vz" value="1.0" />  <!-- 从0.5提高到1.0（2倍） -->
```

### 修复后的计算

```
当前高度：2.7m
目标高度：3.0m
高度误差：0.3m

vz = Kp_z * error = 2.0 * 0.3 = 0.6 m/s
限制后vz = clip(0.6, -1.0, 1.0) = 0.6 m/s ✅
```

**改进**：
- ✅ 0.6 m/s上升速度（快4倍）
- ✅ 上限提高到1.0 m/s，允许更快调整

---

## 🔬 高度控制逻辑

### 代码实现

```python
# human_tracker.py - _compute_tracking_velocity()
if self.current_pose:
    # 计算高度误差
    height_error = self.ideal_distance - self.current_pose.position.z
    # ideal_distance = 3.0m
    # current_z = 当前高度
    
    # P控制
    vz_control = self.Kp_z * height_error
    
    # 限幅
    self.cmd_vel.linear.z = np.clip(vz_control, -self.max_vz, self.max_vz)
    
    # 调试日志
    rospy.loginfo_throttle(5.0,
        f"[高度保持] 目标:{ideal_distance}m 当前:{current_z}m "
        f"误差:{height_error}m vz:{vz}m/s")
```

### 坐标系转换（drone_controller.py）

```python
# FLU → NED转换
# FLU: Z上为正
# NED: Z下为正
target.velocity.z = -self.current_cmd.twist.linear.z

# 例子：
# human_tracker(FLU): vz = 0.6（想要上升）
# drone_controller(NED): vz = -0.6（NED中负值是上升）✅
```

---

## 📊 高度控制强度对比

| 配置 | Kp_z | max_vz | 0.3m误差响应 | 效果 |
|------|------|--------|-------------|------|
| 旧配置 | 0.5 | 0.5 | 0.15 m/s | ❌ 太弱 |
| 新配置 | 2.0 | 1.0 | 0.6 m/s | ✅ 强力 |
| 激进配置 | 3.0 | 1.5 | 0.9 m/s | ⚠️ 可能抖动 |

---

## 🧪 测试验证

### 1. 观察高度日志

启动追踪后应该看到：
```
[高度保持] 目标:3.0m 当前:2.8m 误差:0.20m vz:0.40m/s
[高度保持] 目标:3.0m 当前:2.9m 误差:0.10m vz:0.20m/s
[高度保持] 目标:3.0m 当前:3.0m 误差:0.00m vz:0.00m/s ✅
```

### 2. 监控实际高度

```bash
# 实时查看高度
rostopic echo /typhoon_h480_0/mavros/local_position/pose | grep "z:"

# 应该看到
position.z: 3.0  # 稳定在3米
position.z: 3.0
position.z: 3.0
```

### 3. 检查速度命令

```
[追踪优化] ... 速度:(x=0.50,y=0.10,vz=0.40,yaw=-0.12)
                                      ↑ 应该有vz输出

[控制权-TRACKING] 转发追踪速度(NED): vx=0.50 vy=-0.10 yaw=0.12
# NED中vz会在控制循环中转换和发送
```

---

## 🎛️ 现场调节

### 如果仍然下降

逐步提高Kp_z：
```xml
<!-- 尝试更强的控制 -->
<param name="Kp_z" value="3.0" />
<param name="max_vz" value="1.5" />
```

### 如果高度抖动

降低Kp_z或增加阻尼：
```xml
<!-- 更平稳但响应慢 -->
<param name="Kp_z" value="1.5" />
<param name="max_vz" value="0.8" />
```

### 如果追踪时想要不同高度

修改ideal_distance：
```xml
<!-- 改为4米追踪 -->
<param name="ideal_tracking_distance" value="4.0" />
```

---

## 🔧 紧急修复方案

如果现场发现高度控制完全失效，可以临时禁用Z轴控制：

```python
# human_tracker.py临时修改
self.cmd_vel.linear.z = 0.0  # 强制为0，让PX4自己保持高度
```

**注意**：这会让无人机依赖PX4的高度保持，可能不够精确。

---

## 📊 参数总结

| 参数 | 旧值 | 新值 | 改进 |
|------|------|------|------|
| Kp_z | 0.5 | 2.0 | 4倍增益 ✅ |
| max_vz | 0.5 | 1.0 | 2倍上限 ✅ |
| 响应速度 | 0.15m/s | 0.6m/s | 4倍快 ✅ |

---

**修复日期**：2025-10-16  
**版本**：v10.1.2-optimized  
**状态**：✅ 高度保持增强4倍  
**东华大学 Astraeus队**

