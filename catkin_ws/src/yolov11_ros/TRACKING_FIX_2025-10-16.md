# 追踪问题诊断与修复 - 2025-10-16深夜

## 🔴 用户反馈的问题

1. **检测到人之后没有正常追踪**
2. **高度失控（飞到6米）**
3. **目标居中但无人机左右晃动**

---

## 🐛 已修复的Bug

### Bug 1：变量未定义导致崩溃 ✅
**错误**：`UnboundLocalError: local variable 'Kp_yaw_adaptive' referenced before assignment`

**原因**：死区内没有定义`Kp_yaw_adaptive`，但日志输出时引用了

**修复**（human_tracker.py 第373行）：
```python
# 提前计算（在死区判断之前）
Kp_yaw_adaptive = self.Kp_yaw_base / max(box_height, 20)

# 然后再做死区判断
if abs(u_offset) < 15:
    yaw_rate = 0.0
```

---

### Bug 2：状态不一致导致追踪不触发 ✅
**问题**：`tracking_active=True`但`flight_mode="WAYPOINT"`，状态矛盾

**修复**（human_tracker.py 第297行）：
```python
# 旧代码（错误）
if not self.tracking_active or ...  # ❌ 状态可能不一致

# 新代码（正确）
if self.flight_mode != "TRACKING" or ...  # ✅ 以真实模式为准
```

**同步修复**（human_tracker.py 第250-256行）：
```python
def _mode_callback(self, msg):
    if self.flight_mode == "TRACKING":
        self.tracking_active = True  # 同步设置
    else:
        self.tracking_active = False
        self.tracking_requested = False
```

---

### Bug 3：回调条件限制导致速度丢失 ✅
**问题**：`if self.flight_mode == "TRACKING"`导致模式切换瞬间丢失速度命令

**修复**（drone_controller.py 第164行）：
```python
# 旧代码
def _tracker_cmd_callback_twist(self, msg):
    if self.flight_mode == "TRACKING":  # ❌ 限制条件
        self.current_cmd = cmd_stamped

# 新代码
def _tracker_cmd_callback_twist(self, msg):
    # 直接缓存，不检查flight_mode（避免丢失）
    self.current_cmd = cmd_stamped
```

---

### Bug 4：坐标系转换缺失 ✅
**问题**：`velocity.z`和`yaw_rate`没有做FLU→NED转换

**修复**（drone_controller.py 第396-397行）：
```python
# 旧代码
target.velocity.z = self.current_cmd.twist.linear.z      # ❌ 没转换
target.yaw_rate = self.current_cmd.twist.angular.z       # ❌ 没转换

# 新代码
target.velocity.z = -self.current_cmd.twist.linear.z     # ✅ FLU→NED
target.yaw_rate = -self.current_cmd.twist.angular.z      # ✅ FLU→NED
```

---

### Bug 5：后退响应不足 ✅
**问题**：行人在图像下半区往后走时，无人机不后退

**修复**（human_tracker.py 第351-363行）：
```python
# 旧代码：只用框高
forward_speed = Kp_distance * (height_error / ideal_box_height)

# 新代码：框高 + 纵向偏移
forward_speed_distance = Kp_distance * (height_error / ideal_box_height)
forward_speed_position = -Kp_v_offset * v_offset  # v>0（下半区）→后退
forward_speed = forward_speed_distance + forward_speed_position
```

**参数**：`Kp_v_offset = 0.003`（可调）

---

### Bug 6：高度漂移到6米 ✅
**问题**：vz=0只能维持当前高度，无法主动下降

**修复**（human_tracker.py 第414-434行）：
```python
# 安全限制：正常时vz=0，异常时主动修正
if abs(height_error) > 1.0:
    # 高度严重偏离（>4米或<2米）
    vz_control = Kp_z * height_error
    self.cmd_vel.linear.z = np.clip(vz_control, -max_vz, max_vz)
    rospy.logwarn(f"⚠️ 高度异常！...主动修正vz:{vz}")
else:
    # 高度正常（2-4米），vz=0
    self.cmd_vel.linear.z = 0.0
```

**效果**：
- 2-4米范围：vz=0（PX4自动保持）
- <2米或>4米：主动P控制修正

---

## 🎯 抗震荡优化

### 横向死区（human_tracker.py 第370-375行）
```python
# ±10像素内不横向移动
if abs(u_offset) < 10:
    lateral_speed = 0.0
else:
    lateral_speed = self.Kp_lateral * u_offset
```

### 偏航死区（human_tracker.py 第376-394行）
```python
# ±15像素内不偏航
if abs(u_offset) < 15:
    yaw_rate = 0.0
    self.yaw_error_integral = 0.0  # 清空积分
else:
    # 正常PI控制
```

---

## 📊 修复文件总结

| 文件 | 修改内容 | 行号 |
|------|---------|------|
| human_tracker.py | 修复Kp_yaw_adaptive未定义 | 373 |
| human_tracker.py | 状态统一（flight_mode） | 297, 250-256, 319 |
| human_tracker.py | 添加纵向偏移控制 | 351-363 |
| human_tracker.py | 高度安全限制 | 414-434 |
| human_tracker.py | 横向死区10px | 370-375 |
| human_tracker.py | 偏航死区15px | 376-394 |
| drone_controller.py | 移除回调条件检查 | 164 |
| drone_controller.py | FLU→NED转换（z, yaw_rate） | 396-397 |
| drone_controller.py | 增强调试日志 | 173-174, 179, 403-404 |

---

## 🧪 验证要点

### 1. 追踪触发
```bash
# 应该看到完整流程
🎯 无人机X: 发现red目标，请求切换到追踪模式
[DEBUG] 发布追踪请求到话题: /drone_X/request_tracking_mode
[DEBUG] 无人机X 收到追踪请求: data=True, is_flying=True
🎯 无人机X: 切换到追踪模式
[DEBUG] 无人机X 模式切换: WAYPOINT → TRACKING
✅ 无人机X 进入TRACKING模式，开始追踪
```

### 2. 速度发布
```bash
# 每5秒应该看到
[DEBUG-追踪] 无人机X 发布速度: vx=..., vy=..., vz=..., yaw=...
[追踪速度FLU] 无人机X 收到并缓存: vx=..., vy=..., vz=..., yaw=...
[控制权-TRACKING] 无人机X yaw=... FLU:(...) → NED:(...)
```

### 3. 死区效果
```bash
# 目标居中时（|u_offset| < 15）
[追踪优化] 偏移:(u=8,v=...) ... 机体速度:(vx=...,vy=0.00,vz=...,yaw=0.00)
                                                   ↑死区      ↑死区
```

### 4. 高度安全限制
```bash
# 高度正常（2-4米）
[高度保持] 目标:3.0m 当前:3.2m vz:0.0

# 高度异常（>4米或<2米）
⚠️ 高度异常！目标:3.0m 当前:6.2m 误差:-3.2m → 主动修正vz:-1.00
```

### 5. 后退响应
```bash
# 目标在下半区（v_offset > 0）
[追踪优化] 偏移:(u=5,v=80) ... 机体速度:(vx=-0.24,...)
                      ↑下半区                ↑后退
```

---

## 🎛️ 参数调节建议

### 如果追踪丢失频繁
```xml
<!-- 降低ideal_box_height（允许更远距离追踪） -->
<param name="ideal_box_height" value="80" />  <!-- 从160降低 -->

<!-- 提高前后速度 -->
<param name="max_vx" value="4.0" />  <!-- 从3.0提高 -->
```

### 如果仍然震荡
```python
# human_tracker.py 增大死区
if abs(u_offset) < 20:  # 从10或15提高到20
    lateral_speed = 0.0
    yaw_rate = 0.0
```

### 如果后退响应慢
```python
# human_tracker.py 第359行
Kp_v_offset = 0.005  # 从0.003提高到0.005
```

### 如果高度控制太敏感
```python
# human_tracker.py 第420行
if abs(height_error) > 1.5:  # 从1.0提高到1.5
```

---

## 📋 调试命令

```bash
# 1. 监听模式切换
rostopic echo /drone_0/flight_mode

# 2. 监听速度命令
rostopic echo /drone_0/human_tracker/cmd_vel

# 3. 监听MAVROS接收
rostopic echo /typhoon_h480_0/mavros/setpoint_raw/local

# 4. 检查高度
rostopic echo /typhoon_h480_0/mavros/local_position/pose | grep "z:"

# 5. 手动触发追踪（测试用）
rostopic pub -1 /drone_0/request_tracking_mode std_msgs/Bool "data: true"
```

---

**修复完成时间**：2025-10-16深夜  
**状态**：✅ 6个关键bug已修复  
**版本**：v10.1.2-stable-fixed  
**东华大学 Astraeus队**

