# 追踪控制算法对比分析

## v10.1.2 vs v11.0 控制算法对比

### 控制增益对比

| 参数 | v10.1.2 | v11.0（修复前） | v11.0（修复后） |
|------|---------|----------------|----------------|
| Kp_xy / kp_xy | 0.5 | 0.8 ❌ | 0.5 ✅ |
| Kp_z / kp_z | 1.0 | 1.0 ✅ | 1.0 ✅ |
| Kp_yaw / kp_yaw | 0.002 | 1.2 ❌ | 0.002 ✅ |

### 云台角度处理对比

**v10.1.2**：
```python
self.gimbal_pitch = rospy.get_param('~gimbal_pitch', -45.0)  # 度数

# 在追踪控制中使用
theta = math.radians(abs(self.gimbal_pitch))  # 转换为弧度
z = height / max(math.sin(theta), 0.1)
```

**v11.0（修复前）**：
```python
self.gimbal_pitch = math.radians(rospy.get_param('~gimbal_pitch', -45.0))  # 错误：转换为弧度

# 在追踪控制中使用
theta = math.radians(abs(self.gimbal_pitch))  # 错误：再次转换！
z = height / max(math.sin(theta), 0.1)

# 在云台旋转矩阵中使用
cp = math.cos(self.gimbal_pitch)  # 错误：应该是弧度
sp = math.sin(self.gimbal_pitch)
```

**v11.0（修复后）**：
```python
self.gimbal_pitch = rospy.get_param('~gimbal_pitch', -45.0)  # 度数 ✅

# 在追踪控制中使用
theta = math.radians(abs(self.gimbal_pitch))  # 正确转换 ✅
z = height / max(math.sin(theta), 0.1)

# 在云台旋转矩阵中使用
pitch_rad = math.radians(self.gimbal_pitch)  # 正确转换 ✅
cp = math.cos(pitch_rad)
sp = math.sin(pitch_rad)
```

### 控制算法对比

**像素偏移**（完全相同）：
```python
u_offset = u - self.cx
v_offset = v - self.cy
```

**中间速度**（完全相同）：
```python
u_velocity = -self.kp_xy * u_offset
v_velocity = -self.kp_xy * v_offset
```

**机体坐标系转换**（完全相同）：
```python
cmd_vel.linear.x = v_velocity * z / max(v_offset * math.cos(math.radians(45)) + self.fy * math.sin(math.radians(45)), 1.0)
cmd_vel.linear.y = (z * u_velocity - u_offset * math.cos(math.radians(45)) * cmd_vel.linear.x) / max(self.fx, 1.0)
cmd_vel.linear.z = self.kp_z * (self.ideal_tracking_distance - height)
cmd_vel.angular.z = -self.kp_yaw * u_offset
```

## 问题诊断

### 问题1：云台角度重复转换 ✅ 已修复
- **原因**：参数读取时转换为弧度，使用时又转换一次
- **影响**：云台旋转矩阵错误，坐标计算错误
- **修复**：保持参数为度数，在使用时才转换

### 问题2：控制增益不匹配 ✅ 已修复
- **原因**：kp_yaw从0.002改为1.2（600倍差异）
- **影响**：偏航控制过强，可能导致方向混乱
- **修复**：恢复为v10.1.2的值

### 问题3：可能的坐标系问题？

如果修复后仍然方向相反，可能的原因：
1. **相机安装方向**：相机坐标系定义问题
2. **机体坐标系**：FRD vs FLU
3. **符号问题**：linear.x的符号

## 调试步骤

### 1. 测试追踪方向

启动系统，观察追踪行为：
- 目标在图像**上方** → 无人机应该**向前**飞
- 目标在图像**下方** → 无人机应该**向后**飞
- 目标在图像**左侧** → 无人机应该**向左**飞
- 目标在图像**右侧** → 无人机应该**向右**飞

### 2. 检查像素偏移

添加调试日志：
```python
rospy.loginfo(f"像素: u={u:.0f}, v={v:.0f}, "
              f"偏移: u_offset={u_offset:.0f}, v_offset={v_offset:.0f}, "
              f"速度: X={cmd_vel.linear.x:.2f}, Y={cmd_vel.linear.y:.2f}")
```

### 3. 符号测试

如果方向仍然相反：
```python
# 尝试取反linear.x
cmd_vel.linear.x = -cmd_vel.linear.x
```

## 修复记录

### 2025-01-15 修复
1. ✅ 云台角度参数保持度数
2. ✅ 控制增益恢复v10.1.2值
3. ✅ 云台旋转矩阵正确转换
4. ⏳ 等待测试验证方向是否正确

---
**记录人**：东华大学 Astraeus队  
**状态**：待测试验证

