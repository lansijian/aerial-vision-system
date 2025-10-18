# 追踪方向反向问题修复

**日期**：2025-01-15  
**版本**：v11.0  
**问题**：追踪控制方向反了，向前飞变成向后飞

## 问题诊断

### 症状
用户报告："追踪的控制方向反向了，本来应该向前飞结果向后飞"

### 根本原因

发现了**两个关键错误**：

#### 1. 云台角度重复转换
```python
# 错误实现
self.gimbal_pitch = math.radians(rospy.get_param('~gimbal_pitch', -45.0))  # 转换为弧度

# 在追踪控制中
theta = math.radians(abs(self.gimbal_pitch))  # 再次转换！❌

# 在云台旋转矩阵中
cp = math.cos(self.gimbal_pitch)  # 期望是弧度，但已经是弧度了
```

**影响**：
- 追踪控制：theta = radians(radians(-45°)) ≈ -0.785弧度 的弧度值 ≈ -0.0137弧度
- 正确应该是：theta = radians(-45°) = -0.785弧度
- 错误导致：z距离计算错误，速度方向混乱

#### 2. 控制增益不匹配

| 参数 | v10.1.2（正确） | v11.0（错误） | 差异 |
|------|----------------|--------------|------|
| kp_xy | 0.5 | 0.8 | 60%增大 |
| kp_z | 1.0 | 1.0 | 一致 ✅ |
| kp_yaw | 0.002 | 1.2 | **600倍！** ❌ |

**kp_yaw从0.002变成1.2的影响**：
- 偏航控制过强
- 可能导致无人机快速旋转
- 影响整体控制稳定性

## 修复内容

### 1. 修正云台角度处理（target_tracker.py）

```python
# 修改前
self.gimbal_pitch = math.radians(rospy.get_param('~gimbal_pitch', -45.0))  # 错误

# 修改后
self.gimbal_pitch = rospy.get_param('~gimbal_pitch', -45.0)  # 保持度数 ✅
```

### 2. 修正云台旋转矩阵计算（target_tracker.py）

```python
# 修改前
cp = math.cos(self.gimbal_pitch)  # 期望弧度，但实际已经是弧度

# 修改后
pitch_rad = math.radians(self.gimbal_pitch)  # 正确转换
cp = math.cos(pitch_rad)  # 使用弧度 ✅
sp = math.sin(pitch_rad)
```

### 3. 修正控制增益（target_tracker.py）

```python
# 修改前
self.kp_xy = rospy.get_param('~kp_xy', 0.8)
self.kp_z = rospy.get_param('~kp_z', 1.0)
self.kp_yaw = rospy.get_param('~kp_yaw', 1.2)  # 错误：太大

# 修改后（参考v10.1.2）
self.kp_xy = rospy.get_param('~kp_xy', 0.5)  # v10.1.2值
self.kp_z = rospy.get_param('~kp_z', 1.0)
self.kp_yaw = rospy.get_param('~kp_yaw', 0.002)  # v10.1.2值（像素级增益）
```

### 4. 添加launch文件参数配置

在所有无人机的target_tracker配置中添加：
```xml
<!-- 控制增益（参考v10.1.2稳定值） -->
<param name="kp_xy" value="0.5" />
<param name="kp_z" value="1.0" />
<param name="kp_yaw" value="0.002" />
```

### 5. 添加调试日志（target_tracker.py）

```python
rospy.loginfo_throttle(2.0, 
    f"[追踪控制] 像素:(u={u:.0f},v={v:.0f}) "
    f"偏移:(u_off={u_offset:.0f},v_off={v_offset:.0f}) "
    f"距离z={z:.1f}m 高度={height:.1f}m "
    f"速度:X={cmd_vel.linear.x:.2f} Y={cmd_vel.linear.y:.2f} Z={cmd_vel.linear.z:.2f}")
```

## 测试和验证

### 1. 方向测试

观察追踪行为，验证方向正确性：
- 目标在图像**上方** → 无人机应该**向前**飞（linear.x > 0）
- 目标在图像**下方** → 无人机应该**向后**飞（linear.x < 0）
- 目标在图像**左侧** → 无人机应该**向左**飞（linear.y > 0）
- 目标在图像**右侧** → 无人机应该**向右**飞（linear.y < 0）

### 2. 查看调试日志

启动系统后，追踪目标时应该看到：
```
[追踪控制] 像素:(u=320,v=180) 偏移:(u_off=0,v_off=0) 距离z=4.2m 高度=3.0m 速度:X=0.00 Y=0.00 Z=0.00
```

- 如果目标在图像中心，偏移应该接近0
- 如果有偏移，速度应该合理（不会太大）

### 3. 控制参数调优

如果追踪效果仍不理想，可调整：

```xml
<!-- 增大响应速度 -->
<param name="kp_xy" value="0.6" />  <!-- 0.4-0.8 -->

<!-- 增强高度控制 -->
<param name="kp_z" value="1.2" />  <!-- 0.8-1.5 -->

<!-- 调整偏航灵敏度 -->
<param name="kp_yaw" value="0.003" />  <!-- 0.001-0.005 -->
```

## 如果方向仍然相反

如果修复后仍然方向相反，可能是坐标系定义问题。快速修复方法：

### 方案A：反转X轴
```python
# 在_compute_tracking_velocity()中
cmd_vel.linear.x = -cmd_vel.linear.x  # 反转前后方向
```

### 方案B：反转Y轴
```python
cmd_vel.linear.y = -cmd_vel.linear.y  # 反转左右方向
```

### 方案C：调整增益符号
```python
self.kp_xy = -0.5  # 取反
```

## 控制算法验证

### 完整的控制流程
```
1. 检测目标 → 获取像素坐标(u, v)
2. 计算偏移 → u_offset, v_offset
3. 计算中间速度 → u_velocity, v_velocity
4. 坐标系转换 → linear.x, linear.y
5. 高度控制 → linear.z
6. 偏航控制 → angular.z
7. 速度限制 → 最终速度
```

### 关键公式（与v10.1.2完全一致）
```python
u_velocity = -kp_xy * u_offset
v_velocity = -kp_xy * v_offset

linear.x = v_velocity * z / (v_offset*cos(45°) + fy*sin(45°))
linear.y = (z*u_velocity - u_offset*cos(45°)*linear.x) / fx
linear.z = kp_z * (ideal_distance - height)
angular.z = -kp_yaw * u_offset
```

## 预期效果

修复后：
- ✅ 追踪方向正确（向前是向前，不是向后）
- ✅ 响应速度合理（不会过快或过慢）
- ✅ 偏航控制稳定（不会剧烈旋转）
- ✅ 目标保持在图像中心附近

## 相关文档

- [TRACKING_ALGORITHM_COMPARISON.md](./TRACKING_ALGORITHM_COMPARISON.md) - 算法详细对比
- [FIX_TRACKING_CONTROL_2025-01-15.md](./FIX_TRACKING_CONTROL_2025-01-15.md) - 追踪控制修复
- [ALL_FIXES_SUMMARY_2025-01-15.md](./ALL_FIXES_SUMMARY_2025-01-15.md) - 所有修复汇总

---
**记录人**：东华大学 Astraeus队  
**状态**：已修复，待测试验证

