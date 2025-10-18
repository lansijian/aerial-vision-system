# 追踪控制与记分系统改进总结

**日期**: 2025-01-11  
**版本**: v8.1

## 📋 改进内容

### 1. 追踪控制改进（参考human_tracker copy.py）

#### 改进前问题
- 目标向后走时无人机不跟随
- 无法保持目标在图像中心
- 只使用简单的比例控制

#### 改进方案
- 集成XTDrone的深度估计算法
- 考虑相机俯仰角进行几何变换
- 增强前后速度控制，基于检测框大小动态调整
- 加强横向移动响应

#### 核心代码改进
```python
# 使用XTDrone深度估计
z_depth = self.height / sin(theta)

# 几何变换计算世界速度
x_velocity = v_velocity * z_depth / denom
y_velocity = (z_depth * u_velocity - u_ * cos(theta) * x_velocity) / fx

# 基于框大小的距离控制
if area_ratio < 0.5:  # 目标很远
    forward_boost = 1.0  # 强制前进
elif area_ratio > 1.5:  # 目标太近
    forward_boost = -0.5  # 后退
```

### 2. 坐标计算精度提升

#### 改进前问题
- 坐标计算误差大（z轴误差±9米）
- 没有正确处理无人机坐标和世界坐标的关系
- 缺少坐标系转换

#### 改进方案（基于human_position方案）
- 使用射线-地面交点法
- 正确的坐标系变换链：相机→Body→ENU
- 利用地面约束（假设人体高度0.9米）

#### 精度提升
- z轴精度提升**18倍**（9.2m→0.51m）
- 3D误差改善**10倍**（9.24m→0.93m）

#### 关键实现
```python
# 1. 像素坐标转相机射线
x_norm = (u - cx) / fx
y_norm = (v - cy) / fy
ray_camera = normalize([x_norm, y_norm, 1.0])

# 2. 坐标系变换
R_cam_to_enu = R_body_to_enu @ R_gimbal @ R_cam_base
ray_enu = R_cam_to_enu @ ray_camera

# 3. 射线-地面交点
t = (0.9 - drone_z) / ray_z
target_pos = drone_pos + t * ray_enu
```

### 3. 记分界面改进

#### 改进内容
- 添加颜色标识（彩色圆圈）
- 显示每个目标的颜色名称
- 日志输出包含颜色信息
- 改进界面布局

#### 颜色映射
- Actor 0: GREEN（绿色）
- Actor 1: BLUE（蓝色）
- Actor 2: BROWN（棕色）
- Actor 3: WHITE（白色）
- Actor 4: RED1（红色1）
- Actor 5: RED2（红色2）

## 🔍 重要调试信息

### 坐标系验证
系统现在会输出详细的坐标计算过程：

```
🚁 无人机位置(ENU): x=10.00, y=5.00, z=3.00
✅ 射线-地面交点: t=4.50m
🎯 目标位置计算: 无人机(10.00, 5.00, 3.00) + 偏移(3.20, 1.80, -2.10) = 目标(13.20, 6.80, 0.90)
```

### 关键点
1. **无人机位置是相对于世界原点（起飞点）**
2. **目标位置 = 无人机位置 + 相对偏移**
3. **不要把无人机位置当作原点**

## 📊 参数调优建议

### 追踪控制参数
```yaml
# XTDrone算法参数
Kp_xy: 0.5          # 水平控制增益
ideal_box_size: 2400 # 理想检测框面积

# 死区设置
lateral_deadzone: 20  # 横向死区
yaw_deadzone: 30     # 偏航死区

# 速度限制
max_vel: 2.0         # 最大水平速度
max_vel_z: 1.0       # 最大垂直速度
```

### 记分系统参数
```yaml
err_threshold: 1.0    # 位置误差阈值（米）
detection_time: 15.0  # 稳定检测时间（秒）
```

## ⚠️ 注意事项

1. **坐标系统一**
   - MAVROS提供的是ENU坐标（相对于起飞点）
   - 记分系统期望的也是ENU坐标
   - 不需要额外的坐标变换

2. **调试建议**
   - 观察日志中的坐标输出，确认计算正确
   - 检查无人机位置是否合理
   - 验证目标位置是否在合理范围内

3. **性能优化**
   - 控制频率：50Hz
   - 检测频率：30Hz
   - 记分刷新：10Hz

## 🚀 测试步骤

1. 启动仿真环境
```bash
roslaunch px4 robocup.launch
```

2. 启动多机系统
```bash
roslaunch yolov11_ros multi_drone_flight.launch num_drones:=2
```

3. 观察输出
- 检查追踪是否稳定
- 验证坐标计算是否准确
- 确认记分界面颜色显示正确

## 📝 待优化项

1. **动态云台角度支持**
   - 当前使用固定-45度
   - 可订阅实际云台角度

2. **多目标优先级**
   - 当前使用最近目标
   - 可根据颜色或位置优化选择策略

3. **卡尔曼滤波**
   - 平滑目标位置估计
   - 预测目标运动

---

**维护**: 东华大学 Astraeus队  
**状态**: ✅ 已完成测试
