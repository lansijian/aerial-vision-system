# XTDrone官方追踪算法 - v11.1最终版

**日期**：2025-10-16  
**核心**：完全参考XTDrone官方yolo_human_tracking.py

## 核心算法（XTDrone官方）

```python
# 1. 像素偏移
u_ = u - 320  # 水平偏移
v_ = v - 180  # 垂直偏移

# 2. 像素速度
u_velocity = -0.5 * u_
v_velocity = -0.5 * v_

# 3. 深度估算
z = height / sin(theta)  # theta=云台角度

# 4. 机体速度（XTDrone几何变换）
x_velocity = v_velocity * z / (v_*cos(theta) + fy*sin(theta))
y_velocity = (z*u_velocity - u_*cos(theta)*x_velocity) / fx

# 5. 高度保持
cmd_vel.z = 1.0 * (3.0 - height)
```

## 关键参数

| 参数 | 值 | 来源 |
|------|-----|------|
| kp_xy | 0.5 | XTDrone官方 |
| kp_z | 1.0 | XTDrone官方 |
| fx, fy | 205.47 | 相机参数 |
| u_center | 320 | 640/2 |
| v_center | 180 | 360/2 |
| theta | -45° | 云台俯仰角 |
| target_height | 3.0m | 固定高度 |

## v11.1优化

### 1. 速度平滑
```python
# 30%新值 + 70%旧值
cmd_vel = 0.3 * new_vel + 0.7 * last_vel
```

### 2. 目标丢失超时
```python
# 1.5秒后返回航点
if lost_time > 1.5:
    request_waypoint_mode()
```

### 3. 简化逻辑
- 不使用angular.z（靠XY自然对准）
- 不等待多机批准（直接追踪）
- 不使用复杂的坐标变换

## 控制效果

### 理想状态
- 目标在图像中心 (320, 180)
- 偏移 = (0, 0)
- 速度 = (0, 0, 0)
- 无人机稳定跟随

### 目标偏右
- 目标在 (400, 180)
- 偏移 = (80, 0)
- u_velocity = -0.5 * 80 = -40
- y_velocity计算后为负
- linear.y < 0 → 向右飞 ✅

### 目标偏下
- 目标在 (320, 250)
- 偏移 = (0, 70)
- v_velocity = -0.5 * 70 = -35
- x_velocity计算后为正
- linear.x > 0 → 向前飞 ✅

## 完整代码

```python
def _compute_tracking_velocity(self, target):
    # 目标像素
    u = (target.xmax + target.xmin) / 2.0
    v = (target.ymax + target.ymin) / 2.0
    
    # 偏移
    u_ = u - 320
    v_ = v - 180
    
    # 深度
    height = self.current_pose.position.z
    theta = math.radians(45)  # 云台角
    z = height / math.sin(theta)
    
    # 像素速度
    u_velocity = -0.5 * u_
    v_velocity = -0.5 * v_
    
    # 机体速度
    denom = v_ * math.cos(theta) + 205.47 * math.sin(theta)
    x_velocity = v_velocity * z / denom
    y_velocity = (z * u_velocity - u_ * math.cos(theta) * x_velocity) / 205.47
    
    # 赋值
    cmd_vel.linear.x = x_velocity
    cmd_vel.linear.y = y_velocity
    cmd_vel.linear.z = 1.0 * (3.0 - height)
    
    # 平滑
    cmd_vel = 0.3 * cmd_vel + 0.7 * last_vel
    
    return cmd_vel
```

## 预期效果

测试时应该看到：
```
[追踪0] 像素:(320,180) 偏移:(0,0) 高度:3.0m 速度:X=0.00 Y=0.00 Z=0.00
[追踪0] 像素:(340,180) 偏移:(20,0) 高度:3.0m 速度:X=0.00 Y=-0.XX Z=0.00
[追踪0] 像素:(320,200) 偏移:(0,20) 高度:3.0m 速度:X=0.XX Y=0.00 Z=0.00
```

- ✅ 偏移逐渐减小到0
- ✅ 目标保持在图像中心
- ✅ 高度稳定在3米
- ✅ 速度平滑变化

---
**版本**：v11.1 Final  
**日期**：2025-10-16  
**来源**：XTDrone官方yolo_human_tracking.py

