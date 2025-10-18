# 坐标计算算法 - 简化版

## 问题诊断

### 旧算法问题（射线-地面交点法）
1. **复杂度高**：需要计算相机射线、旋转矩阵、射线-地面交点
2. **误差累积**：多个旋转变换导致误差累积
3. **参数敏感**：对云台角度、相机参数极度敏感
4. **畸变问题**：目标不在图像中心时，摄像头畸变导致误差放大
5. **实际误差**：4.73米（远超1米要求）

### 新算法优势（简化投影法）
1. **简单可靠**：只用检测框大小和偏航角
2. **误差小**：避免复杂的旋转矩阵
3. **鲁棒性强**：对参数误差不敏感
4. **易于调试**：每个步骤都可以验证

---

## 新算法详解

### 核心思路
```
检测框大小 → 估算距离
无人机偏航角 + 像素横向偏移 → 目标方位角
距离 + 方位角 → 目标世界坐标（简单投影）
```

### 算法步骤

#### 步骤1：计算目标在图像中的位置
```python
u = (xmax + xmin) / 2.0  # 横向中心
v = (ymax + ymin) / 2.0  # 纵向中心
```

#### 步骤2：基于检测框大小估算距离
```python
# 核心公式（小孔成像）
distance = (真实高度 * 焦距) / 像素高度

# 实现
box_height = ymax - ymin  # 像素高度
estimated_distance = (1.7 * 205.47) / box_height

# 考虑云台俯角（-45度）
# 相机看到的是斜向距离，需要转换为水平距离
horizontal_distance = estimated_distance * cos(45°) ≈ estimated_distance * 0.707
```

**优势**：
- 不依赖复杂的射线计算
- 直接基于检测框，更可靠
- 云台角度修正简单

#### 步骤3：计算目标方位角
```python
# 横向偏移（像素）
u_offset = u - cx  # cx = 320

# 方位角修正
# 假设水平视场角FOV = 60度
# 像素偏移 → 角度偏移
lateral_angle_offset = (u_offset / 640) * (60° * π/180)

# 例如：
# u_offset = 320（目标在最右侧） → lateral_angle_offset ≈ +30°
# u_offset = -320（目标在最左侧） → lateral_angle_offset ≈ -30°
# u_offset = 0（目标居中） → lateral_angle_offset = 0°
```

#### 步骤4：获取无人机偏航角
```python
# 从四元数获取偏航角
q = current_pose.orientation
_, _, drone_yaw = tf_trans.euler_from_quaternion([q.x, q.y, q.z, q.w])

# drone_yaw：无人机机头指向（ENU坐标系）
# 0° = 东, 90° = 北, 180° = 西, -90° = 南
```

#### 步骤5：计算目标世界坐标
```python
# 目标相对北的角度
target_bearing = drone_yaw + lateral_angle_offset

# 简单投影（ENU坐标系）
actor_x = drone_x + horizontal_distance * cos(target_bearing)
actor_y = drone_y + horizontal_distance * sin(target_bearing)

# 加上spawn偏移
actor_x += spawn_offset_x
actor_y += spawn_offset_y
```

#### 步骤6：5帧移动平均平滑
```python
# 保留历史
coord_history_x.append(actor_x)
coord_history_y.append(actor_y)

# 移动平均
actor_x_smoothed = sum(coord_history_x) / len(coord_history_x)
actor_y_smoothed = sum(coord_history_y) / len(coord_history_y)
```

---

## 算法对比

### 旧算法（射线-地面交点法）
```python
像素(u,v) → 相机射线 → Body坐标系 → ENU坐标系 → 射线-地面交点 → 世界坐标
```

**问题**：
- 5个坐标变换环节
- 3个旋转矩阵计算
- 对参数极度敏感
- 误差累积严重

### 新算法（简化投影法）
```python
检测框高度 → 估算距离
无人机偏航角 + 像素偏移 → 方位角
距离 + 方位角 → 世界坐标
```

**优势**：
- 只有2个步骤
- 不需要复杂旋转矩阵
- 参数鲁棒性强
- 误差小且稳定

---

## 参数说明

| 参数 | 值 | 说明 |
|------|---|------|
| assumed_human_height | 1.7m | 人体高度 |
| fy | 205.47 | 相机焦距 |
| horizontal_fov_deg | 60° | 水平视场角 |
| gimbal_pitch | -45° | 云台俯角 |
| cos(45°) | 0.707 | 俯角修正系数 |

---

## 精度预期

### 距离估算精度
- 3米距离：box_height ≈ 116像素 → 误差约±0.3米
- 5米距离：box_height ≈ 70像素 → 误差约±0.5米
- 10米距离：box_height ≈ 35像素 → 误差约±1.0米

### 方位角精度
- 目标居中（u_offset=0）：角度误差0°
- 目标偏移100像素：角度误差约±10°
- 目标偏移200像素：角度误差约±20°

### 总体精度（理论）
- **追踪模式**（目标居中）：误差约±0.5米 ✅
- **巡检发现目标**（目标可能偏离）：误差约±1.5米 ⚠️

**改进建议**：
- 追踪时保持目标居中（已实现）
- 发布坐标前等待目标居中稳定
- 使用5帧平滑减小噪声

---

## 调试日志格式

```
[坐标-white] 原始:(10.50,5.30) 平滑:(10.48,5.28) | 
             无人机:(8.20,3.10) 偏航:45° | 
             距离:3.2m 方位:60°
```

**关键信息**：
- `原始` - 当前帧计算的坐标
- `平滑` - 5帧移动平均后的坐标
- `无人机` - 当前位置
- `偏航` - 机头指向（度）
- `距离` - 估算的水平距离（米）
- `方位` - 目标相对北的角度（度）

---

## 验证方法

### 1. 对比Gazebo真实位置
```bash
# 查看actor真实位置
rostopic echo /gazebo/model_states

# 查看发布的坐标
rostopic echo /actor_white_info

# 计算误差
error = sqrt((x_pub - x_true)^2 + (y_pub - y_true)^2)
```

### 2. 观察平滑效果
```
# 原始坐标应该有波动
原始:(10.50,5.30)
原始:(10.48,5.28)
原始:(10.52,5.32)

# 平滑坐标应该稳定
平滑:(10.50,5.30)
平滑:(10.50,5.30)
平滑:(10.50,5.30)
```

### 3. 检查距离估算
```
# box_height大（近距离）→ 距离小
box_height:150 → 距离:2.3m

# box_height小（远距离）→ 距离大
box_height:50 → 距离:7.0m
```

---

**算法版本**：simplified-v1  
**更新日期**：2025-10-16  
**状态**：✅ 已实现，待测试验证  
**东华大学 Astraeus队**

