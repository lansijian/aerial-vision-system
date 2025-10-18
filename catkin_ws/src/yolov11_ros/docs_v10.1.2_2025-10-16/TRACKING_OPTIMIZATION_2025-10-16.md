# 追踪优化方案 - 2025-10-16

## 🚀 优化目标

提升追踪精度和响应速度，所有参数ROS化方便现场调节。

---

## ✅ 实现的优化

### 1. 2D Kalman滤波器（u,v预测）

**目的**：平滑像素坐标，减小抖动，使用预测值提高响应

**实现**：
```python
class KalmanFilter2D:
    # 状态向量 [u, v, u_dot, v_dot]
    # 预测 + 更新两步
    
    def predict(self, dt):
        # 基于运动模型预测下一帧位置
        self.x = self.F @ self.x
        
    def update(self, z):
        # 用YOLO观测值更新状态
        K = self.P @ self.H.T @ inv(S)  # 卡尔曼增益
        self.x = self.x + K @ y
```

**使用方式**：
- **图像回调（30Hz）**：更新Kalman观测
- **控制循环（100Hz）**：使用Kalman预测值计算偏移

**优势**：
- ✅ 平滑u,v坐标，减小抖动
- ✅ 预测下一帧位置，提前响应
- ✅ 100Hz控制不受30Hz图像限制

---

### 2. 自适应偏航增益 + 积分控制

**旧算法**：
```python
Kp_yaw = 0.003  # 固定增益
yaw_rate = -Kp_yaw * u_offset
```

**新算法**：
```python
# 自适应增益：目标近时增益大，远时增益小
Kp_yaw_adaptive = Kp_yaw_base / box_height  # 80 / box_height

# PI控制
yaw_p = -Kp_yaw_adaptive * u_offset  # P项
yaw_i = -Ki_yaw * integral(u_offset)  # I项（消除稳态误差）
yaw_rate = yaw_p + yaw_i
```

**参数**：
- `Kp_yaw_base = 80.0` - 基数（可调）
- `Ki_yaw = 0.001` - 积分增益（可调）

**优势**：
- ✅ 目标近时快速对准（增益大）
- ✅ 目标远时平稳追踪（增益小）
- ✅ 积分项消除长期偏差

---

### 3. 前后速度：框高误差法

**旧算法（面积法）**：
```python
ideal_box_size = 1000  # 像素²
size_error = (ideal_box_size - box_size) / ideal_box_size
forward_speed = Kp_distance * size_error
```

**新算法（框高法）**：
```python
ideal_box_height = 116  # 像素（对应3米）
height_error = ideal_box_height - box_height
forward_speed = Kp_distance * (height_error / ideal_box_height)
```

**优势**：
- ✅ 框高比面积更稳定（不受宽度影响）
- ✅ 线性关系更直观
- ✅ 误差计算更准确

---

### 4. 控制饱和值Launch化

**可调参数**（launch文件）：
```xml
<param name="max_vx" value="3.0" />       <!-- 前后速度上限 -->
<param name="max_vy" value="1.0" />       <!-- 横向速度上限 -->
<param name="max_vz" value="0.5" />       <!-- 垂直速度上限 -->
<param name="max_yaw_rate" value="0.5" /> <!-- 偏航率上限 -->
```

**现场调节**：
```bash
# 比赛现场可以直接修改launch文件，无需改代码
# 例如：追踪太慢 → 提高max_vx到4.0
# 例如：转向太猛 → 降低max_yaw_rate到0.3
```

---

### 5. 100Hz控制循环

**旧架构（30Hz）**：
```python
# 图像回调中直接计算速度（30Hz）
def detection_callback(msg):
    target = select_target(msg)
    compute_velocity(target)
    publish_velocity()  # 30Hz
```

**新架构（100Hz）**：
```python
# 图像回调：只更新观测（30Hz）
def detection_callback(msg):
    kalman.update([u, v])  # 更新观测
    self.last_detection = target

# 控制循环：100Hz定时器
def control_loop(event):  # 100Hz
    kalman.predict(dt)  # 预测
    compute_velocity()  # 使用预测值
    publish_velocity()  # 100Hz发布
```

**优势**：
- ✅ 控制频率提高3倍（30Hz→100Hz）
- ✅ 响应更快，追踪更平滑
- ✅ 图像和控制解耦

---

## 📊 参数总览

### 完整ROS参数列表

| 参数名 | 默认值 | 说明 | 现场可调 |
|--------|--------|------|---------|
| `use_kalman` | true | 是否启用Kalman滤波 | ✅ |
| `control_frequency` | 100.0 | 控制循环频率（Hz） | ✅ |
| `Kp_yaw_base` | 80.0 | 自适应偏航增益基数 | ✅ 重要 |
| `Ki_yaw` | 0.001 | 偏航积分增益 | ✅ 重要 |
| `Kp_lateral` | 0.002 | 横向速度增益 | ✅ |
| `ideal_box_height` | 116 | 理想框高（像素） | ✅ 重要 |
| `Kp_distance` | 1.5 | 前后距离增益 | ✅ 重要 |
| `Kp_z` | 0.5 | 高度控制增益 | ✅ |
| `max_vx` | 3.0 | 前后速度上限（m/s） | ✅ 重要 |
| `max_vy` | 1.0 | 横向速度上限（m/s） | ✅ |
| `max_vz` | 0.5 | 垂直速度上限（m/s） | ✅ |
| `max_yaw_rate` | 0.5 | 偏航率上限（rad/s） | ✅ |

### 现场调节指南

#### 追踪太慢
```xml
<param name="max_vx" value="4.0" />  <!-- 提高到4.0 -->
<param name="Kp_distance" value="2.0" />  <!-- 提高响应 -->
```

#### 转向不够快
```xml
<param name="Kp_yaw_base" value="100.0" />  <!-- 提高到100 -->
<param name="max_yaw_rate" value="0.7" />  <!-- 提高上限 -->
```

#### 抖动严重
```xml
<param name="Kp_distance" value="1.0" />  <!-- 降低增益 -->
<param name="use_kalman" value="true" />  <!-- 确保启用Kalman -->
```

#### 目标总是偏离中心
```xml
<param name="Ki_yaw" value="0.002" />  <!-- 提高积分增益 -->
```

---

## 🔬 算法详解

### Kalman滤波流程

```
图像回调（30Hz）:
  ├─ YOLO检测 → [u, v]观测值
  ├─ kalman.predict(dt)
  └─ kalman.update([u, v])

控制循环（100Hz）:
  ├─ kalman.predict(dt) → [u_pred, v_pred]
  ├─ u_offset = u_pred - cx
  ├─ 计算速度命令
  └─ 发布（100Hz）
```

### 自适应Kp_yaw计算

```python
# 例子：
box_height = 116 → Kp_yaw = 80/116 = 0.690（目标在3米，增益适中）
box_height = 200 → Kp_yaw = 80/200 = 0.400（目标很近，增益降低）
box_height = 50  → Kp_yaw = 80/50  = 1.600（目标很远，增益提高）
```

### PI偏航控制

```python
# P项：比例控制
yaw_p = -Kp_yaw_adaptive * u_offset

# I项：积分控制（消除稳态误差）
integral += u_offset * dt
yaw_i = -Ki_yaw * integral

# 总输出
yaw_rate = yaw_p + yaw_i
```

---

## 🧪 测试验证

### 启动后观察日志

```
================================================================================
🚀 无人机0 追踪器v10.1.2-optimized开始运行
   控制频率: 100.0Hz
   Kalman滤波: 启用
   自适应Kp_yaw: 80.0 / box_height
   积分控制Ki_yaw: 0.001
   理想框高: 116px
   速度限制: vx±3.0, vy±1.0, vz±0.5, yaw±0.5
================================================================================
```

### 追踪时观察

```
[追踪优化] 偏移:(u=-50,v=10) 框高:120(理想116) Kp_yaw:0.6667 
          速度:(x=0.05,y=0.10,yaw=-0.12)

# 关键指标：
# - u偏移应该逐渐减小（目标居中）
# - Kp_yaw根据box_height自动调整
# - 速度在饱和限制内
```

### 性能对比

| 指标 | 旧算法 | 新算法 |
|------|--------|--------|
| 控制频率 | 30Hz | 100Hz ✅ |
| u,v平滑 | 无 | Kalman ✅ |
| 偏航增益 | 固定 | 自适应 ✅ |
| 稳态误差 | 存在 | PI消除 ✅ |
| 前后控制 | 面积法 | 框高法 ✅ |
| 速度上限 | 硬编码 | ROS参数 ✅ |

---

## 📝 代码改动总结

### human_tracker.py

1. **添加KalmanFilter2D类**（106行）
2. **参数全部ROS化**（12个参数）
3. **分离检测和控制**：
   - `_detection_callback`：更新Kalman观测（30Hz）
   - `_control_loop`：计算速度命令（100Hz）
4. **优化控制算法**：
   - 框高误差法（前后）
   - 自适应Kp_yaw + 积分（偏航）
   - Kalman预测值（平滑）

### multi_drone_system.launch

1. **添加追踪优化参数**（两架无人机各12个参数）
2. **分类清晰**：
   - Kalman滤波参数
   - 自适应偏航控制参数
   - 前后距离控制参数
   - 速度饱和限制参数

---

## 🎯 使用建议

### 默认参数（已优化）

当前配置适合：
- 3米追踪距离
- 中等速度追踪
- 平稳控制

### 激进追踪（快速响应）

```xml
<param name="max_vx" value="4.0" />
<param name="Kp_distance" value="2.0" />
<param name="Kp_yaw_base" value="100.0" />
<param name="max_yaw_rate" value="0.7" />
```

### 保守追踪（稳定第一）

```xml
<param name="max_vx" value="2.0" />
<param name="Kp_distance" value="1.0" />
<param name="Kp_yaw_base" value="60.0" />
<param name="Ki_yaw" value="0.0005" />
```

---

## 🔍 调试技巧

### 观察Kp_yaw自适应

```
# 目标近（box_height大）
框高:200 Kp_yaw:0.4000  # 增益小，平稳

# 目标中等
框高:116 Kp_yaw:0.6897  # 增益适中

# 目标远（box_height小）
框高:50 Kp_yaw:1.6000  # 增益大，快速对准
```

### 观察积分项作用

```
# 无积分时：目标可能持续偏离中心
偏移:u=-20 → u=-18 → u=-20 → ...（震荡）

# 有积分时：逐渐消除偏差
偏移:u=-20 → u=-15 → u=-10 → u=-5 → u=0 ✅
```

### 调节参数实时生效

```bash
# 修改launch文件后重启节点
rosnode kill /drone_0/human_tracker
# 节点会自动respawn，新参数生效

# 或者使用rosparam动态设置（需要节点支持）
rosparam set /drone_0/human_tracker/max_vx 4.0
```

---

## ⚠️ 注意事项

### 1. Kalman滤波调优

如果跟踪有延迟：
- 降低过程噪声`Q`
- 增加观测噪声`R`

如果抖动严重：
- 增加过程噪声`Q`
- 降低观测噪声`R`

### 2. 积分饱和

积分项有限幅保护：
```python
integral_limit = 100.0  # 防止积分饱和
```

如果发现转向过猛，可能是积分项累积过大。

### 3. 速度饱和

所有速度都有上限保护：
- `max_vx = 3.0` - 前后速度（现在提高了）
- `max_vy = 1.0` - 横向速度
- `max_vz = 0.5` - 垂直速度
- `max_yaw_rate = 0.5` - 偏航率

---

## 📈 预期效果

### 追踪精度
- 横向居中误差：<20像素（原<50像素）
- 距离误差：<0.3米（原<0.5米）
- 响应延迟：<30ms（原<100ms）

### 稳定性
- ✅ Kalman平滑，减小抖动
- ✅ PI控制，消除稳态误差
- ✅ 100Hz高频控制

### 鲁棒性
- ✅ 自适应增益，适应不同距离
- ✅ 参数ROS化，现场可调
- ✅ 积分限幅，防止饱和

---

**优化版本**：v10.1.2-optimized  
**实施日期**：2025-10-16  
**状态**：✅ 已实现，待测试验证  
**东华大学 Astraeus队**

