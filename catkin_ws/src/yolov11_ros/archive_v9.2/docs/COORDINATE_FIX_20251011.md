# 坐标系统修复说明

**日期**: 2025-10-11
**版本**: v8.2

## 🔍 问题诊断

### 问题描述
无人机发布的目标坐标与Gazebo中actor的实际位置相差很大，导致记分系统无法正确判定目标。

### 根本原因
1. **坐标系混淆**：
   - MAVROS提供的`/mavros/local_position/pose`是**ENU坐标**（东-北-天），原点在**无人机起飞位置**
   - Gazebo中的actors位置是**世界坐标系**，原点在世界中心(0,0,0)
   - 无人机起飞位置不在世界原点：
     - typhoon_h480_0: (0, -5, 1)
     - typhoon_h480_1: (0, 5, 1)

2. **Actor实际位置**（世界坐标）：
   - actor_0 (GREEN): (-39.55, -40.8, 0.0)
   - actor_1 (BLUE): (-20.2, -40.8, 0.0)
   - actor_2 (BROWN): (-41.3, -5.2, 0.0)
   - actor_3 (WHITE): (5.0, -30.0, 0.0)
   - actor_4 (RED1): (25.0, -15.0, 0.0)
   - actor_5 (RED2): (40.0, 15.0, 0.0)

3. **记分系统期望**：
   - `robocup_score_cal.py`从Gazebo获取actor的世界坐标
   - 比较接收到的坐标与actor世界坐标
   - 误差阈值1米内认为匹配成功

## ✅ 解决方案

### 1. 添加世界坐标偏移
```python
# 获取无人机在世界坐标系中的起飞位置偏移
if 'typhoon_h480_0' in vehicle_ns or self.drone_id == 0:
    self.world_offset_x = 0.0  
    self.world_offset_y = -5.0  # typhoon_h480_0在(0, -5)起飞
elif 'typhoon_h480_1' in vehicle_ns or self.drone_id == 1:
    self.world_offset_x = 0.0
    self.world_offset_y = 5.0   # typhoon_h480_1在(0, 5)起飞
```

### 2. 坐标转换流程
```python
# 步骤1: 计算ENU坐标（相对于起飞点）
target_x_enu = drone_position.x + offset_x
target_y_enu = drone_position.y + offset_y

# 步骤2: 转换到世界坐标系
target_x = target_x_enu + world_offset_x
target_y = target_y_enu + world_offset_y
```

### 3. 相机坐标系修正
原代码假设FLU（前-左-上）坐标系，但PX4实际使用FRD（前-右-下）：
```python
# 修正前（错误）
R_cam_base = np.array([
    [0, 0, 1],   # Body X = 相机Z (前)
    [-1, 0, 0],  # Body Y = -相机X (左 = -右)
    [0, -1, 0]   # Body Z = -相机Y (上 = -下)
])

# 修正后（正确）
R_cam_base = np.array([
    [0, 0, 1],   # Body X = 相机Z (前)
    [1, 0, 0],   # Body Y = 相机X (右)
    [0, 1, 0]    # Body Z = 相机Y (下)
])
```

## 📊 调试输出

增加详细的坐标计算调试输出：
```
🎯 目标位置计算:
   无人机ENU: (10.5, 3.2, 3.0)    # 相对于起飞点
   相对偏移: (5.2, -2.1, -2.1)     # 检测到的相对位置
   目标ENU: (15.7, 1.1, 0.9)       # ENU坐标系中的目标
   世界偏移: (0.0, -5.0)           # 起飞点在世界坐标系的偏移
   目标世界: (15.7, -3.9, 0.9)     # 最终的世界坐标
```

## 🔧 修改的文件

1. `human_tracker.py`:
   - 添加世界坐标偏移参数
   - 修正相机到Body坐标系转换矩阵
   - 在所有坐标计算中加入世界偏移
   - 增加详细调试输出

2. `robocup_score_cal.py`:
   - 添加颜色显示（已完成）
   - 保持原有坐标比较逻辑

## ⚠️ 注意事项

1. **多机协同**：每架无人机有不同的起飞位置，需要正确配置world_offset
2. **坐标系验证**：通过调试输出验证计算的世界坐标是否正确
3. **误差阈值**：保持1米的误差阈值，这是合理的检测精度

## 🎯 预期效果

- 无人机发布的目标坐标将准确对应Gazebo世界坐标
- 记分系统能正确识别和消除目标
- 坐标误差应在1米以内

## 测试命令
```bash
# 启动仿真
roslaunch px4 robocup.launch

# 启动追踪系统
roslaunch yolov11_ros multi_drone_flight.launch

# 启动记分系统
rosrun yolov11_ros robocup_score_cal.py
```
