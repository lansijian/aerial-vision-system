# 简化系统 v11.1 使用指南

## 快速开始

### 1. 启动仿真环境
```bash
# 终端1
roslaunch px4 robocup.launch
```

### 2. 启动简化双机系统
```bash
# 终端2（等待10秒，确保仿真完全启动）
roslaunch yolov11_ros simple_dual_drone_system.launch
```

## 系统组成

### 简化架构（3个模块）
1. **simple_drone_controller.py** - 控制器（起飞、模式切换、速度转发）
2. **simple_waypoint_navigator.py** - 航点导航
3. **simple_human_tracker.py** - 人体追踪

### 与v10.1.2对比

| 模块 | v10.1.2 | v11.1简化版 | 说明 |
|------|---------|------------|------|
| 控制器 | drone_controller.py | simple_drone_controller.py | ✅ 简化 |
| 航点 | waypoint_navigator.py | simple_waypoint_navigator.py | ✅ 简化 |
| 追踪 | human_tracker.py | simple_human_tracker.py | ✅ 简化 |
| 多机管理 | 无 | 无 | 双机不需要 |
| 避障 | 无 | 无 | 后续添加 |

## 预期行为

### 启动阶段
```
1. YOLO检测器加载
2. 记分系统启动
3. 无人机控制器初始化
4. 航点导航器加载航点
5. 人体追踪器初始化
```

### 飞行阶段
```
1. 等待MAVROS连接
2. 发送setpoint建立连接
3. 切换OFFBOARD模式
4. 解锁无人机
5. 起飞到3米
6. 切换到航点模式
7. 开始巡检
```

### 任务执行
```
航点模式：
- 航点导航器计算速度
- 控制器转发到MAVROS
- 飞向各航点

检测到目标：
- 追踪器请求切换模式
- 控制器切换到TRACKING
- 追踪器计算速度
- 控制器转发到MAVROS

目标丢失：
- 追踪器请求返回航点
- 控制器切换到WAYPOINT
- 继续巡检
```

## 调试

### 查看日志
```bash
# 查看所有输出
roslaunch yolov11_ros simple_dual_drone_system.launch

# 应该看到的关键信息：
[简化控制器] ✅ 初始化完成 - 无人机0
[简化航点导航器] ✅ 初始化完成 - 无人机0
   加载20个航点
[简化人体追踪器] ✅ 初始化完成 - 无人机0
✅ 无人机0 MAVROS已连接
✅ 无人机0已切换到OFFBOARD模式
✅ 无人机0解锁成功
✅ 无人机0: 起飞完成，开始航点巡检
```

### 检查速度命令
```bash
# 航点速度
rostopic echo /drone_0/waypoint/cmd_vel

# 追踪速度
rostopic echo /drone_0/tracking/cmd_vel

# 最终MAVROS速度
rostopic echo /typhoon_h480_0/mavros/setpoint_velocity/cmd_vel
```

### 检查飞行模式
```bash
rostopic echo /drone_0/flight_mode

# 应该看到：
# "WAYPOINT" 或 "TRACKING"
```

## 如果出现问题

### 问题1：无人机不起飞
**检查**：
```bash
rostopic echo /typhoon_h480_0/mavros/state
# 确认：connected: True, mode: "OFFBOARD", armed: True
```

### 问题2：无人机倒着飞
**分析**：
1. 查看第一个航点方向是否合理
2. 检查偏航角计算
3. 查看日志中的error_x, error_y

### 问题3：追踪方向反
**分析**：
1. 查看调试日志中的像素偏移
2. 检查v_offset和linear.x的对应关系
3. 如果v_offset<0（目标在上方）但linear.x<0（向后），则需要反转

### 问题4：无法加载航点
**检查**：
```bash
# 确认文件存在
ls ~/catkin_ws/src/yolov11_ros/scripts/waypoints/

# 应该看到：
# waypoints_drone_0.json
# waypoints_drone_1.json
```

## 参数调整

### 巡航速度
```xml
<param name="cruise_speed" value="5.0" />  <!-- 建议3.0-7.0 -->
```

### 追踪速度
在simple_human_tracker.py中：
```python
self.Kp_xy = 0.5      # 建议0.4-0.6
self.Kp_yaw = 0.002   # 建议0.001-0.005
```

## 恢复到v11.0

如果简化版本有问题，想恢复到v11.0：
```bash
# 使用原来的launch文件
roslaunch yolov11_ros multi_drone_system.launch
```

## 后续计划

v11.1稳定后可以逐步添加：
1. 多机目标分配
2. 冲量式避障
3. 更多安全检查

---
**作者**：东华大学 Astraeus队  
**版本**：v11.1  
**状态**：待测试

