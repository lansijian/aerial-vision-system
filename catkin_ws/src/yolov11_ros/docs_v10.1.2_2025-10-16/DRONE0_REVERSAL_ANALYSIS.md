# 无人机0追踪反向问题分析

## 新发现（2025-10-16）

### 现象
- 无人机0在执行航点飞行时转向180°
- 转向后识别到行人
- 切换到追踪模式时方向反向

### 可能原因分析

#### 假设1：初始偏航角差异（最可能）
```
无人机0初始spawn时：
- Gazebo spawn: Y=0° (朝东)
- 但实际PX4认为：Y=180° (朝西)

无人机1初始spawn时：
- Gazebo spawn: Y=0° (朝东)  
- PX4也认为：Y=0° (朝东)

结论：两架无人机的初始坐标系定义相差180°
```

**验证方法**：
```bash
# 启动后立即查看两架无人机的偏航角
rostopic echo /typhoon_h480_0/mavros/local_position/pose | grep orientation
rostopic echo /typhoon_h480_1/mavros/local_position/pose | grep orientation

# 对比四元数，看是否相差180度
```

#### 假设2：转向后坐标系切换问题
```
无人机执行航点飞行：
1. 从(0,-3)飞向(-15,0) - 需要转向
2. 转向180°后机头朝向改变
3. 此时切换到追踪模式
4. 追踪使用速度控制（body frame）
5. 如果body frame定义不同，速度命令会反向
```

**问题**：
- 位置控制（航点）：使用绝对坐标，不受影响 ✅
- 速度控制（追踪）：使用机体坐标系，受偏航角影响 ❌

---

## 解决方案对比

### 方案A：参数化速度修正（已实现但被删除）

**优点**：
- 简单直接
- 参数化配置，不硬编码
- 支持6机扩展

**缺点**：
- 治标不治本
- 每架无人机都要单独配置

**实现**：
```python
# drone_controller.py
self.velocity_invert_x = rospy.get_param('~velocity_invert_x', False)
cmd.linear.x = -msg.linear.x if self.velocity_invert_x else msg.linear.x
```

```xml
<!-- multi_drone_system.launch -->
<param name="velocity_invert_x" value="true" />  <!-- 无人机0 -->
<param name="velocity_invert_x" value="false" /> <!-- 无人机1 -->
```

---

### 方案B：统一初始偏航角（根本解决）

**思路**：
- 找到为什么无人机0和1的初始偏航角不同
- 修改PX4配置或Gazebo spawn参数，让它们一致

**需要检查**：
1. spawn launch文件中的Y参数
2. PX4参数配置
3. MAVROS坐标系设置

**验证**：
```bash
# 查看spawn配置
cat launch(1)/launch/spawn_single_typhoon_0.launch  # Y值
cat launch(1)/launch/spawn_single_typhoon_1.launch  # Y值

# 都是0度，说明spawn配置相同
```

---

### 方案C：追踪使用ENU坐标系（待验证）

**思路**：
- 将追踪速度从机体坐标系转换为ENU坐标系
- 使用绝对方位控制，不依赖机体坐标系

**实现**（未测试）：
```python
# human_tracker.py
# 计算ENU坐标系下的速度
yaw = get_drone_yaw()
enu_vx = body_vx * cos(yaw) - body_vy * sin(yaw)
enu_vy = body_vx * sin(yaw) + body_vy * cos(yaw)

# drone_controller转换回body坐标系
body_vx = enu_vx * cos(yaw) + enu_vy * sin(yaw)
body_vy = -enu_vx * sin(yaw) + enu_vy * cos(yaw)
```

**风险**：
- 需要大量测试
- 可能引入新的问题
- 不建议比赛前尝试

---

## 当前状态

### 用户选择
- 已删除launch文件中的velocity_invert参数配置
- 可能想尝试其他解决方案

### 建议

#### 如果问题不严重
- 保持当前状态
- 无人机0避免转向180°的航点
- 或者手动调整航点顺序

#### 如果需要彻底解决
1. **先验证假设**：检查初始偏航角是否真的相差180°
   ```bash
   # 运行诊断脚本
   rosrun yolov11_ros diagnostic_script.py
   ```

2. **如果确认是初始偏航角问题**：
   - 方案A：恢复velocity_invert参数（最简单）
   - 方案B：修改spawn配置（需要找到根本原因）

3. **如果不是初始偏航角问题**：
   - 需要更详细的调试信息
   - 可能是其他原因

---

## 调试建议

### 添加调试日志
```python
# drone_controller.py - _tracker_cmd_callback_twist
rospy.loginfo_throttle(2.0,
    f"[追踪命令] 无人机{self.drone_id} "
    f"收到: vx={msg.linear.x:.2f} vy={msg.linear.y:.2f} "
    f"偏航角:{math.degrees(drone_yaw):.0f}°")
```

### 运行时观察
```
# 正常情况（无人机1）
偏航角:0° → 收到vx=0.5 → 向前飞 ✅
偏航角:90° → 收到vx=0.5 → 向前飞 ✅

# 异常情况（无人机0，如果存在）
偏航角:180° → 收到vx=0.5 → 向后飞？❌
```

---

## ⚠️ 重要提醒

**不确定的情况下不要修改代码！**

建议：
1. 先用诊断脚本验证假设
2. 收集更多调试信息
3. 确认问题原因后再修复

如果问题只在特定场景出现（转向180°后），可以：
- 调整航点顺序，避免大角度转向
- 或者接受这个小问题，专注于其他优化

---

**分析日期**：2025-10-16  
**状态**：待用户验证  
**东华大学 Astraeus队**

