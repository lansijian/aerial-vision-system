# 追踪控制修复记录

**日期**：2025-01-15  
**版本**：v11.0  
**问题**：无人机追踪控制效果不佳

## 问题分析

### v11.0原始实现（有问题）
使用**基于世界坐标的追踪控制**：
1. 通过射线-地面交点法计算目标世界坐标
2. 计算无人机与目标的位置误差
3. 基于位置误差计算速度指令

**存在的问题**：
- 世界坐标计算依赖复杂的坐标变换
- 坐标误差会累积，影响追踪精度
- 对参数标定要求高
- 追踪响应不够灵敏

### v10.1.2经过验证的实现（工作正常）
使用**基于图像的视觉伺服控制（IBVS - Image-Based Visual Servoing）**：
1. 直接使用目标在图像中的像素坐标
2. 计算像素偏移（相对于图像中心）
3. 通过视觉伺服算法转换为机体速度指令

**优势**：
- 不需要精确的世界坐标
- 直接基于视觉反馈控制
- 响应快速，鲁棒性好
- 对标定误差不敏感

## 修复内容

### 1. 修改追踪控制算法
**文件**：`target_tracker.py`  
**函数**：`_compute_tracking_velocity()`（第383-423行）

```python
# 修改前：基于世界坐标
def _compute_tracking_velocity(self, target_x, target_y):
    # 当前位置
    drone_x = self.current_pose.position.x + self.spawn_offset_x
    drone_y = self.current_pose.position.y + self.spawn_offset_y
    
    # 位置误差
    error_x = target_x - drone_x
    error_y = target_y - drone_y
    
    # 基于位置误差计算速度
    cmd_vel.linear.x = self.kp_xy * error_x
    cmd_vel.linear.y = self.kp_xy * error_y
    ...

# 修改后：基于图像的视觉伺服（参考v10.1.2）
def _compute_tracking_velocity(self, target):
    # 计算目标中心像素坐标
    u = (target.xmax + target.xmin) / 2.0
    v = (target.ymax + target.ymin) / 2.0
    
    # 计算像素偏移（相对于图像中心）
    u_offset = u - self.cx
    v_offset = v - self.cy
    
    # 基于高度和云台角度估算距离
    height = self.current_pose.position.z
    theta = math.radians(abs(self.gimbal_pitch))
    z = height / max(math.sin(theta), 0.1)
    
    # 计算速度（图像伺服控制算法）
    u_velocity = -self.kp_xy * u_offset
    v_velocity = -self.kp_xy * v_offset
    
    # 转换到机体坐标系
    cmd_vel.linear.x = v_velocity * z / max(v_offset * math.cos(math.radians(45)) + self.fy * math.sin(math.radians(45)), 1.0)
    cmd_vel.linear.y = (z * u_velocity - u_offset * math.cos(math.radians(45)) * cmd_vel.linear.x) / max(self.fx, 1.0)
    cmd_vel.linear.z = self.kp_z * (self.ideal_tracking_distance - height)
    cmd_vel.angular.z = -self.kp_yaw * u_offset
```

### 2. 修改函数调用
**文件**：`target_tracker.py`  
**位置**：第475-492行

```python
# 修改前：传入世界坐标
cmd_vel = self._compute_tracking_velocity(smooth_x, smooth_y)

# 修改后：传入目标对象
cmd_vel = self._compute_tracking_velocity(target)
```

### 3. 添加速度限制
**文件**：`target_tracker.py`  
**位置**：第413-421行

```python
# 速度限制
horizontal_speed = math.sqrt(cmd_vel.linear.x**2 + cmd_vel.linear.y**2)
if horizontal_speed > self.max_tracking_speed:
    scale = self.max_tracking_speed / horizontal_speed
    cmd_vel.linear.x *= scale
    cmd_vel.linear.y *= scale

cmd_vel.linear.z = np.clip(cmd_vel.linear.z, -1.0, 1.0)
cmd_vel.angular.z = np.clip(cmd_vel.angular.z, -0.5, 0.5)
```

### 4. 简化控制流程
**文件**：`target_tracker.py`  
**位置**：第475-492行

```python
# 修改后的流程
if self.is_tracking_approved and self.tracking_color == target.Class:
    # 1. 计算世界坐标（仅用于发布ActorInfo）
    actor_x, actor_y = self._compute_world_position(target)
    if actor_x is not None and actor_y is not None:
        self._publish_actor_info(actor_x, actor_y, target.Class)
    
    # 2. 计算追踪速度（基于图像的视觉伺服控制）
    cmd_vel = self._compute_tracking_velocity(target)
    
    # 3. 输出调试信息
    rospy.loginfo_throttle(1.0, ...)
```

## 控制算法原理

### 图像伺服控制（IBVS）
基于图像的视觉伺服控制直接使用视觉特征（像素坐标）来控制机器人运动：

1. **特征提取**：提取目标在图像中的位置（u, v）
2. **误差计算**：计算当前特征与期望特征的偏差
3. **控制律**：通过图像雅可比矩阵将特征误差映射到速度指令
4. **执行**：发送速度指令到无人机

### 坐标转换关系
```
像素偏移 → 速度指令：
- u_offset（水平偏移）→ linear.y（左右速度）+ angular.z（偏航）
- v_offset（垂直偏移）→ linear.x（前后速度）
- height（高度）→ linear.z（上下速度）
```

### 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| kp_xy | 0.8 | 水平位置增益 |
| kp_z | 1.0 | 高度增益 |
| kp_yaw | 1.2 | 偏航增益 |
| ideal_tracking_distance | 3.0 | 理想追踪距离（高度）|
| max_tracking_speed | 2.0 | 最大追踪速度 |

## 预期效果

修复后的追踪控制应该：
- ✅ 响应更快速
- ✅ 控制更平滑
- ✅ 对标定误差不敏感
- ✅ 能够稳定追踪目标
- ✅ 保持合适的追踪距离

## 测试建议

1. **观察追踪行为**：
   - 无人机应该能快速对准目标
   - 目标应该保持在图像中心附近
   - 追踪距离应该稳定在3米左右

2. **检查速度指令**：
   ```bash
   rostopic echo /drone_0/target_tracker/cmd_vel
   ```

3. **观察日志输出**：
   - 应该看到速度指令不为零
   - 像素偏移应该逐渐减小

## 技术参考

- **IBVS原理**：E. Malis, F. Chaumette, and S. Boudet. "2 1/2 D visual servoing." IEEE Transactions on Robotics and Automation, 1999.
- **v10.1.2实现**：`archive_v10.1.2/human_tracker_v10.1.2.py`

## 相关文档

- [FIX_VELOCITY_ZERO_2025-01-15.md](./FIX_VELOCITY_ZERO_2025-01-15.md) - 速度为零问题修复
- [FIX_ARMING_ISSUE_2025-01-15.md](./FIX_ARMING_ISSUE_2025-01-15.md) - 解锁失败问题修复

---
**记录人**：东华大学 Astraeus队  
**审核状态**：已完成

