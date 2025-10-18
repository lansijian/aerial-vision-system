# 冲量式避障优化记录

**日期**：2025-01-15  
**版本**：v11.0  
**问题**：避障模块抢占控制权，导致无人机停住不动

## 问题描述

### 原实现的问题
1. **持续性避障**：检测到障碍物后，持续发送避障速度
2. **抢占控制权**：避障速度优先级最高，会覆盖主任务速度
3. **停止不动**：当检测到障碍物时，无人机会停住不动

### 用户需求
> "如果无人机在向前飞的时候右前方检测到障碍了，就给一个瞬间的向左的加速度，在原来速度的基础上，然后过很短的时间取消这个加速度，用这样简单的方式避开障碍物。"

核心需求：
- **冲量式避障**：短暂的加速度脉冲
- **叠加不覆盖**：在原速度基础上叠加
- **自动取消**：很短时间后自动归零

## 解决方案

### 1. 冲量触发机制（obstacle_avoidance.py）

#### 边沿检测触发
```python
# 只在首次检测到障碍物时触发（上升沿）
current_obstacle_detected = self.nearest_obstacle_distance < self.avoidance_range

if current_obstacle_detected and not self.last_obstacle_detected:
    # 触发新的避障冲量
    self._trigger_avoidance_impulse()

self.last_obstacle_detected = current_obstacle_detected
```

#### 智能避让方向
```python
def _trigger_avoidance_impulse(self):
    # 根据障碍物位置选择避让方向
    if obstacle_angle > 0:
        # 障碍物在右侧，向左避让（逆时针旋转90度）
        avoid_x = -obstacle_y
        avoid_y = obstacle_x
    else:
        # 障碍物在左侧，向右避让（顺时针旋转90度）
        avoid_x = obstacle_y
        avoid_y = -obstacle_x
    
    # 计算冲量强度（距离越近，冲量越强）
    distance_factor = 1.0 - (self.nearest_obstacle_distance / self.avoidance_range)
    impulse_magnitude = self.impulse_strength * distance_factor
```

#### 自动衰减机制
```python
if self.impulse_active:
    elapsed = (rospy.Time.now() - self.impulse_start_time).to_sec()
    
    if elapsed >= self.impulse_duration:
        # 冲量结束，归零
        self.impulse_active = False
        self.repulsion_vector = np.array([0.0, 0.0])
    else:
        # 线性衰减，使过渡更平滑
        decay_factor = 1.0 - (elapsed / self.impulse_duration)
        self.repulsion_vector *= decay_factor
```

### 2. 速度叠加机制（mission_controller.py）

#### 修改前（覆盖式）
```python
# 紧急避障：临时覆盖主任务速度
avoidance_factor = min(avoidance_mag / 1.0, 1.0)

# 混合主任务和避障速度
fused.linear.x = fused.linear.x * (1 - avoidance_factor) + self.avoidance_vel.linear.x * avoidance_factor
fused.linear.y = fused.linear.y * (1 - avoidance_factor) + self.avoidance_vel.linear.y * avoidance_factor
```

**问题**：避障速度会根据强度覆盖主任务速度，导致无人机减速或停止

#### 修改后（叠加式）
```python
# 避障冲量叠加（不覆盖主任务，只是额外加速度）
if avoidance_mag > 0.01:  # 只要有避障冲量就叠加
    fused.linear.x += self.avoidance_vel.linear.x
    fused.linear.y += self.avoidance_vel.linear.y
```

**优势**：避障冲量作为额外的加速度，不影响主任务速度

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `avoidance_range` | 0.3m | 避障检测范围 |
| `impulse_strength` | 1.5 m/s | 冲量强度 |
| `impulse_duration` | 0.15s | 冲量持续时间 |

### 参数调优建议

1. **冲量强度（impulse_strength）**：
   - 增大：避障更强力，但可能偏离轨迹
   - 减小：避障更温和，但可能避让不足
   - 推荐：1.0 ~ 2.0

2. **冲量持续时间（impulse_duration）**：
   - 增大：避障效果更持久，但恢复慢
   - 减小：反应更快，但可能效果不足
   - 推荐：0.1 ~ 0.2秒

3. **检测范围（avoidance_range）**：
   - 增大：提前避障，但敏感度高
   - 减小：避障更精准，但反应时间短
   - 推荐：0.2 ~ 0.5米

## 工作原理

### 避障流程
```
1. 激光雷达检测 → 发现0.3m内障碍物
2. 边沿检测 → 首次检测到（上升沿）
3. 触发冲量 → 计算避让方向和强度
4. 发送冲量 → 持续0.15秒
5. 自动衰减 → 线性衰减到零
6. 冲量结束 → 恢复主任务控制
```

### 速度融合流程
```
主任务速度（航点/追踪）
        ↓
     + 避障冲量（如果有）
        ↓
    速度限制和安全检查
        ↓
    发送到MAVROS
```

## 避让策略

### 左右避让选择
- **障碍物在右前方**（0° ~ 90°）→ 向左避让
- **障碍物在左前方**（-90° ~ 0°）→ 向右避让

### 冲量强度计算
```python
distance_factor = 1.0 - (distance / range)
impulse_magnitude = impulse_strength * distance_factor
```

距离越近，冲量越强：
- 0.1m → 冲量强度 = 1.5 * (1 - 0.1/0.3) = 1.0 m/s
- 0.2m → 冲量强度 = 1.5 * (1 - 0.2/0.3) = 0.5 m/s
- 0.3m → 冲量强度 = 1.5 * (1 - 0.3/0.3) = 0.0 m/s

## 优势对比

| 特性 | 原实现（持续式） | 新实现（冲量式） |
|------|------------------|------------------|
| 触发方式 | 持续检测 | 边沿触发 |
| 避障效果 | 停止不动 | 瞬间偏移 |
| 控制权 | 抢占主任务 | 叠加不覆盖 |
| 响应速度 | 慢 | 快 |
| 轨迹影响 | 大幅偏离 | 微小调整 |
| 任务连续性 | 中断 | 连续 |

## 测试建议

### 1. 功能测试
```bash
# 观察避障日志
rostopic echo /drone_0/obstacle_avoidance/cmd_vel

# 预期：
# - 检测到障碍物时有一个短暂的速度脉冲
# - 0.15秒后速度归零
# - 不会持续输出避障速度
```

### 2. 效果验证
- ✅ 无人机应该**不会停止**
- ✅ 轻微**偏离轨迹**避开障碍物
- ✅ 很快**恢复**主任务飞行
- ✅ 避障后**继续**执行原任务

### 3. 参数调试
如果避障效果不理想：
1. 增大 `impulse_strength` - 更强的避障力
2. 增加 `impulse_duration` - 更持久的避障
3. 调整 `avoidance_range` - 改变检测范围

## 相关文件

- `scripts/obstacle_avoidance.py` - 避障模块（冲量计算）
- `scripts/mission_controller.py` - 任务控制器（速度叠加）
- `launch/multi_drone_system.launch` - 参数配置

## 相关文档

- [FIX_WAYPOINT_SWAP_2025-01-15.md](./FIX_WAYPOINT_SWAP_2025-01-15.md) - 航点文件修复
- [FIX_TRACKING_CONTROL_2025-01-15.md](./FIX_TRACKING_CONTROL_2025-01-15.md) - 追踪控制修复
- [V11_REFACTOR_SUMMARY.md](./V11_REFACTOR_SUMMARY.md) - v11.0重构总结

---
**记录人**：东华大学 Astraeus队  
**审核状态**：已完成

