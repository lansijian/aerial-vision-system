# 追踪左右控制反向修复 - 2025-10-16

## 问题

**症状**：追踪时左右控制反向
- 目标在左边 → 无人机往右飞
- 目标在右边 → 无人机往左飞

## 根本原因

linear.y的符号错误。

### 分析

```
目标在左边：
  u_offset < 0 (左侧像素<中心)
  u_velocity = -Kp_xy * u_offset > 0 (正)
  
如果linear.y不加负号：
  linear.y = (z * u_velocity - ...) / fx
  由于u_velocity > 0，计算结果可能为正
  
在FLU坐标系中：
  linear.y > 0 表示向左
  
但转换到NED后：
  controller中: target.velocity.y = -cmd.linear.y
  所以变成向右了！错误！
```

等等，这个分析不对。让我重新想...

实际上controller会进行FLU→NED转换：
```python
target.velocity.y = -cmd.linear.y  # Y左→右（反转）
```

所以如果原始算法中linear.y的方向对应关系不对，需要反转。

## 解决方案

```python
# 修改前
cmd_vel.linear.y = (z * u_velocity - u_offset * math.cos(math.radians(45)) * cmd_vel.linear.x) / max(self.fx, 1.0)

# 修改后（添加负号）
cmd_vel.linear.y = -(z * u_velocity - u_offset * math.cos(math.radians(45)) * cmd_vel.linear.x) / max(self.fx, 1.0)
```

## 验证方法

### 测试1：目标在左边
- 目标在图像左侧（u < cx）
- u_offset < 0
- 无人机应该向左飞（linear.y > 0在FLU系）

### 测试2：目标在右边  
- 目标在图像右侧（u > cx）
- u_offset > 0
- 无人机应该向右飞（linear.y < 0在FLU系）

## 调试日志

追踪时查看：
```
[追踪] 像素:(200,180) 偏移:(-120,0) 速度:X=... Y=正值
[追踪] 像素:(440,180) 偏移:(120,0) 速度:X=... Y=负值
```

如果偏移和速度符号对应正确，说明修复成功。

---
**修复时间**：2025-10-16  
**状态**：已修复

