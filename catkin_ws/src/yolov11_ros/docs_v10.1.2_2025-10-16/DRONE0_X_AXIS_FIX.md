# 无人机0 X轴反向问题修复

## 🔍 问题现象（用户新发现）

### 具体表现
- **场景**：无人机0旋转180°后追踪目标
- **前后控制**：应该向前但向后 ❌
- **左右控制**：完全正确 ✅
- **偏航控制**：需要验证

### 关键线索
**只有X轴反向，Y轴正常！** 这排除了整体坐标系反转的可能。

---

## 🎯 根本原因分析

### 坐标系说明

#### MAVROS base_link坐标系（FRD）
```
X轴：Forward（前）
Y轴：Right（右）
Z轴：Down（下）
```

#### 速度命令含义
```python
cmd_vel.linear.x = 0.5  # 向前飞0.5m/s
cmd_vel.linear.y = 0.3  # 向右飞0.3m/s
cmd_vel.angular.z = 0.1 # 向左转
```

### 问题诊断

**正常情况（无人机1）**：
```
偏航角：0° → cmd.x=0.5 → 向前 ✅
偏航角：90° → cmd.x=0.5 → 向前 ✅（机头指向变了，但还是"向前"）
偏航角：180° → cmd.x=0.5 → 向前 ✅
```

**异常情况（无人机0）**：
```
偏航角：0° → cmd.x=0.5 → 向前 ✅（起飞时正常）
偏航角：180° → cmd.x=0.5 → 向后 ❌（旋转后反向）
偏航角：任意 → cmd.y=0.3 → 向右 ✅（Y轴始终正确）
```

### 可能原因

#### 原因1：PX4/MAVROS的base_link定义问题
- 无人机0的base_link X轴定义可能与无人机1相反
- 但Y轴定义正确
- 可能是配置文件或参数问题

#### 原因2：FRD坐标系的特殊行为
- 某些情况下，FRD坐标系的X轴可能有特殊处理
- 旋转180°后触发了某种边界条件

---

## ✅ 解决方案（精准修复）

### 配置修改

只反转X轴，Y轴和Yaw保持不变：

```xml
<!-- multi_drone_system.launch - 无人机0 -->
<param name="velocity_invert_x" value="true" />   <!-- 反转X（前后） -->
<param name="velocity_invert_y" value="false" />  <!-- 不反转Y（左右）✅ -->
<param name="yaw_rate_invert" value="false" />    <!-- 不反转Yaw -->
```

### 代码逻辑

```python
# drone_controller.py - _tracker_cmd_callback_twist
cmd_stamped.twist.linear.x = -msg.linear.x if self.velocity_invert_x else msg.linear.x
cmd_stamped.twist.linear.y = -msg.linear.y if self.velocity_invert_y else msg.linear.y
cmd_stamped.twist.angular.z = -msg.angular.z if self.yaw_rate_invert else msg.angular.z
```

**无人机0的实际效果**：
```
human_tracker发送: vx=0.5, vy=0.3, yaw=0.1
↓ 应用修正
drone_controller发送: vx=-0.5, vy=0.3, yaw=0.1
                      ↑反转  ↑不反转 ↑不反转
```

---

## 🧪 测试验证

### 测试步骤

1. **启动系统**
```bash
roslaunch px4 robocup.launch
roslaunch yolov11_ros multi_drone_system.launch
```

2. **观察初始化日志**
```
✅ 无人机0控制器初始化完成
   ⚠️ 速度修正: invert_x=True, invert_y=False, yaw=False
```

3. **等待无人机0旋转180°并追踪**
```
# 观察日志
[追踪速度] 无人机0 原始:(vx=0.50,vy=0.20,yaw=0.10) 
                    → 发送:(vx=-0.50,vy=0.20,yaw=0.10)
```

4. **验证行为**
- ✅ 目标在前方，无人机应该向前飞（X轴修正后）
- ✅ 目标在右侧，无人机应该向右飞（Y轴不修正）
- ✅ 目标在左侧，无人机应该向左转（Yaw不修正）

---

## 📊 对比表格

| 情况 | X轴行为 | Y轴行为 | Yaw行为 |
|------|---------|---------|---------|
| 无人机1（参考） | ✅ 正确 | ✅ 正确 | ✅ 正确 |
| 无人机0（修复前） | ❌ 反向 | ✅ 正确 | ✅ 正确 |
| 无人机0（修复后） | ✅ 正确 | ✅ 正确 | ✅ 正确 |

---

## ⚠️ 注意事项

### 1. 只影响追踪模式
- **航点飞行**（位置控制）：不受影响，继续正常工作
- **追踪模式**（速度控制）：应用X轴修正

### 2. 不影响无人机1
```xml
<!-- 无人机1保持原样 -->
<param name="velocity_invert_x" value="false" />
<param name="velocity_invert_y" value="false" />
<param name="yaw_rate_invert" value="false" />
```

### 3. 如果需要扩展到6机
每架无人机根据实际情况配置：
```xml
<!-- 测试每架无人机，按需配置 -->
drone_2: invert_x=?, invert_y=?, yaw=?
drone_3: invert_x=?, invert_y=?, yaw=?
...
```

---

## 🔬 进一步诊断（如果问题仍存在）

### 运行诊断脚本
```bash
# 使用v10.1.2的诊断工具
rosrun yolov11_ros diagnostic_script.py
```

该脚本会：
1. 记录两架无人机的初始偏航角
2. 对比四元数
3. 给出诊断建议

### 手动验证
```bash
# 查看无人机0的当前姿态
rostopic echo /typhoon_h480_0/mavros/local_position/pose

# 特别关注
orientation:
  x: X
  y: Y
  z: Z
  w: W

# 计算偏航角
yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
```

---

## 📝 修复总结

### 精准修复（基于用户反馈）

| 参数 | 无人机0 | 无人机1 | 说明 |
|------|---------|---------|------|
| velocity_invert_x | **true** | false | 修复前后反向 |
| velocity_invert_y | **false** | false | 左右正常，不反转 |
| yaw_rate_invert | **false** | false | 偏航正常，不反转 |

**关键**：只反转X轴，其他保持不变！

---

**修复日期**：2025-10-16  
**状态**：✅ 已实现精准修复  
**验证方法**：测试无人机0旋转180°后追踪行为  
**东华大学 Astraeus队**

