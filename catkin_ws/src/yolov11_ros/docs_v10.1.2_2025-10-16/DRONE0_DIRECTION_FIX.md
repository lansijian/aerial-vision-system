# 无人机0方向反向问题修复

## 问题描述
- **现象**：无人机0在追踪模式下前后方向反向
- **影响范围**：仅追踪模式（速度控制），航点模式正常（位置控制）
- **对比**：无人机1追踪模式正常

## 根本原因
- **位置控制**（航点飞行）：直接指定目标位置，不受坐标系影响 ✅
- **速度控制**（追踪模式）：依赖机体坐标系，受初始偏航角影响 ❌

**诊断结果**：
- 无人机0和无人机1的初始偏航角可能相差180度
- 导致机体坐标系（FRD）的X轴方向相反
- 速度命令`cmd_vel.linear.x`在两架无人机上含义不同

## 解决方案（参数化配置）

### 1. drone_controller.py - 添加速度修正参数
```python
# 参数化配置（避免硬编码）
self.velocity_invert_x = rospy.get_param('~velocity_invert_x', False)
self.velocity_invert_y = rospy.get_param('~velocity_invert_y', False)
self.yaw_rate_invert = rospy.get_param('~yaw_rate_invert', False)

# 接收追踪速度时应用修正
def _tracker_cmd_callback_twist(self, msg):
    cmd_stamped.twist.linear.x = -msg.linear.x if self.velocity_invert_x else msg.linear.x
    cmd_stamped.twist.linear.y = -msg.linear.y if self.velocity_invert_y else msg.linear.y
    cmd_stamped.twist.angular.z = -msg.angular.z if self.yaw_rate_invert else msg.angular.z
```

### 2. multi_drone_system.launch - 配置参数
```xml
<!-- 无人机0 - 需要修正 -->
<node pkg="yolov11_ros" type="drone_controller.py" name="drone_controller">
  <param name="velocity_invert_x" value="true" />
  <param name="velocity_invert_y" value="true" />
  <param name="yaw_rate_invert" value="true" />
</node>

<!-- 无人机1 - 方向正常 -->
<node pkg="yolov11_ros" type="drone_controller.py" name="drone_controller">
  <param name="velocity_invert_x" value="false" />
  <param name="velocity_invert_y" value="false" />
  <param name="yaw_rate_invert" value="false" />
</node>
```

## 优势

1. ✅ **参数化配置** - 不是硬编码
2. ✅ **支持6机扩展** - 每架无人机独立配置
3. ✅ **清晰可维护** - 配置在launch文件中
4. ✅ **调试友好** - 有日志输出修正前后的速度

## 测试验证

启动系统后观察日志：
```bash
# 无人机0应该显示
✅ 无人机0控制器初始化完成
   ⚠️ 速度修正: invert_x=True, invert_y=True, yaw=True

# 追踪时应该显示
[速度修正] 无人机0 原始:(vx=0.50,vy=0.20) 修正后:(vx=-0.50,vy=-0.20)
```

## 未来改进

如果需要支持更多无人机，只需在launch文件中为每架无人机配置相应参数：
```xml
<param name="velocity_invert_x" value="true/false" />
<param name="velocity_invert_y" value="true/false" />
<param name="yaw_rate_invert" value="true/false" />
```

---
**修复日期**：2025-10-16  
**版本**：v10.1.2-refactored-final  
**东华大学 Astraeus队**

