# 程序退出异常修复记录

**日期**：2025-01-15  
**版本**：v11.0  
**问题**：程序退出时出现publish到已关闭topic的异常

## 问题描述

### 错误信息
```
Exception in thread Thread-23:
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/timer.py", line 240, in run
    self._callback(TimerEvent(last_expected, last_real, current_expected, current_real, last_duration))
  File "/home/cty/catkin_ws/src/yolov11_ros/scripts/obstacle_avoidance.py", line 566, in _control_loop
    self.vel_pub.publish(cmd_stamped)
rospy.exceptions.ROSException: publish() to a closed topic
```

### 根本原因
1. **定时器未正确停止**：程序退出时，定时器回调仍在运行
2. **Topic已关闭**：ROS节点关闭后，Publisher已失效
3. **缺少异常处理**：未捕获发布异常

## 解决方案

### 1. 添加shutdown检查（obstacle_avoidance.py）

#### 控制循环
```python
def _control_loop(self, event):
    """控制循环"""
    # 检查是否已关闭
    if rospy.is_shutdown():
        return
    
    # ... 原有逻辑 ...
    
    # 发布速度指令
    try:
        self.cmd_vel_pub.publish(cmd_vel)
    except rospy.ROSException:
        # 节点已关闭，忽略
        pass
```

#### 障碍物共享
```python
def _share_obstacles(self, event):
    """共享本地检测的障碍物"""
    if rospy.is_shutdown():
        return
    
    # ... 原有逻辑 ...
    
    try:
        self.shared_map_pub.publish(msg)
    except rospy.ROSException:
        pass
```

#### 可视化
```python
def _visualize_obstacles(self, event):
    """可视化障碍物"""
    if rospy.is_shutdown():
        return
    
    # ... 原有逻辑 ...
    
    try:
        self.obstacle_marker_pub.publish(marker_array)
    except rospy.ROSException:
        pass
```

### 2. 改进解锁服务调用（mission_controller.py）

#### 问题
解锁服务调用失败：`service [/typhoon_h480_0/mavros/cmd/arming] responded with an error: b''`

#### 修复
```python
# 解锁服务调用
rospy.loginfo("[任务控制器] 调用解锁服务...")
try:
    response = self.arming_service(True)
    rospy.loginfo(f"[任务控制器] 解锁服务响应: {response}")
except Exception as arm_err:
    rospy.logerr(f"[任务控制器] 解锁服务调用失败: {arm_err}")
    rospy.sleep(1.0)
    continue  # 重试下一次
```

## 修复的问题

### 1. 退出时的异常
- ✅ 控制循环中添加shutdown检查
- ✅ 所有publish操作添加异常捕获
- ✅ 定时器回调在shutdown时立即返回

### 2. 解锁服务调用
- ✅ 分离服务调用异常和解锁失败
- ✅ 添加详细的错误日志
- ✅ 改进重试逻辑

## 最佳实践

### ROS定时器回调模板
```python
def _timer_callback(self, event):
    """定时器回调"""
    # 1. 检查shutdown
    if rospy.is_shutdown():
        return
    
    # 2. 执行业务逻辑
    # ...
    
    # 3. 发布时捕获异常
    try:
        self.pub.publish(msg)
    except rospy.ROSException:
        pass  # 节点已关闭，静默忽略
```

### 服务调用模板
```python
def _call_service(self):
    """服务调用"""
    try:
        response = self.service(args)
        rospy.loginfo(f"服务响应: {response}")
        return response
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")
        return None
    except Exception as e:
        rospy.logerr(f"未知异常: {e}")
        return None
```

## 测试验证

### 1. 正常退出
```bash
# 启动系统
roslaunch yolov11_ros multi_drone_system.launch

# Ctrl+C 退出
# 应该：干净退出，无异常信息
```

### 2. 解锁测试
```bash
# 观察日志
# 应该看到：
[INFO] [任务控制器] 调用解锁服务...
[INFO] [任务控制器] 解锁服务响应: success: True, result: 0
[INFO] [任务控制器] 解锁成功
```

## 相关文件

- `scripts/obstacle_avoidance.py` - 避障模块（添加shutdown检查）
- `scripts/mission_controller.py` - 任务控制器（改进服务调用）

## 相关文档

- [FIX_IMPULSE_AVOIDANCE_2025-01-15.md](./FIX_IMPULSE_AVOIDANCE_2025-01-15.md) - 冲量式避障优化
- [FIX_WAYPOINT_SWAP_2025-01-15.md](./FIX_WAYPOINT_SWAP_2025-01-15.md) - 航点文件修复
- [V11_REFACTOR_SUMMARY.md](./V11_REFACTOR_SUMMARY.md) - v11.0重构总结

---
**记录人**：东华大学 Astraeus队  
**审核状态**：已完成

