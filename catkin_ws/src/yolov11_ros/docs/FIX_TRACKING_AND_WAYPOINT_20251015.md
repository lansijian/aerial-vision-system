# 修复记录 - 追踪与航点问题修复

## 修复时间
2025-10-15

## 问题描述

### 1. 检测到行人后不追踪
- human_tracker发现目标但无人机不切换到追踪模式
- 话题名称不匹配导致模式切换失败

### 2. 无人机撞墙
- 航点坐标范围不正确
- 原设置：-15到25米（太小）
- 实际需要：-45到105米

## 解决方案

### 1. 追踪功能修复

#### A. 话题名称修正
```python
# human_tracker.py
# 修正前：
self.tracking_status_pub = rospy.Publisher(
    f'/drone_{self.drone_id}/tracking_request',  # 错误
    Bool, queue_size=1
)

# 修正后：
self.tracking_mode_pub = rospy.Publisher(
    f'/drone_{self.drone_id}/request_tracking_mode',  # 正确
    Bool, queue_size=1
)
```

#### B. 添加模式监听
```python
# human_tracker.py添加
def _mode_callback(self, msg):
    """飞行模式回调"""
    self.flight_mode = msg.data
    # 如果不是追踪模式，停止发送速度命令
    if self.flight_mode != "TRACKING":
        self.cmd_vel = Twist()
        self.cmd_vel_pub.publish(self.cmd_vel)
```

#### C. 修改检测逻辑
```python
# 只在TRACKING模式下计算和发布速度命令
if self.flight_mode == "TRACKING":
    self._compute_tracking_velocity(best_target)
    self.cmd_vel_pub.publish(self.cmd_vel)
```

### 2. 航点轨迹修正

#### 无人机0航点（21个点）
- 起点：(0, 0)
- 范围：X轴 -45到105米，Y轴 -45到45米
- 采用S型扫描路径

#### 无人机1航点（21个点）
- 起点：(0, 0)
- 范围：X轴 -45到105米，Y轴 -45到45米
- 采用反向S型扫描路径

## 代码修改

### human_tracker.py
1. 修正话题发布器名称
2. 添加flight_mode状态变量
3. 添加_mode_callback函数
4. 修改_detection_callback逻辑
5. 修改_start_tracking和_stop_tracking

### waypoints_drone_0.json
- 更新为正确的21个航点坐标

### waypoints_drone_1.json
- 更新为正确的21个航点坐标（反向路径）

## 系统工作流程

```
1. 起飞 → 航点模式(WAYPOINT)
   ↓
2. 航点巡检中
   ↓
3. 检测到目标 → human_tracker请求切换
   ↓
4. drone_controller切换到TRACKING模式
   ↓
5. human_tracker开始发送速度命令
   ↓
6. 目标丢失 → 请求恢复WAYPOINT模式
   ↓
7. 继续航点巡检
```

## 测试验证

1. **航点飞行测试**
   - 无人机不再撞墙
   - 按照正确路径飞行
   - 覆盖整个搜索区域

2. **追踪测试**
   - 检测到目标后自动切换模式
   - 稳定追踪目标
   - 丢失后恢复巡检

## 状态
✅ 已修复
