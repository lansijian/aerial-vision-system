# 追踪系统调试指南

**日期**: 2025-10-16  
**版本**: v10.1.2-stable  

---

## 🔍 调试模式使用

### 开启调试模式

```bash
# 方法1: 启动时开启
roslaunch yolov11_ros multi_drone_system.launch debug_mode:=true

# 方法2: 运行时开启（需要重启节点）
rosparam set /drone_0/human_tracker/debug_mode true
# 然后重启human_tracker节点
rosnode kill /human_tracker
```

---

## 📊 调试日志解读

### 1. 检测框信息
```
[调试-检测框] 原始框: xmin=285.3 xmax=354.8 ymin=120.5 ymax=240.2
```
- 检查检测框位置是否合理
- 检查框大小是否稳定

### 2. 平滑效果
```
[调试-平滑] 原始: u=320.1 v=180.4 h=119.7 | 平滑后: u=318.5 v=179.8 h=121.3
```
- 对比原始值和平滑值
- 平滑后应该变化更平缓
- 如果差异大，说明抖动严重

### 3. 小框修正
```
[调试-小框修正] h=35.2 < 阈值50 | 修正因子=1.044 → h_修正=36.7
```
- 只在框高 < 50px 时触发
- 修正因子越大，距离补偿越多
- 用于修正远距离测距误差

### 4. 偏移计算
```
[调试-偏移] 图像中心=(320,180) | 框中心=(285.3,175.2) | 偏移=(u:-34.7, v:-4.8)px
```
- u_offset < 0: 框在左边
- u_offset > 0: 框在右边
- v_offset < 0: 框在上面
- v_offset > 0: 框在下面

### 5. 偏航控制（关键！）
```
[调试-偏航] u_offset=-34.7px | 原始yaw=-0.0694 | 死区(10px)=❌ | 最终yaw=-0.0694rad/s
```

**重要逻辑验证**：
- ✅ 框在左边 (`u_offset < 0`) → 左转 (`yaw < 0`)
- ✅ 框在右边 (`u_offset > 0`) → 右转 (`yaw > 0`)
- ✅ 死区内 (`|u_offset| < 10px`) → 不转 (`yaw = 0`)

**如果发现反向**：
- ❌ 框在左边 → 右转（`yaw > 0`）
- ❌ 框在右边 → 左转（`yaw < 0`）
- 说明符号错误，需要修改公式

### 6. 前后距离控制
```
[调试-前后] 理想h=160 实际h=121.3 | 误差=+38.7px | 原始vx=+0.363 | 死区=❌ | 最终vx=+0.363m/s
```

**逻辑验证**：
- ✅ 框太小 (`h < ideal`) → 前进 (`vx > 0`)
- ✅ 框太大 (`h > ideal`) → 后退 (`vx < 0`)
- ✅ 死区内 (`|error| < 10px`) → 不动 (`vx = 0`)

### 7. 最终速度命令
```
[调试-最终速度] vx=+0.363 vy=+0.000 vz=+0.000 yaw=-0.0694
```
- `vx`: 前后速度（前进为正）
- `vy`: 左右速度（应该始终为0，靠偏航对齐）
- `vz`: 上下速度（应该始终为0，PX4自动保持）
- `yaw`: 偏航率（rad/s）

---

## 🐛 常见问题诊断

### 问题1: 框在左边，无人机向右转 ❌

**症状**:
```
[调试-偏航] u_offset=-50.0px | yaw=+0.100rad/s
```
框在左（负数），但yaw为正（右转）

**原因**: 偏航公式符号错误

**修复**: 已修复为 `yaw_rate = Kp_yaw * u_offset`（无负号）

---

### 问题2: 框在中心附近抖动

**症状**:
```
[调试-偏移] 偏移=(u:+3.2, v:-2.1)px
[调试-偏航] yaw=+0.006rad/s
```
在死区内（10px）还在输出速度

**诊断**:
- 检查 `dead_zone_px` 参数
- 应该在 ±10px 内输出 yaw=0

**修复**:
```bash
rosparam set /drone_0/human_tracker/dead_zone_px 15
```

---

### 问题3: 远距离时忽远忽近

**症状**:
```
[调试-前后] 实际h=35.2 | vx=+0.532
[调试-前后] 实际h=34.8 | vx=-0.488
```
速度频繁正负切换

**诊断**:
- 小框修正未生效
- 平滑帧数不足

**修复**:
```bash
# 增加小框修正斜率
rosparam set /drone_0/human_tracker/small_box_factor_slope 0.004

# 增加平滑帧数
rosparam set /drone_0/human_tracker/bbox_smooth_frames 5
```

---

### 问题4: 追踪响应太慢

**症状**:
```
[调试-偏航] u_offset=-100.0px | yaw=-0.200rad/s | 最终yaw=-0.200rad/s
```
偏移很大但速度很小

**诊断**:
- `Kp_yaw` 增益太小

**修复**:
```bash
rosparam set /drone_0/human_tracker/Kp_yaw 0.003
```

---

### 问题5: 框抖动严重

**症状**:
```
[调试-平滑] 原始: u=320.1 v=180.4 h=119.7
[调试-平滑] 原始: u=285.6 v=175.2 h=98.3
[调试-平滑] 原始: u=330.8 v=185.1 h=135.2
```
连续帧差异大（>20px）

**诊断**:
- YOLO检测不稳定
- 平滑效果不够

**修复**:
```bash
# 增加平滑帧数
rosparam set /drone_0/human_tracker/bbox_smooth_frames 5

# 增大死区
rosparam set /drone_0/human_tracker/dead_zone_px 15
```

---

## 📝 调试检查清单

### 启动系统
```bash
# 1. 启动Gazebo
roslaunch px4 robocup.launch

# 2. 启动无人机（调试模式）
roslaunch yolov11_ros multi_drone_system.launch debug_mode:=true
```

### 验证偏航控制方向 ✅
观察日志，验证逻辑：
```
框在左边 (u<0) → 左转 (yaw<0) ✅
框在右边 (u>0) → 右转 (yaw>0) ✅
框居中 (|u|<10) → 不转 (yaw=0) ✅
```

### 验证前后控制方向 ✅
观察日志，验证逻辑：
```
框太小 (h<ideal) → 前进 (vx>0) ✅
框太大 (h>ideal) → 后退 (vx<0) ✅
框合适 (|error|<10) → 不动 (vx=0) ✅
```

### 验证平滑效果 ✅
对比原始值和平滑值：
```
原始值跳动 ±10px，平滑值稳定 ±3px ✅
```

### 验证小框修正 ✅
远距离时观察：
```
h<50px 时有修正因子 ✅
修正后框高增大，vx减小 ✅
```

---

## 🎯 实时监控命令

### 查看速度命令
```bash
rostopic echo /drone_0/human_tracker/cmd_vel
```

### 查看飞行模式
```bash
rostopic echo /drone_0/flight_mode
```

### 查看检测结果
```bash
rostopic echo /yolo_detector/drone_0/detections
```

### 查看无人机位置
```bash
rostopic echo /typhoon_h480_0/mavros/local_position/pose
```

---

## 📊 参数调优参考

### 保守参数（稳定优先）
```bash
rosparam set /drone_0/human_tracker/Kp_yaw 0.0015
rosparam set /drone_0/human_tracker/Kp_distance 1.0
rosparam set /drone_0/human_tracker/dead_zone_px 15
rosparam set /drone_0/human_tracker/bbox_smooth_frames 5
```

### 激进参数（响应优先）
```bash
rosparam set /drone_0/human_tracker/Kp_yaw 0.003
rosparam set /drone_0/human_tracker/Kp_distance 2.0
rosparam set /drone_0/human_tracker/dead_zone_px 5
rosparam set /drone_0/human_tracker/bbox_smooth_frames 3
```

### 推荐参数（平衡）
```bash
rosparam set /drone_0/human_tracker/Kp_yaw 0.002
rosparam set /drone_0/human_tracker/Kp_distance 1.5
rosparam set /drone_0/human_tracker/dead_zone_px 10
rosparam set /drone_0/human_tracker/bbox_smooth_frames 3
```

---

## ✅ 符号修复验证

### 修复前（错误）❌
```python
yaw_rate = -self.Kp_yaw * u_offset

# 框在左边: u=-100
# yaw = -0.002 * (-100) = +0.2 (右转) ❌ 错误！
```

### 修复后（正确）✅
```python
yaw_rate = self.Kp_yaw * u_offset

# 框在左边: u=-100
# yaw = 0.002 * (-100) = -0.2 (左转) ✅ 正确！
```

---

**东华大学 Astraeus队**  
**2025 RoboCup中国赛**  
**版本**: v10.1.2-stable  
**状态**: ✅ 偏航符号已修复







