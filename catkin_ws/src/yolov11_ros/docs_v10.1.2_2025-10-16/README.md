# 技术文档归档 - v10.1.2-optimized

## 归档日期
2025-10-16

## 文档列表

### 优化方案文档
- `30MIN_OPTIMIZATION_REPORT.md` - 30分钟优化实施报告
- `TRACKING_OPTIMIZATION_2025-10-16.md` - 追踪优化详解
- `PARAMETERS_TUNING_GUIDE.md` - 现场参数调节指南

### 坐标系修复文档
- `COORDINATE_ALGORITHM_SIMPLIFIED.md` - 简化坐标算法说明
- `COORDINATE_FRAME_FINAL_FIX.md` - 坐标系最终修复
- `COORDINATE_SYSTEM_FIX.md` - 坐标系统一修复
- `YAW_ROTATION_FIX_FINAL.md` - 偏航角旋转修复

### 无人机0方向问题文档
- `DRONE0_DIRECTION_FIX.md` - 无人机0方向修复
- `DRONE0_REVERSAL_ANALYSIS.md` - 反向问题分析
- `DRONE0_X_AXIS_FIX.md` - X轴修复说明

### 高度控制文档
- `HEIGHT_CONTROL_FIX.md` - 高度控制修复
- `HEIGHT_HOLD_FINAL.md` - 高度保持最终方案

### 综合修复文档
- `FINAL_FIXES_2025-10-16.md` - 最终修复总结
- `FINAL_STATUS_2025-10-16.md` - 最终状态总结
- `FIXES_2025-10-16.md` - 修复记录

### 测试验证文档
- `TESTING_CHECKLIST.md` - 测试验证清单
- `TRACKING_DIRECTION_STATUS.md` - 追踪方向状态说明

## 主要修复内容

### 1. 坐标系统一（世界坐标系）
- 使用PositionTarget + FRAME_LOCAL_NED
- 机体速度 → 世界速度转换
- 解决旋转180°问题

### 2. 追踪优化
- 2D Kalman滤波器
- 自适应Kp_yaw（80/box_height）
- PI积分控制
- 框高误差法
- 100Hz控制频率

### 3. 坐标精度提升
- 真实行人高度1.78m
- 简化投影法
- 5帧平滑

### 4. 系统优化
- 高度保持vz=0
- 边缘急速旋转
- 控制权冲突修复
- 12个参数ROS化

## 版本信息

- **版本号**：v10.1.2-optimized
- **日期**：2025-10-16
- **状态**：稳定运行
- **团队**：东华大学 Astraeus队

---

**说明**：本目录保存v10.1.2-optimized版本的所有技术文档，供技术复查和问题排查使用。

