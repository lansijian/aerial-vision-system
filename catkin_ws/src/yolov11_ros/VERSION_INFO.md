# 版本信息 - YOLOv11 ROS 多机协同系统

## 当前版本：v10.1.2-simple

**发布日期**：2025-10-16  
**版本状态**：✅ 极简版（回归本质）  
**维护团队**：东华大学 Astraeus队

## 版本特性

### 🚀 核心功能
- ✅ 极简比例控制（参考XTDrone）
- ✅ 保持YOLO检测框在图像中心
- ✅ 单一增益Kp_yaw=0.002
- ✅ 无分级、无平滑、无复杂逻辑
- ✅ 航点飞行使用位置控制
- ✅ 支持2-6架无人机扩展
- ✅ v10.0.1高精度坐标计算
- ✅ 符合官方记分系统标准

### 🔧 技术特点（极简设计）
- 简单比例控制：yaw = -Kp * u
- 核心算法仅40行
- 只有2个主要参数
- 代码简洁易调试
- 回归追踪本质

## 版本历史

### v10.1.2-simple (2025-10-16) - 极简追踪 ⭐当前版本
- ✨ **回归本质**：参考XTDrone极简设计
- ✅ 单一比例增益Kp_yaw=0.002
- ✅ 去除分级增益、平滑滤波
- ✅ 核心算法仅40行
- ✅ 禁用横向速度，完全靠偏航
- ✅ 简洁日志：✅居中/🔄调整/⚠️偏离
- 📄 详见[SIMPLE_TRACKING.md](SIMPLE_TRACKING.md)

### v10.1.2-plan3-optimized (2025-10-16) - 提高响应速度v2（已废弃）
- ⚡ **关键优化**：基于终端数据提高响应速度
- ✅ 修复UnboundLocalError（yaw_gain未定义）
- ✅ 提高偏航增益33-67%（0.002-0.008）
- ✅ 提高max_yaw_rate到0.8 rad/s（+60%）
- ✅ 提高平滑系数到0.50（+67%）
- ✅ 改进日志（状态标识：✅对齐/🔄调整/⚠️偏离）
- 📊 对齐速度提升33-37%
- 📄 详见[TRACKING_OPTIMIZATION_V2.md](TRACKING_OPTIMIZATION_V2.md)

### v10.1.2-plan3 (2025-10-16) - 修复追踪控制
- 🔧 **关键修复**：偏航增益过大导致抖动（0.625 → 0.0012-0.006）
- ✅ 参考plan3_optimized优化版
- ✅ 分级偏航增益：very_close/close/medium/far
- ✅ 使用box_size面积反馈（更稳定）
- ✅ 添加平滑滤波（alpha=0.30）
- ✅ 禁用横向速度（vy=0），专注偏航
- 📄 详见[TRACKING_FIX_PLAN3.md](TRACKING_FIX_PLAN3.md)

### v10.1.2-simplified (2025-10-16) - 追踪简化优化（已废弃）
- 🎯 **核心优化**：专注保持检测框在图像中心
- ❌ 删除Kalman滤波器（降低延迟67%）
- ❌ 删除积分控制项（避免超调）
- ❌ 删除绕圈搜索（快速回航点）
- ✅ 简化控制逻辑（代码减少19%）
- ✅ 提高响应速度（Kp_yaw=100, Kp_dist=2.0）
- ✅ 优化丢失恢复（0.5秒立即回航点）
- 📄 详见[TRACKING_SIMPLIFIED.md](TRACKING_SIMPLIFIED.md)

### v10.1.2-refactored (2025-10-16) - 架构重构
- 🔧 **关键修复**：航点飞行改用位置控制
- ✅ 解决v10.1.2无人机反向问题
- ✅ 归档v11.3.9混沌代码
- ✅ 清理冗余脚本
- ✅ 基于v10.1.2稳定架构重构

### v11.3.9-tracking-fix (2025-10-16) - 已废弃
- ❌ 航点使用速度控制导致反向
- ❌ 三层嵌套架构过于复杂
- ✅ 已归档到`archive_v11.3.9/`

### v11.1.1-final (2025-10-16) - 已废弃
- ❌ 航点抖动问题
- ❌ 控制冲突
- ✅ 已归档到`archive_v11.3.9/`

### v10.1.2 (2025-10-15) - 已归档
- ❌ 无人机0控制反向
- ⚠️ 航点使用速度控制
- ✅ 稳定的三模块架构
- ✅ 已归档到`archive_v10.1.2/`

### v10.0.1 (2025-10-14) - 已归档
- ✅ 高精度坐标算法
- ✅ 修正颜色映射
- ⚠️ 误差约2米

### v10.0.0 (2025-10-13) - 已归档
- ✅ 基础双机协同
- ✅ YOLO目标检测
- ✅ 航点巡检系统

## 核心模块版本

| 模块 | 版本 | 最后更新 | 状态 | 说明 |
|------|------|----------|------|------|
| human_tracker.py | 10.1.2-simple | 2025-10-16 | ✨ 极简 | 参考XTDrone极简设计 |
| drone_controller.py | 10.1.2-refactored | 2025-10-16 | ✅ 稳定 | 速度控制转发 |
| waypoint_navigator.py | 10.1.2-refactored | 2025-10-16 | ✅ 稳定 | 位置控制航点 |
| gimbal_controller.py | 1.0.0 | 2025-01-15 | ✅ 稳定 | 云台控制 |
| score_calculator.py | 10.1.0 | 2025-01-12 | ✅ 稳定 | 记分系统 |
| yolo_v11.py | 1.0.1 | 2025-01-15 | ✅ 稳定 | 目标检测 |

## 依赖版本

- **ROS**: Noetic (Ubuntu 20.04)
- **Python**: 3.8+
- **YOLOv11**: ultralytics 8.0+
- **MAVROS**: 1.15.0+
- **PX4**: 1.13+
- **OpenCV**: 4.5+
- **NumPy**: 1.19+

## 兼容性

### 支持的无人机型号
- ✅ typhoon_h480 (默认)
- ✅ iris
- ✅ solo
- ⚠️ 其他型号需要调整参数

### 支持的传感器
- ✅ 单目相机 (640x360)
- ✅ IMU
- ✅ GPS

## 关键修复说明

### v10.1.2反向问题诊断

**问题**：无人机0/1控制全部反向

**根本原因**：
```python
# ❌ 错误的速度控制（v10.1.2）
def _compute_navigation_command(self):
    # ... 计算速度
    body_vel_x = control[0] * cos_yaw + control[1] * sin_yaw
    body_vel_y = -control[0] * sin_yaw + control[1] * cos_yaw
    # 问题：速度控制容易受坐标系影响
```

**解决方案**：
```python
# ✅ 正确的位置控制（v10.1.2-refactored）
def _create_position_target(self, x, y, z):
    cmd = PositionTarget()
    cmd.position.x = x  # 直接位置控制
    cmd.position.y = y
    cmd.position.z = z
    # 优势：不受坐标系转换影响
```

## 架构对比

### v11.3.9（已废弃）
```
target_tracker → waypoint_mission → mission_controller → MAVROS
（3层嵌套，复杂度高，稳定性差）
```

### v10.1.2-refactored（当前）
```
waypoint_navigator ──位置控制──> MAVROS
human_tracker ──速度控制──> drone_controller ──> MAVROS
（2层清晰，稳定可靠）
```

## 归档说明

### archive_v10.1.2/
- 原始v10.1.2版本（存在反向问题）
- 包含诊断脚本和问题分析

### archive_v11.3.9/
- v11全系列混沌代码
- 归档原因：架构过于复杂，航点控制错误

## 版本规划

### v10.1.3 (计划中)
- [ ] 进一步优化坐标精度
- [ ] 添加多机协调
- [ ] 改进模式切换

### v11.0 (远期)
- [ ] 深度学习路径优化
- [ ] 自适应任务分配
- [ ] 云端监控支持

## 技术文档

详细技术说明请参考：
- [README.md](README.md) - 项目说明
- [CHANGELOG.md](CHANGELOG.md) - 更新日志
- [TECHNICAL.md](TECHNICAL.md) - 技术文档（v10.0坐标算法）

---

**最后更新**: 2025-10-16  
**维护者**: 东华大学 Astraeus队  
**版本**: v10.1.2-refactored
