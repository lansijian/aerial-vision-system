# v11.3.9 归档说明

## 📦 归档原因
v11版本经过多次迭代，代码架构变得混沌，存在多个问题需要重构。

## 🔴 核心问题
1. **航点控制错误**：使用速度控制而非位置控制，导致无人机反向飞行
2. **架构过于复杂**：mission_controller、waypoint_mission、target_tracker三层嵌套
3. **模式切换不稳定**：追踪和航点模式切换存在延迟和冲突

## 📂 归档文件

### 主要脚本
- `mission_controller_v11.3.9.py` - 任务控制器（388行）
- `waypoint_mission_v11.3.9.py` - 航点任务（389行）
- `target_tracker_v11.3.9.py` - 目标追踪（475行）

### 问题分析
1. **waypoint_mission使用速度控制** - 这是导致无人机反向的根本原因
2. **三层控制链路** - target_tracker → waypoint_mission → mission_controller → MAVROS
3. **话题转发延迟** - 追踪命令经过多次转发，响应慢

## ✅ 重构方向（v10.1.2架构）

### 新架构特点
1. **航点使用位置控制** - 直接发布PositionTarget到MAVROS
2. **简化控制链路** - 只有两层：drone_controller和专用模块
3. **稳定的模式切换** - 基于v10.1.2的成熟架构

### 三模块架构
```
drone_controller.py     - 主控制器（起飞、模式切换、命令转发）
waypoint_navigator.py   - 航点导航（位置控制）
human_tracker.py        - 人体追踪（速度控制）
```

## 📊 版本对比

| 特性 | v11.3.9 | v10.1.2重构 |
|------|---------|------------|
| 航点控制 | ❌ 速度控制 | ✅ 位置控制 |
| 控制层数 | ❌ 3层 | ✅ 2层 |
| 代码行数 | 1252行 | ~1100行 |
| 稳定性 | ❌ 不稳定 | ✅ 稳定 |

## 🗂️ 归档日期
2025-10-16

---
**东华大学 Astraeus队**  
**2025 RoboCup中国赛**

