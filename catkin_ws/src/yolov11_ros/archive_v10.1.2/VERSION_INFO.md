# 版本信息 - v10.1.2

## 版本概述
- **版本号**: v10.1.2
- **归档时间**: 2025-10-15
- **代码状态**: 混合版本，部分功能正常
- **主要问题**: 无人机0控制反向

## 版本历史

### v10.0.1 (2025-10-14)
- ✅ 修复world文件actor颜色映射错误
- ✅ 系统基本稳定运行
- ⚠️ 坐标误差约2米

### v10.1 (2025-10-14) [失败]
- ❌ 引入卡尔曼滤波等7项优化技术
- ❌ 误差反而增大到5米以上
- ❌ 已废弃

### v10.2 (2025-10-14) [失败]  
- ❌ 简化算法版本
- ❌ 追踪效果差
- ❌ 已废弃

### v10.0.1-restored (2025-10-14)
- ✅ 恢复到稳定版本
- ✅ 追踪稳定
- ⚠️ 误差约2米

### v10.1.2 (2025-10-15) [当前]
- 🔄 修复了部分问题
- ✅ 话题名称修正
- ✅ 添加飞行模式监听
- ❌ 无人机0控制反向未解决
- ⚠️ 代码混乱需要重构

## 文件清单

### 核心脚本
1. `drone_controller_v10.1.2.py`
   - 主控制器，负责起飞、模式切换
   - 338行代码
   - 存在问题：缺少velocity_invert参数支持

2. `waypoint_navigator_v10.1.2.py`
   - 航点导航器
   - 321行代码
   - 存在问题：无人机0飞行方向错误

3. `human_tracker_v10.1.2.py`
   - 人体追踪器（基于v10.0.1修复版）
   - 438行代码
   - 已修正话题名称和模式监听

### 配置文件
4. `multi_drone_system_v10.1.2.launch`
   - 系统启动文件
   - 支持2-6架无人机
   - 无人机0使用waypoints_drone_1.json
   - 无人机1使用waypoints_drone_0.json

5. `waypoints_drone_0.json`
   - 20个航点，S型扫描路径
   - 范围：-45到105米（X轴），-45到45米（Y轴）

6. `waypoints_drone_1.json`
   - 20个航点，反向S型扫描路径
   - 范围：-45到105米（X轴），-45到45米（Y轴）

## 依赖关系

```
drone_controller.py
    ├── 订阅 waypoint_navigator/cmd_vel
    ├── 订阅 human_tracker/cmd_vel
    ├── 发布 flight_mode
    └── 发布到 MAVROS/cmd_vel

waypoint_navigator.py
    ├── 订阅 flight_mode
    ├── 订阅 local_position/pose
    └── 发布 cmd_vel

human_tracker.py
    ├── 订阅 flight_mode
    ├── 订阅 YOLO detections
    ├── 发布 request_tracking_mode
    ├── 发布 request_waypoint_mode
    └── 发布 ActorInfo
```

## 参数配置

### 无人机0参数
- drone_id: 0
- vehicle_type: typhoon_h480
- takeoff_altitude: 3.0
- spawn_offset_x: 0.0
- spawn_offset_y: -3.0
- waypoint_file: waypoints_drone_1.json
- **问题**: 速度命令需要反转

### 无人机1参数
- drone_id: 1
- vehicle_type: typhoon_h480
- takeoff_altitude: 3.0
- spawn_offset_x: 3.0
- spawn_offset_y: -3.0
- waypoint_file: waypoints_drone_0.json
- **状态**: 正常工作

## 测试结果

### 功能测试
| 功能 | 无人机0 | 无人机1 |
|------|---------|---------|
| 起飞解锁 | ✅ | ✅ |
| 航点飞行 | ❌ 反向 | ✅ |
| 目标追踪 | ❌ 反向 | ✅ |
| 模式切换 | ✅ | ✅ |
| 坐标发布 | ✅ 误差2m | ✅ 误差2m |

### 性能指标
- 检测频率: 30Hz
- 控制频率: 50Hz
- 坐标误差: ~2米
- CPU占用: 30-40%
- 内存占用: 1.5GB

## 重构计划

### 第一阶段：问题诊断
1. 分析无人机0和1的初始化差异
2. 检查MAVROS配置
3. 记录姿态和坐标系信息

### 第二阶段：参数化改造
1. 添加velocity_transform配置
2. 支持per-drone参数
3. 消除硬编码

### 第三阶段：6机扩展
1. 动态配置支持
2. 性能优化
3. 完整测试

## 备注

⚠️ **警告**：当前版本存在关键问题，不建议直接用于比赛

📌 **提醒**：重构时保持向后兼容性，支持6机扩展

---

*归档人：AI助手*
*日期：2025-10-15*
*用途：代码重构前的版本备份*
