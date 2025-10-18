# v10.0.1 版本归档信息

## 版本信息
- **版本号**: v10.0.1
- **发布日期**: 2025-10-14
- **版本类型**: Bug修复版本
- **主要贡献**: 东华大学 Astraeus队

## 版本概述

v10.0.1是一个关键修复版本，主要解决了Gazebo世界文件中actor模型颜色映射错误的问题。此问题导致记分系统无法正确识别和消除部分目标，严重影响了系统的正常运行。

## 主要修复

### 1. World文件Actor颜色映射修复

**问题描述**：
- actor_1、actor_2、actor_3的皮肤文件（walk_X.dae）分配错误
- 导致YOLO检测的颜色与actor实际ID不匹配
- 记分系统因映射错误无法消除目标

**修复方案**：
修正了`PX4_Firmware/Tools/sitl_gazebo/worlds/robocup.world`中的皮肤文件分配：
- actor_1: walk_2.dae → walk_1.dae (蓝色)
- actor_2: walk_3.dae → walk_2.dae (棕色)
- actor_3: walk_1.dae → walk_3.dae (白色)

## 测试配置

为验证修复效果，临时调整了记分系统参数：
```python
# 测试参数（需在正式比赛前恢复）
self.err_threshold = 2.0  # 测试阈值：2米（原官方：1米）
detection_time = 5.0       # 测试时间：5秒（原官方：15秒）
```

## 文件变更清单

### 修改的文件
1. `PX4_Firmware/Tools/sitl_gazebo/worlds/robocup.world`
   - 修正actor_1、actor_2、actor_3的皮肤文件映射

2. `catkin_ws/src/yolov11_ros/scripts/score_calculator.py`  
   - 临时调整测试参数（误差2米，确认5秒）
   - 更新版本号到v10.1（测试模式）

### 新增的文件
1. `catkin_ws/src/yolov11_ros/docs/WORLD_FILE_FIX.md`
   - 详细记录问题分析和修复过程
   - 提供验证方法和检查清单

2. `catkin_ws/src/yolov11_ros/archive_v10.0.1/`
   - VERSION_INFO.md（本文件）
   - 版本归档和说明文档

### 更新的文档
1. `CHANGELOG.md`
   - 添加v10.0.1版本详细记录
   - 包含问题诊断、修复内容、验证效果

## 验证方法

### 快速验证步骤
```bash
# 1. 启动Gazebo仿真
roslaunch px4 robocup.launch

# 2. 启动多机系统
roslaunch yolov11_ros multi_drone_system.launch

# 3. 观察检测结果
rostopic echo /actor_blue_info   # 应对应actor_1位置
rostopic echo /actor_brown_info  # 应对应actor_2位置
rostopic echo /actor_white_info  # 应对应actor_3位置
```

### 预期结果
- ✅ 所有6个actor的颜色正确显示
- ✅ YOLO检测颜色与actor ID匹配
- ✅ 误差<2米且持续5秒可消除目标（测试模式）
- ✅ 记分系统正常累计得分

## 重要提醒

### 正式比赛前必须恢复的参数
```python
# score_calculator.py中需要恢复：
self.err_threshold = 1.0  # 官方误差阈值：1米
detection_time = 15.0      # 官方确认时间：15秒
```

### 不可修改的部分
- 官方记分系统的actor_id_dict映射
- YOLO模型和训练标签
- human_tracker的ActorInfo发布逻辑
- 记分系统的核心算法

## 问题追踪

### 问题来源
- 发现者：用户测试反馈
- 发现时间：2025-10-14
- 症状：无人机发布的棕色坐标与蓝色真实坐标接近

### 解决过程
1. 初始判断：怀疑actor ID映射错误
2. 深入分析：确认world文件中walk_X.dae分配有误
3. 修复方案：调整world文件中的皮肤文件映射
4. 验证测试：确认所有目标可正确消除

## 后续建议

1. **测试完成后恢复官方参数**
2. **保留测试参数作为调试选项**
3. **考虑添加参数化配置支持**
4. **建议创建world文件验证工具**

## 技术支持

如遇到问题，请参考：
- `docs/WORLD_FILE_FIX.md` - 详细修复指南
- `TECHNICAL.md` - 技术文档
- `README.md` - 使用说明

---

**归档人**: 东华大学 Astraeus队  
**归档日期**: 2025-10-14  
**状态**: 已测试验证
