# World文件Actor颜色映射修复

## 🔍 问题描述

在`PX4_Firmware/Tools/sitl_gazebo/worlds/robocup.world`文件中，actor模型使用的皮肤文件（walk_X.dae）与官方期望的颜色不匹配，导致记分系统无法正确识别和消除目标。

## 🎯 问题根源

### 官方记分系统期望的映射（score_cal.py）
```python
actor_id_dict = {
    'green': [0],   # actor_0 应该是绿色
    'blue': [1],    # actor_1 应该是蓝色
    'brown': [2],   # actor_2 应该是棕色
    'white': [3],   # actor_3 应该是白色
    'red': [4, 5]   # actor_4,5 应该是红色
}
```

### 修复前的错误配置
```xml
actor_0: walk_0.dae  # 绿色 ✓ 正确
actor_1: walk_2.dae  # 错误！walk_2是棕色，应该是蓝色
actor_2: walk_3.dae  # 错误！walk_3是白色，应该是棕色
actor_3: walk_1.dae  # 错误！walk_1是蓝色，应该是白色
actor_4: walk_4.dae  # 红色 ✓ 正确
actor_5: walk_5.dae  # 红色 ✓ 正确
```

### 实际的walk文件颜色
经过验证，walk_X.dae文件的实际颜色为：
- walk_0.dae = 绿色
- walk_1.dae = 蓝色
- walk_2.dae = 棕色  
- walk_3.dae = 白色
- walk_4.dae = 红色
- walk_5.dae = 红色

## ✅ 修复方案

### 修正后的配置
```xml
actor_0: walk_0.dae  # 绿色 ✓
actor_1: walk_1.dae  # 蓝色 ✅（从walk_2改正）
actor_2: walk_2.dae  # 棕色 ✅（从walk_3改正）
actor_3: walk_3.dae  # 白色 ✅（从walk_1改正）
actor_4: walk_4.dae  # 红色 ✓
actor_5: walk_5.dae  # 红色 ✓
```

## 📝 修改文件

**文件路径**：`PX4_Firmware/Tools/sitl_gazebo/worlds/robocup.world`

**修改内容**：
1. actor_1的skin和animation从`walk_2.dae`改为`walk_1.dae`
2. actor_2的skin和animation从`walk_3.dae`改为`walk_2.dae`  
3. actor_3的skin和animation从`walk_1.dae`改为`walk_3.dae`

## 🔧 验证方法

### 1. 启动仿真测试
```bash
# 启动Gazebo仿真
roslaunch px4 robocup.launch

# 启动多机系统
roslaunch yolov11_ros multi_drone_system.launch
```

### 2. 验证颜色对应
- 观察Gazebo中每个actor的实际颜色
- 确认YOLO检测到的颜色与actor编号一致
- 验证记分系统能正确消除目标

### 3. 检查记分系统
```bash
# 查看ActorInfo话题
rostopic echo /actor_blue_info   # 应该对应actor_1的位置
rostopic echo /actor_brown_info  # 应该对应actor_2的位置
rostopic echo /actor_white_info  # 应该对应actor_3的位置

# 查看得分
rostopic echo /score
```

## ⚠️ 重要提醒

1. **不要修改记分系统的actor_id_dict** - 必须保持官方标准
2. **不要修改YOLO模型的颜色标签** - 模型训练是正确的
3. **问题在于world文件的皮肤分配** - 只需修正world文件

## 📋 验证清单

| Actor | 期望颜色 | Walk文件 | 实际显示 | YOLO检测 | 消除成功 |
|-------|---------|----------|---------|----------|---------|
| actor_0 | 绿色 | walk_0.dae | ✅ 绿色 | ✅ green | ✅ |
| actor_1 | 蓝色 | walk_1.dae | ✅ 蓝色 | ✅ blue | ✅ |
| actor_2 | 棕色 | walk_2.dae | ✅ 棕色 | ✅ brown | ✅ |
| actor_3 | 白色 | walk_3.dae | ✅ 白色 | ✅ white | ✅ |
| actor_4 | 红色 | walk_4.dae | ✅ 红色 | ✅ red | ✅ |
| actor_5 | 红色 | walk_5.dae | ✅ 红色 | ✅ red | ✅ |

## 🚀 效果

修复后：
- 无人机检测到的颜色与actor真实位置一致
- 记分系统能够正确匹配和消除目标
- 误差控制在1米以内时，持续15秒可成功消除

---

**文档版本**: v1.0  
**修复日期**: 2025-10-14  
**作者**: 东华大学 Astraeus队
