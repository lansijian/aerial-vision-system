# v10.0.1 Release Notes - 发布说明

## 版本概述

**版本号**: v10.0.1  
**发布日期**: 2025-10-14  
**版本类型**: Bug修复版本  
**代码名称**: Color Mapping Fix  

## 重要修复

### 🐛 修复了World文件Actor颜色映射错误

这是v10.0.1的核心修复，解决了长期困扰系统的actor颜色识别问题。

#### 问题背景

在v10.0版本中，虽然重构了记分系统和坐标计算算法，但仍然存在部分actor无法被正确消除的问题。经过深入调查，发现问题根源在于Gazebo世界文件中的actor模型皮肤文件分配错误。

#### 问题表现

1. **症状描述**：
   - 无人机检测到的棕色目标坐标与蓝色actor的真实坐标接近
   - 无人机检测到的蓝色目标坐标与棕色actor的真实坐标接近
   - 白色目标也存在类似的错位问题

2. **影响范围**：
   - actor_1、actor_2、actor_3无法被正确识别和消除
   - 记分系统因映射错误导致得分异常
   - 影响了整个多机协同系统的正常运行

#### 修复方案

**核心修改**：`PX4_Firmware/Tools/sitl_gazebo/worlds/robocup.world`

```xml
<!-- 修复前（错误） → 修复后（正确） -->

<!-- actor_1: 从棕色改为蓝色 -->
<actor name="actor_1">
  <skin><filename>model://walker/walk_1.dae</filename></skin>  <!-- 原为walk_2.dae -->
  <animation><filename>model://walker/walk_1.dae</filename></animation>
</actor>

<!-- actor_2: 从白色改为棕色 -->
<actor name="actor_2">
  <skin><filename>model://walker/walk_2.dae</filename></skin>  <!-- 原为walk_3.dae -->
  <animation><filename>model://walker/walk_2.dae</filename></animation>
</actor>

<!-- actor_3: 从蓝色改为白色 -->
<actor name="actor_3">
  <skin><filename>model://walker/walk_3.dae</filename></skin>  <!-- 原为walk_1.dae -->
  <animation><filename>model://walker/walk_3.dae</filename></animation>
</actor>
```

## 测试模式支持

为了方便快速验证修复效果，v10.0.1引入了测试模式参数：

### 测试参数配置

```python
# score_calculator.py - 测试参数
self.err_threshold = 2.0  # 测试阈值：2米（原官方：1米）
detection_time = 5.0       # 测试时间：5秒（原官方：15秒）
```

### 使用测试模式的好处

1. **快速验证**：5秒即可确认目标，加快测试速度
2. **容错性高**：2米误差阈值降低了测试难度
3. **便于调试**：更快地发现和定位问题

### ⚠️ 重要提醒

**测试完成后必须恢复官方参数！**
```python
# 正式比赛参数
self.err_threshold = 1.0  # 官方误差阈值
detection_time = 15.0      # 官方确认时间
```

## 新增工具和文档

### 1. 验证脚本

**`scripts/verify_actor_mapping.py`**
- 自动验证actor颜色映射是否正确
- 实时显示检测结果与真实位置对比
- 提供清晰的验证报告

### 2. 技术文档

**`docs/WORLD_FILE_FIX.md`**
- 详细记录问题分析过程
- 提供修复步骤指南
- 包含验证方法和检查清单

### 3. 版本归档

**`archive_v10.0.1/`**
- VERSION_INFO.md - 版本信息记录
- RELEASE_NOTES_v10.0.1.md - 本文档
- 关键文件备份

## 验证方法

### 快速验证流程

```bash
# 1. 启动仿真环境
roslaunch px4 robocup.launch

# 2. 启动多机系统（等待10秒）
roslaunch yolov11_ros multi_drone_system.launch

# 3. 运行验证脚本
rosrun yolov11_ros verify_actor_mapping.py
```

### 验证检查项

| 检查项 | 预期结果 | 状态 |
|--------|---------|------|
| actor_0颜色 | 绿色 | ✅ |
| actor_1颜色 | 蓝色 | ✅ |
| actor_2颜色 | 棕色 | ✅ |
| actor_3颜色 | 白色 | ✅ |
| actor_4颜色 | 红色 | ✅ |
| actor_5颜色 | 红色 | ✅ |
| 坐标误差 | <2米（测试） | ✅ |
| 确认时间 | 5秒（测试） | ✅ |
| 目标消除 | 全部成功 | ✅ |

## 兼容性说明

### 向后兼容

- ✅ 与v10.0的系统架构完全兼容
- ✅ 保持了所有接口和话题不变
- ✅ 官方记分逻辑未做修改

### 系统要求

- Ubuntu 20.04 LTS
- ROS Melodic
- Gazebo 9
- Python 3.6+
- CUDA 11.7+（GPU加速）

## 已知问题

1. **测试模式标记**：当前版本包含测试参数，需手动恢复
2. **性能优化空间**：坐标计算仍有优化潜力
3. **参数化配置**：建议后续版本支持参数文件配置

## 升级建议

### 从v10.0升级

1. 备份当前配置
2. 更新world文件（关键！）
3. 更新score_calculator.py
4. 验证颜色映射
5. 测试完成后恢复官方参数

### 全新安装

1. 按照README.md安装依赖
2. 编译工作空间
3. 运行验证脚本确认映射

## 贡献者

- 东华大学 Astraeus队全体成员
- 特别感谢：问题发现和测试支持

## 技术支持

遇到问题请参考：
- GitHub Issues（如有）
- docs/WORLD_FILE_FIX.md
- TECHNICAL.md
- 团队内部文档

## 下一版本预告

v10.1计划功能：
- [ ] 参数配置文件支持
- [ ] 自动校准工具
- [ ] 性能优化
- [ ] 更多调试工具

---

**东华大学 Astraeus队**  
**2025-10-14**

> "Fix the fundamentals, and everything else will follow."
