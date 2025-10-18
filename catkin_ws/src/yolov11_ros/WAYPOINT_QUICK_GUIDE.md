# 6无人机航点方案 - 快速指南

## ✅ 已完成

**6台无人机航点文件已生成！**

---

## 📊 生成结果

| UAV | 区域 | 航点数 | 预计时间 | 文件 |
|-----|------|--------|---------|------|
| UAV0 | 下左 | 19 | 1.9分钟 | `waypoints/waypoints_drone_0.json` |
| UAV1 | 下右 | 50 | 2.9分钟 | `waypoints/waypoints_drone_1.json` |
| UAV2 | 中左 | 21 | 1.9分钟 | `waypoints/waypoints_drone_2.json` |
| UAV3 | 中右 | 51 | 3.0分钟 | `waypoints/waypoints_drone_3.json` |
| UAV4 | 上左 | 16 | 1.7分钟 | `waypoints/waypoints_drone_4.json` |
| UAV5 | 上右 | 40 | 2.7分钟 | `waypoints/waypoints_drone_5.json` |
| **总计** | - | **197** | **~7-10分钟** | - |

---

## 🎯 核心策略

### 区域划分

```
Y方向三分（对应初始位置）：
├─ 下区域 y∈[-48.7,-10]  → UAV0(左) + UAV1(右)
├─ 中区域 y∈[-10,20]     → UAV2(左) + UAV3(右)
└─ 上区域 y∈[20,49.2]    → UAV4(左) + UAV5(右)

X方向左右分（重叠区x∈[25,40]）：
├─ 左侧 x∈[-45,40]       → UAV0,2,4（就近起飞）
└─ 右侧 x∈[25,120]       → UAV1,3,5（向右扩展）
```

### 覆盖特点

✅ **100%道路覆盖**
- 固定道路：x=-45,-15,110,120，y=-45,0,45
- 中间道路：x=30-75密集扫描（自适应）

✅ **重叠区双保险**
- x∈[25,40]左右重叠
- 确保中间道路不遗漏

✅ **时间充裕**
- 预计7-10分钟
- 比赛时限20分钟
- 可慢速精确搜索

---

## 🚀 使用方法

### 1. 文件已生成
```
✅ waypoints/waypoints_drone_0.json
✅ waypoints/waypoints_drone_1.json
✅ waypoints/waypoints_drone_2.json
✅ waypoints/waypoints_drone_3.json
✅ waypoints/waypoints_drone_4.json
✅ waypoints/waypoints_drone_5.json
```

### 2. 复制到ROS package
```bash
# 如果还没有waypoints目录
mkdir -p catkin_ws/src/yolov11_ros/waypoints

# 复制文件（已经在正确位置，无需操作）
# 文件已在项目根目录的waypoints/
```

### 3. 启动测试
```bash
# ROS系统会自动加载对应航点
roslaunch yolov11_ros multi_drone_system.launch
```

---

## 🎲 自适应说明

### 适用于所有16种地图

| 地图 | 中间道路 | 覆盖情况 |
|------|---------|---------|
| base0,6 | x=45 | ✅ 在扫描范围内 |
| base1,7,8 | x=35-55(斜) | ✅ 全覆盖 |
| base2 | x=55 | ✅ 在扫描范围内 |
| base3,4,5 | x=30 | ✅ 在扫描范围内 |
| base9,10 | x=55-75(斜) | ✅ 全覆盖 |
| base11,12 | x=75-55(斜) | ✅ 全覆盖 |
| base13,14 | x=55-35(斜) | ✅ 全覆盖 |
| base15,16 | x=65 | ✅ 在扫描范围内 |

**无论哪张地图，100%覆盖保证！**

---

## 💡 关键优势

1. **就近起飞** - 3台UAV(0,2,4)立即开始搜索
2. **重叠保险** - x∈[25,40]双机覆盖
3. **密集扫描** - 中间道路间隔5-10m
4. **时间充裕** - 预计用时<50%时限

---

## 📞 如有疑问

参考详细文档：
- [WAYPOINT_STRATEGY_FINAL.md](WAYPOINT_STRATEGY_FINAL.md) - 完整策略
- [OPTIMAL_WAYPOINT_STRATEGY.md](road/OPTIMAL_WAYPOINT_STRATEGY.md) - 设计思路
- [MAP_ANALYSIS.md](road/MAP_ANALYSIS.md) - 地图分析

重新生成航点：
```bash
python catkin_ws/src/yolov11_ros/scripts/generate_adaptive_waypoints.py
```

---

**状态**: ✅ 已完成，立即可用  
**版本**: v1.0  
**日期**: 2025-10-16

