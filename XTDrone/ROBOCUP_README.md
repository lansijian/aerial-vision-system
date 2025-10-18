# XTDrone - RoboCup 2025

本目录为XTDrone官方仿真框架，用于多无人机协同仿真。

## 📖 关于XTDrone

XTDrone是一个基于PX4、ROS和Gazebo的无人机仿真平台，支持多机协同、视觉感知等功能。

- **官方仓库**: https://github.com/robin-shaun/XTDrone
- **文档**: https://www.yuque.com/xtdrone/manual_cn
- **许可证**: MIT License

## ⚠️ 重要说明

- 这是**官方XTDrone仓库**，未做修改
- 项目主要用作参考和工具库
- 实际使用的是我们自己开发的ROS包（catkin_ws）

## 🎯 在本项目中的角色

XTDrone主要提供：

1. **参考实现**: 多机协同的设计思路
2. **工具脚本**: 仿真辅助工具
3. **示例代码**: 学习和参考

**实际使用**: 我们基于XTDrone的思路，开发了自己的系统（`catkin_ws/src/yolov11_ros`）

## 📁 目录结构

```
XTDrone/
├── communication/          # 通信模块
├── control/               # 控制算法
├── coordination/          # 协同算法
├── indoor/               # 室内仿真
├── motion_planning/      # 路径规划
├── sensing/              # 传感器处理
└── sitl_config/          # SITL配置
```

## 🔍 相关功能对比

| 功能 | XTDrone | 本项目实现 |
|------|---------|-----------|
| **多机编队** | ✅ 提供 | ❌ 未使用 |
| **目标检测** | ❌ 无 | ✅ YOLOv11 |
| **人员追踪** | ❌ 无 | ✅ 自研算法 |
| **协同避免重复** | ❌ 无 | ✅ Coordinator |
| **航点导航** | ✅ 基础 | ✅ 增强版 |
| **云台控制** | ✅ 基础 | ✅ 追踪优化 |

## 📚 学习资源

### 推荐阅读

如果你想学习XTDrone：

1. **官方教程**: https://www.yuque.com/xtdrone/manual_cn
2. **快速开始**: `docs/1.快速开始.md`
3. **多机仿真**: `docs/2.多机仿真.md`

### 示例代码

XTDrone提供的有用示例：

```
sitl_config/
├── multi_vehicle.sh       # 多机启动脚本
└── setup_gazebo.sh        # Gazebo配置

control/
├── keyboard_control.py    # 键盘控制
└── offboard_control.py    # OFFBOARD模式控制
```

## 🔧 与本项目的集成

### 环境配置

XTDrone不需要单独配置，我们使用的是独立的系统。

### 参考使用

如果需要参考XTDrone代码：

```bash
cd XTDrone

# 查看多机启动脚本
cat sitl_config/multi_vehicle.sh

# 查看控制示例
cat control/keyboard_control.py
```

## 💡 借鉴思路

本项目从XTDrone借鉴的设计：

### 1. 模块化架构

```
XTDrone思路:
sensing → planning → control → actuator

本项目实现:
YOLOv11检测 → 人员追踪 → 飞行控制 → MAVROS
```

### 2. 多机命名空间

```python
# XTDrone方式
/uav0/mavros/...
/uav1/mavros/...

# 本项目采用
/mavros_0/...
/mavros_1/...
```

### 3. 话题设计

学习XTDrone的话题组织方式，设计了：
- `/yolo_detector/drone_X/detections`
- `/drone_X/locked_color`
- `/coordinator/locked_colors`

## 🚀 如果你想运行XTDrone

### 安装依赖

```bash
cd XTDrone
pip3 install -r requirements.txt
```

### 运行示例

```bash
# 单机键盘控制
rosrun xtdrone keyboard_control.py

# 多机编队（需要配置）
bash sitl_config/multi_vehicle.sh
```

**注意**: 这会与本项目的配置冲突，建议分开测试。

## 📖 官方文档

- [XTDrone Wiki](https://github.com/robin-shaun/XTDrone/wiki)
- [语雀文档](https://www.yuque.com/xtdrone/manual_cn)
- [视频教程](https://space.bilibili.com/1052231526)

## 🤔 为什么不直接用XTDrone？

### XTDrone的局限

1. ❌ 没有目标检测功能
2. ❌ 没有人员追踪算法
3. ❌ 没有协同避免重复机制
4. ❌ 航点规划不够灵活

### 本项目的优势

1. ✅ 集成YOLOv11实时检测
2. ✅ 智能人员追踪算法
3. ✅ 协同协调器避免重复
4. ✅ 针对RoboCup优化的航点
5. ✅ 完整的6机协同系统

## 🔄 未来可能的集成

如果需要XTDrone的某些功能：

- **编队飞行**: 可参考XTDrone的编队算法
- **VIO定位**: 可集成XTDrone的视觉定位
- **更多传感器**: 激光雷达、深度相机等

## 📄 许可证

XTDrone采用MIT许可证，可自由使用和修改。

本项目配置：
**东华大学人工智能创新实验室**

详见: [LICENSE](../LICENSE) 和 [XTDrone LICENSE](LICENSE)

## 🙏 致谢

感谢XTDrone团队提供优秀的开源仿真框架！

- **项目**: XTDrone
- **作者**: robin-shaun
- **GitHub**: https://github.com/robin-shaun/XTDrone

---

**XTDrone作为参考框架，核心功能由本项目独立实现！**
