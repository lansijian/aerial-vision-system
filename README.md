# RoboCup 2025 - 多无人机协同人员追踪系统

[![License](https://img.shields.io/badge/License-DHU--AIIL-blue.svg)](LICENSE)
[![ROS](https://img.shields.io/badge/ROS-Melodic-green.svg)](http://wiki.ros.org/melodic)
[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![YOLOv11](https://img.shields.io/badge/YOLOv11-Ultralytics-orange.svg)](https://github.com/ultralytics/ultralytics)

## ⚠️ 必读！经验教训

**下届参赛者请务必先阅读：[LESSONS_LEARNED.md](LESSONS_LEARNED.md)**

### 🔴 关键教训

1. **Docker镜像导出是第一优先级** - 本次失败的主要原因
2. **禁止使用虚拟机开发** - 必须使用双系统或实验室服务器
3. **从第一天就开始Docker** - 不要等到最后

详见：[完整经验教训文档](LESSONS_LEARNED.md)

---

## 📋 项目简介

本项目是东华大学人工智能创新实验室为RoboCup 2025机器人世界杯救援仿真赛开发的多无人机协同人员追踪系统。系统实现了6架无人机在城市环境中自主巡逻、检测并协同追踪不同颜色衣服的行人目标。

### 核心特性

- 🚁 **6架无人机协同作战**：基于PX4飞控和Gazebo仿真环境
- 🎯 **YOLOv11目标检测**：实时识别不同颜色衣服的行人
- 🤝 **智能协同追踪**：避免多机追踪同一目标，提高搜救效率
- 🗺️ **安全航点规划**：覆盖16种地图的通用航点设计
- 📡 **ROS通信架构**：模块化设计，易于扩展和维护

### 系统架构

```
系统架构图
┌─────────────────────────────────────────────────────────┐
│                      Gazebo仿真环境                       │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐     ┌─────────┐ │
│  │  UAV 0  │  │  UAV 1  │  │  UAV 2  │ ... │  UAV 5  │ │
│  └────┬────┘  └────┬────┘  └────┬────┘     └────┬────┘ │
└───────┼───────────┼────────────┼───────────────┼───────┘
        │           │            │               │
        │      PX4 SITL + MAVROS                 │
        └───────────┼────────────┼───────────────┘
                    │            │
        ┌───────────┴────────────┴───────────────┐
        │            ROS节点层                    │
        │  ┌──────────────┐  ┌────────────────┐ │
        │  │  YOLOv11检测 │  │  航点导航       │ │
        │  │  (6个节点)   │  │  (6个节点)     │ │
        │  └──────┬───────┘  └────────┬───────┘ │
        │         │                    │         │
        │  ┌──────┴────────┐  ┌───────┴──────┐  │
        │  │  人员追踪     │  │  云台控制     │  │
        │  │  (6个节点)   │  │  (6个节点)   │  │
        │  └──────┬────────┘  └──────────────┘  │
        │         │                              │
        │  ┌──────┴────────────────────────┐    │
        │  │  协同协调器(Simple Coordinator) │    │
        │  └───────────────────────────────┘    │
        └─────────────────────────────────────┘
```

## 📁 项目结构

```
2025参赛项目/
├── catkin_ws/                    # ROS工作空间（核心开发代码）
│   ├── src/
│   │   ├── yolov11_ros/         # 主要功能包
│   │   │   ├── scripts/         # Python脚本
│   │   │   │   ├── drone_controller.py      # 无人机控制器
│   │   │   │   ├── waypoint_navigator.py    # 航点导航
│   │   │   │   ├── human_tracker.py         # 人员追踪
│   │   │   │   ├── gimbal_control.py        # 云台控制
│   │   │   │   ├── simple_coordinator.py    # 协同协调器
│   │   │   │   └── waypoints/               # 航点文件
│   │   │   ├── launch/                      # 启动文件
│   │   │   │   └── multi_drone_system.launch
│   │   │   ├── weights/         # YOLO模型权重
│   │   │   └── docs/            # 详细文档
│   │   ├── actor_collisions/    # 行人碰撞检测
│   │   └── gazebo_ros_pkgs/     # Gazebo ROS插件
│   └── build/                   # 编译输出
│
├── PX4_Firmware/                 # PX4飞控固件（官方）
│   ├── launch/
│   │   └── robocup.launch       # 多无人机启动配置
│   └── Tools/sitl_gazebo/       # Gazebo模型
│
├── XTDrone/                      # XTDrone框架（官方）
│   └── ...                      # 无人机仿真工具包
│
├── Mydataset/                    # 训练数据集
│   ├── data/                    # 原始数据
│   ├── yolo11s_custom/          # 自定义数据集
│   ├── train_yolov11.py         # 训练脚本
│   └── Myvoc.yaml               # 数据配置
│
├── yolo11n_auto_annotated/       # 自动标注数据
│   └── ...                      # 标注后的图像数据
│
├── runs/                         # 训练结果
│   └── detect/                  # 检测模型输出
│
├── launch(1)/                    # 比赛官方文档
│   └── launch/                  # 官方启动文件参考
│
├── docs/                         # 项目文档
│   ├── 6机协同配置说明.md
│   ├── 安全航点规划总结.md
│   ├── 简化版协同追踪系统说明.md
│   └── ...
│
├── README.md                     # 本文件
├── LICENSE                       # 许可证
├── QUICK_START.md               # 快速开始指南
└── .gitignore                   # Git忽略文件
```

## 🚀 快速开始

### 系统要求

- **操作系统**: Ubuntu 20.04
- **ROS版本**: ROS Noetic
- **Python**: 3.8+
- **GPU**: NVIDIA GPU（推荐，用于YOLO推理）
- **依赖**:
  - PX4 Autopilot
  - Gazebo 9+
  - MAVROS
  - OpenCV
  - PyTorch
  - Ultralytics YOLOv11

### 安装步骤

详细的安装和配置指南请参考：[INSTALLATION.md](INSTALLATION.md)

```bash
# 1. 克隆项目
git clone https://github.com/lansijian/aerial-vision-system.git
cd aerial-vision-system

# 2. 安装依赖
cd catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. 编译工作空间
catkin build

# 4. 配置环境
source devel/setup.bash
```

### 运行系统

```bash
# 终端1: 启动PX4 SITL + Gazebo仿真
cd PX4_Firmware
roslaunch launch/robocup.launch

# 终端2: 启动多无人机协同系统
cd catkin_ws
source devel/setup.bash
roslaunch yolov11_ros multi_drone_system.launch
```

详细使用说明请查看：[QUICK_START.md](QUICK_START.md)

## 📊 系统性能

- **检测速度**: ~30 FPS（YOLOv11n）
- **追踪精度**: 95%+
- **协同响应时间**: <500ms
- **支持地图数量**: 16个官方地图
- **同时追踪目标**: 最多6个（非红色）

## 📖 文档索引

| 文档 | 描述 |
|------|------|
| [⚠️ 经验教训](LESSONS_LEARNED.md) | **必读！Docker和开发环境经验** ⭐⭐⭐⭐⭐ |
| [🚀 下届参赛方向](下届参赛方向推荐.md) | **2026年技术升级建议** ⭐⭐⭐⭐⭐ |
| [快速启动指南](QUICK_START.md) | 5分钟快速上手 |
| [安装配置指南](INSTALLATION.md) | 详细安装步骤 |
| [系统架构文档](catkin_ws/src/yolov11_ros/TECHNICAL.md) | 技术实现细节 |
| [航点规划说明](docs/安全航点规划总结.md) | 航点设计原理 |
| [协同追踪机制](docs/简化版协同追踪系统说明.md) | 多机协同算法 |
| [故障排除指南](catkin_ws/src/yolov11_ros/DEBUG_GUIDE.md) | 常见问题解决 |

## 🎯 比赛成绩

- **赛题**: RoboCup 2025 Rescue Simulation League
- **任务**: 城市环境中的人员搜索与救援
- **团队**: 东华大学人工智能创新实验室

## 🤝 贡献指南

由于本项目仅供今年赛题参考使用，我们暂不接受外部贡献。

内部团队成员请遵循以下规范：
1. 所有修改需要经过代码审查
2. 提交信息使用规范格式
3. 更新相关文档

## 📄 版权声明

**Copyright © 2025 东华大学人工智能创新实验室 (DHU AI Innovation Lab)**

**All Rights Reserved - 保留所有权利**

### 许可说明

本项目**仅供以下用途**：
- ✅ 东华大学人工智能创新实验室内部学习和研究
- ✅ 2025年RoboCup参赛相关用途
- ✅ 实验室成员学术研究（需注明出处）

### 使用限制

**严格禁止**以下行为：
- ❌ 未经许可的商业使用
- ❌ 代码的二次分发或转让
- ❌ 用于其他竞赛或项目（未经实验室授权）
- ❌ 删除或修改版权声明

### 引用要求

如需使用本项目代码或成果，必须按以下格式标注来源：

```
本项目使用了东华大学人工智能创新实验室开发的
RoboCup 2025多无人机协同追踪系统
项目地址: https://github.com/lansijian/aerial-vision-system
Copyright © 2025 DHU AI Innovation Lab
```

### 第三方组件

本项目使用了以下开源组件：
- PX4 Autopilot (BSD License)
- ROS (BSD License)
- YOLOv11 by Ultralytics (AGPL-3.0)
- XTDrone (MIT License)

这些组件保留其原有许可证。

### 联系方式

如有授权或合作需求，请联系：
- **实验室**: 东华大学人工智能创新实验室
- **飞书**: 实验室内部飞书群（仅实验室成员可访问）
- **项目维护**: 通过飞书群联系项目负责人

---

**本软件按"原样"提供，不提供任何形式的明示或暗示担保。**

## 👥 团队成员

**东华大学人工智能创新实验室**
- 项目负责人: [请填写]
- 核心开发: [请填写]
- 算法优化: [请填写]

## 🙏 致谢

感谢以下开源项目和组织：
- RoboCup救援仿真联盟
- PX4开源飞控项目
- Ultralytics YOLO团队
- ROS社区

## 📞 联系我们

- **实验室**: 东华大学人工智能创新实验室
- **飞书群**: 内部成员专用（加入请联系实验室）
- **GitHub**: [个人维护，地址见上]
- **注意**: 本项目无公开GitHub组织，由个人维护

---

**Made with ❤️ by DHU AI Innovation Lab**

*最后更新: 2025年10月*
