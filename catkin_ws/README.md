# Catkin工作空间

这是RoboCup 2025项目的ROS工作空间，包含所有核心功能包和自定义节点。

## 📁 目录结构

```
catkin_ws/
├── src/                          # 源代码包
│   ├── yolov11_ros/             # 主要功能包（核心）
│   ├── actor_collisions/        # 行人碰撞检测
│   ├── gazebo_ros_actor_plugin/ # Gazebo行人插件
│   └── gazebo_ros_pkgs/         # Gazebo ROS接口
├── build/                       # 编译输出（不提交）
├── devel/                       # 开发环境（不提交）
└── logs/                        # 编译日志（不提交）
```

## 🎯 主要功能包

### 1. yolov11_ros

**核心功能包**，包含所有主要功能：

- 🔍 **YOLOv11目标检测**: 实时行人检测和颜色识别
- 🚁 **无人机控制**: 基于MAVROS的飞行控制
- 🗺️ **航点导航**: 自主巡逻和路径规划
- 🎯 **人员追踪**: 智能追踪算法
- 🎮 **云台控制**: 相机云台控制
- 🤝 **协同协调**: 多机协同避免重复追踪

详细说明: [yolov11_ros/README.md](src/yolov11_ros/README.md)

### 2. actor_collisions

行人碰撞检测插件，用于在仿真中检测无人机与行人的碰撞。

### 3. gazebo_ros_actor_plugin

Gazebo仿真中的行人运动控制插件。

### 4. gazebo_ros_pkgs

Gazebo与ROS的接口包。

## 🔧 编译

### 首次编译

```bash
cd catkin_ws

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译（推荐使用catkin build）
catkin build

# 或使用catkin_make
# catkin_make

# 配置环境
source devel/setup.bash
```

### 单独编译某个包

```bash
# 使用catkin build
catkin build yolov11_ros

# 使用catkin_make
catkin_make --pkg yolov11_ros
```

### 清理编译

```bash
# 清理所有编译文件
catkin clean

# 或手动删除
rm -rf build devel logs
```

## 📦 依赖包

### ROS依赖

- `roscpp`
- `rospy`
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `nav_msgs`
- `mavros`
- `mavros_msgs`
- `cv_bridge`
- `image_transport`
- `tf2_ros`

### Python依赖

```bash
pip3 install \
    numpy \
    opencv-python \
    torch \
    torchvision \
    ultralytics \
    matplotlib
```

## 🚀 运行

### 启动完整系统

```bash
# 配置环境
source devel/setup.bash

# 启动多无人机系统
roslaunch yolov11_ros multi_drone_system.launch
```

### 启动单个节点

```bash
# YOLO检测节点
rosrun yolov11_ros yolo_detector_node.py

# 无人机控制器
rosrun yolov11_ros drone_controller.py

# 航点导航
rosrun yolov11_ros waypoint_navigator.py

# 人员追踪
rosrun yolov11_ros human_tracker.py
```

## 📝 开发指南

### 添加新功能包

```bash
cd src
catkin_create_pkg my_package rospy roscpp std_msgs
```

### 修改代码后

```bash
# 重新编译
catkin build

# 配置环境
source devel/setup.bash
```

### Python脚本权限

```bash
# 确保Python脚本可执行
chmod +x src/yolov11_ros/scripts/*.py
```

## 🧪 测试

### 运行测试

```bash
# 运行所有测试
catkin run_tests

# 运行特定包的测试
catkin run_tests yolov11_ros
```

### 代码检查

```bash
# Python代码检查
pylint src/yolov11_ros/scripts/*.py

# 或使用flake8
flake8 src/yolov11_ros/scripts/
```

## 📊 话题和服务

### 重要话题

```bash
# 检测结果
/yolo_detector/drone_0/detections

# 图像话题
/typhoon_h480_0/cgo3_camera/image_raw

# 无人机状态
/mavros_0/state
/mavros_0/local_position/pose

# 协同协调
/coordinator/locked_colors
/drone_0/locked_color
```

### 查看话题

```bash
# 列出所有话题
rostopic list

# 查看话题数据
rostopic echo /yolo_detector/drone_0/detections

# 查看话题频率
rostopic hz /typhoon_h480_0/cgo3_camera/image_raw
```

## 🔍 调试

### 查看日志

```bash
# 实时查看rosout
rostopic echo /rosout

# 查看节点日志
roscd yolov11_ros
cd ../../logs/latest/
```

### 使用rqt工具

```bash
# 节点图
rqt_graph

# 图像查看
rqt_image_view

# 控制台
rqt_console
```

## 📚 相关文档

- [yolov11_ros详细文档](src/yolov11_ros/README.md)
- [系统架构说明](src/yolov11_ros/TECHNICAL.md)
- [调试指南](src/yolov11_ros/DEBUG_GUIDE.md)
- [快速开始](../QUICK_START.md)

## ❓ 常见问题

### Q: 编译失败

```bash
# 清理并重新编译
catkin clean
catkin build
```

### Q: 找不到包

```bash
# 确保已配置环境
source devel/setup.bash

# 检查包是否存在
rospack find yolov11_ros
```

### Q: Python导入错误

```bash
# 检查Python路径
echo $PYTHONPATH

# 重新配置环境
source devel/setup.bash
```

## 🤝 贡献

仅限实验室内部成员贡献，请遵循：

1. 修改前先创建新分支
2. 代码需要通过测试
3. 提交前进行代码审查

## 📄 许可证

版权所有 © 2025 东华大学人工智能创新实验室

详见: [LICENSE](../LICENSE)
