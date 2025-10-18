# 快速开始指南

本指南将帮助您在5分钟内启动RoboCup 2025多无人机协同追踪系统。

## 🎯 目标

完成本指南后，您将能够：
- ✅ 在Gazebo中启动6架无人机仿真
- ✅ 运行YOLOv11目标检测
- ✅ 观察无人机自主巡逻和协同追踪

## 📋 前置条件

在开始之前，请确保已完成：

1. ✅ Ubuntu 18.04/20.04 已安装
2. ✅ ROS Melodic/Noetic 已配置
3. ✅ PX4、MAVROS、Gazebo 已安装
4. ✅ Python 3.8+ 和相关依赖已安装

> **未完成安装？** 请先查看 [INSTALLATION.md](INSTALLATION.md)

## 🚀 启动步骤

### 步骤 1: 准备环境

打开终端，设置工作目录：

```bash
cd /path/to/robocup2025/2025参赛项目
```

### 步骤 2: 启动PX4仿真

**终端 1** - 启动PX4 SITL和Gazebo：

```bash
cd PX4_Firmware
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

# 启动6架无人机
roslaunch launch/robocup.launch
```

**等待约30秒**，直到看到：
```
[INFO] [WallTime: ...] Gazebo started
[INFO] [WallTime: ...] Spawn model 'typhoon_h480_0'
...
[INFO] [WallTime: ...] Spawn model 'typhoon_h480_5'
```

### 步骤 3: 启动协同追踪系统

**终端 2** - 启动ROS节点：

```bash
cd catkin_ws
source devel/setup.bash

# 启动完整系统（6架无人机）
roslaunch yolov11_ros multi_drone_system.launch
```

> **提示**: 如果只想测试2架无人机：
> ```bash
> roslaunch yolov11_ros multi_drone_system.launch num_drones:=2
> ```

### 步骤 4: 解锁并起飞

**终端 3** - 执行起飞命令：

```bash
cd catkin_ws/src/yolov11_ros/scripts

# 方法1: 使用提供的快速启动脚本
python3 quick_takeoff.py

# 方法2: 手动发送MAVROS命令
rosrun mavros mavsafety arm
rosrun mavros mavsys mode -c OFFBOARD
```

### 步骤 5: 观察系统运行

无人机将自动开始：
1. 📍 **起飞** - 升至3米高度
2. 🗺️ **巡逻** - 按预设航点自主飞行
3. 👀 **检测** - YOLOv11实时检测行人
4. 🎯 **追踪** - 发现目标后自动追踪
5. 🤝 **协同** - 避免重复追踪同一目标

## 👁️ 监控系统状态

### 查看检测结果

**终端 4** - 监听YOLO检测话题：

```bash
# 查看无人机0的检测结果
rostopic echo /yolo_detector/drone_0/detections

# 查看所有无人机的锁定状态
rostopic echo /coordinator/locked_colors
```

### 查看图像输出

使用rqt查看实时检测画面：

```bash
rqt_image_view
```

在下拉菜单中选择：
- `/typhoon_h480_0/cgo3_camera/image_raw` - 原始图像
- `/yolo_detector/drone_0/image_detections` - 带检测框的图像

### 查看无人机位置

```bash
# 查看无人机0的位置
rostopic echo /mavros_0/local_position/pose

# 查看所有话题
rostopic list | grep mavros
```

## 🎮 控制无人机

### 手动控制（可选）

如果需要手动控制无人机：

```bash
# 发送位置命令
rostopic pub /mavros_0/setpoint_position/local geometry_msgs/PoseStamped \
  "{header: {stamp: now, frame_id: 'map'}, 
    pose: {position: {x: 0.0, y: 0.0, z: 3.0}}}"
```

### 切换追踪目标

系统会自动分配追踪目标，但您可以查看当前状态：

```bash
# 查看协调器日志
rosnode info /simple_coordinator
```

## 🛑 停止系统

按照以下顺序停止：

1. **终端 2**: 按 `Ctrl+C` 停止ROS节点
2. **终端 1**: 按 `Ctrl+C` 停止Gazebo
3. 清理进程：
   ```bash
   killall -9 gazebo gzserver gzclient px4
   ```

## ❓ 常见问题

### Q1: Gazebo启动后黑屏

**解决方法**:
```bash
# 重置Gazebo配置
rm -rf ~/.gazebo/models/*
cd PX4_Firmware/Tools/sitl_gazebo
./setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
```

### Q2: MAVROS连接失败

**症状**: `connected: false`

**解决方法**:
```bash
# 检查端口配置
netstat -tulpn | grep 14540
netstat -tulpn | grep 14557

# 确保PX4 SITL正在运行
ps aux | grep px4
```

### Q3: YOLO模型加载失败

**症状**: `Model weights not found`

**解决方法**:
```bash
cd catkin_ws/src/yolov11_ros/weights
# 确保yolo11n.pt存在
ls -lh yolo11n.pt

# 如果不存在，下载模型
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolo11n.pt
```

### Q4: 无人机不起飞

**检查清单**:
1. ✅ PX4 SITL是否成功启动？
2. ✅ MAVROS是否已连接？ (`rostopic echo /mavros_0/state`)
3. ✅ 是否发送了解锁命令？
4. ✅ 是否设置了OFFBOARD模式？

### Q5: 无人机撞到建筑物

**原因**: 航点规划可能不适合当前地图

**解决方法**:
```bash
# 使用经过验证的安全航点
cd catkin_ws/src/yolov11_ros/scripts/waypoints
# 检查waypoints_drone_X.json文件
```

## 📊 性能优化

### 提高检测速度

编辑 `multi_drone_system.launch`:
```xml
<param name="model_name" value="yolo11n.pt"/>  <!-- 最快 -->
<!-- <param name="model_name" value="yolo11s.pt"/> --> <!-- 平衡 -->
<!-- <param name="model_name" value="yolo11m.pt"/> --> <!-- 最准确 -->
```

### 降低CPU使用率

减少无人机数量：
```bash
roslaunch yolov11_ros multi_drone_system.launch num_drones:=3
```

### GPU加速

确保PyTorch使用GPU：
```bash
python3 -c "import torch; print(torch.cuda.is_available())"
# 应该输出: True
```

## 📚 下一步

- 📖 阅读 [系统架构文档](catkin_ws/src/yolov11_ros/TECHNICAL.md)
- 🛠️ 查看 [调试指南](catkin_ws/src/yolov11_ros/DEBUG_GUIDE.md)
- 🗺️ 了解 [航点规划](docs/安全航点规划总结.md)
- 🤝 学习 [协同机制](docs/简化版协同追踪系统说明.md)

## 🆘 获取帮助

如果遇到问题：

1. 📖 查看 [DEBUG_GUIDE.md](catkin_ws/src/yolov11_ros/DEBUG_GUIDE.md)
2. 🔍 搜索 [已知问题](docs/)
3. 💬 联系实验室成员

## 🎓 学习资源

- [PX4开发指南](https://dev.px4.io/)
- [ROS教程](http://wiki.ros.org/ROS/Tutorials)
- [YOLOv11文档](https://docs.ultralytics.com/)
- [Gazebo教程](http://gazebosim.org/tutorials)

---

**祝您使用愉快！🚁**

*有问题？查看 [完整文档索引](README.md#-文档索引)*
