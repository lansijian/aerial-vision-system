# 安装配置指南

本指南将指导您从零开始配置RoboCup 2025多无人机协同追踪系统的完整开发环境。

## 📋 系统要求

### 硬件要求

| 组件 | 最低配置 | 推荐配置 |
|------|---------|---------|
| **CPU** | Intel i5 (4核) | Intel i7/i9 (8核+) |
| **内存** | 8 GB | 16 GB+ |
| **GPU** | 集成显卡 | NVIDIA GTX 1060+ |
| **存储** | 50 GB | 100 GB+ SSD |

### 软件要求

- **操作系统**: Ubuntu 18.04 LTS 或 20.04 LTS
- **ROS版本**: 
  - Ubuntu 18.04 → ROS Melodic
  - Ubuntu 20.04 → ROS Noetic
- **Python**: 3.8 或更高
- **CUDA**: 11.0+ (如使用GPU加速)

## 🚀 完整安装流程

### 步骤 1: 安装Ubuntu系统

如果您已有Ubuntu系统，跳过此步骤。

```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装基础工具
sudo apt install -y \
    git \
    vim \
    curl \
    wget \
    build-essential \
    software-properties-common
```

### 步骤 2: 安装ROS

#### Ubuntu 18.04 (ROS Melodic)

```bash
# 设置sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 添加密钥
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# 安装ROS Melodic完整版
sudo apt update
sudo apt install -y ros-melodic-desktop-full

# 初始化rosdep
sudo rosdep init
rosdep update

# 配置环境
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Ubuntu 20.04 (ROS Noetic)

```bash
# 类似步骤，将'melodic'替换为'noetic'
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 步骤 3: 安装ROS依赖包

```bash
# MAVROS及其依赖
sudo apt install -y \
    ros-$ROS_DISTRO-mavros \
    ros-$ROS_DISTRO-mavros-extras

# 下载GeographicLib数据集
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh

# Gazebo ROS包
sudo apt install -y \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-gazebo-ros-control

# 图像处理
sudo apt install -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-vision-opencv \
    ros-$ROS_DISTRO-image-transport

# 其他常用包
sudo apt install -y \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-geometry-msgs
```

### 步骤 4: 安装PX4 Autopilot

```bash
# 克隆PX4仓库
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.12.3  # 使用稳定版本

# 运行安装脚本
bash ./Tools/setup/ubuntu.sh

# 安装依赖
sudo apt install -y \
    python3-pip \
    python3-dev \
    libgstreamer1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good

# 编译PX4
make px4_sitl_default gazebo
```

**注意**: 首次编译可能需要20-30分钟。

### 步骤 5: 安装Python依赖

```bash
# 升级pip
python3 -m pip install --upgrade pip

# 安装PyTorch (CPU版本)
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# 如果有NVIDIA GPU，安装CUDA版本
# pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# 安装YOLOv11和依赖
pip3 install ultralytics opencv-python numpy

# 安装ROS Python工具
pip3 install \
    catkin_tools \
    rospkg \
    rospy \
    sensor_msgs \
    geometry_msgs \
    std_msgs \
    cv_bridge
```

### 步骤 6: 克隆项目代码

```bash
# 创建工作目录
mkdir -p ~/robocup2025
cd ~/robocup2025

# 克隆项目
git clone https://github.com/lansijian/aerial-vision-system.git
cd aerial-vision-system
```

### 步骤 7: 配置PX4_Firmware

```bash
# 项目已包含配置好的PX4_Firmware
# 只需设置环境变量
cd PX4_Firmware
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

# 建议添加到.bashrc
echo "# RoboCup 2025 PX4 Environment" >> ~/.bashrc
echo "export PX4_HOME=~/robocup2025/2025参赛项目/PX4_Firmware" >> ~/.bashrc
```

### 步骤 8: 配置XTDrone

```bash
cd ~/robocup2025/2025参赛项目/XTDrone

# 安装XTDrone依赖
pip3 install numpy matplotlib pillow
```

### 步骤 9: 编译ROS工作空间

```bash
cd ~/robocup2025/2025参赛项目/catkin_ws

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译
catkin build

# 如果catkin build不可用，使用catkin_make
# catkin_make

# 配置环境
echo "source ~/robocup2025/2025参赛项目/catkin_ws/devel/setup.bash" >> ~/.bashrc
source devel/setup.bash
```

### 步骤 10: 下载YOLO模型

```bash
cd ~/robocup2025/2025参赛项目/catkin_ws/src/yolov11_ros/weights

# 下载预训练模型
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolo11n.pt

# 如果已有自定义模型，复制到此目录
# cp /path/to/your/best.pt ./
```

## ✅ 验证安装

### 测试 1: ROS环境

```bash
# 启动roscore
roscore

# 新终端检查
rostopic list
rosnode list
```

### 测试 2: Gazebo

```bash
# 启动Gazebo
gazebo

# 应该能看到Gazebo界面
```

### 测试 3: MAVROS

```bash
# 检查MAVROS安装
rospack find mavros
rosrun mavros mavros_node --help
```

### 测试 4: PX4 SITL

```bash
cd ~/robocup2025/2025参赛项目/PX4_Firmware
make px4_sitl gazebo

# 应该能看到Gazebo中的Iris无人机
```

### 测试 5: Python依赖

```bash
python3 << EOF
import torch
import ultralytics
import cv2
import rospy
print("✅ All Python dependencies OK")
print(f"PyTorch version: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")
EOF
```

### 测试 6: 完整系统测试

参考 [QUICK_START.md](QUICK_START.md) 运行完整系统。

## 🔧 环境配置

### 创建便捷启动脚本

创建 `~/robocup2025/start_env.sh`:

```bash
#!/bin/bash

# ROS环境
source /opt/ros/$ROS_DISTRO/setup.bash

# Catkin工作空间
source ~/robocup2025/2025参赛项目/catkin_ws/devel/setup.bash

# PX4环境
cd ~/robocup2025/2025参赛项目/PX4_Firmware
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

echo "✅ RoboCup 2025 environment loaded!"
```

使用方法：

```bash
source ~/robocup2025/start_env.sh
```

### 配置GPU加速（可选）

如果有NVIDIA GPU：

```bash
# 安装CUDA驱动
ubuntu-drivers devices
sudo ubuntu-drivers autoinstall

# 重启系统
sudo reboot

# 验证CUDA
nvidia-smi

# 配置PyTorch使用GPU
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
```

## ❓ 常见问题

### Q1: rosdep init失败

```bash
# 方法1: 使用代理
sudo rosdep init

# 方法2: 手动下载
sudo mkdir -p /etc/ros/rosdep/sources.list.d/
sudo curl -o /etc/ros/rosdep/sources.list.d/20-default.list https://mirrors.tuna.tsinghua.edu.cn/rosdistro/rosdep/sources.list.d/20-default.list
```

### Q2: PX4编译失败

```bash
# 清理并重新编译
cd PX4_Firmware
make clean
make distclean
make px4_sitl_default gazebo
```

### Q3: Gazebo模型无法加载

```bash
# 设置Gazebo模型路径
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/robocup2025/2025参赛项目/PX4_Firmware/Tools/sitl_gazebo/models" >> ~/.bashrc
source ~/.bashrc
```

### Q4: Python包冲突

```bash
# 使用虚拟环境
python3 -m venv ~/robocup_venv
source ~/robocup_venv/bin/activate
pip install -r requirements.txt
```

### Q5: 内存不足

```bash
# 减少无人机数量进行测试
roslaunch yolov11_ros multi_drone_system.launch num_drones:=2

# 或者增加swap空间
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

## 📚 参考资源

- [PX4开发指南](https://dev.px4.io/master/en/)
- [ROS安装教程](http://wiki.ros.org/ROS/Installation)
- [MAVROS文档](http://wiki.ros.org/mavros)
- [Gazebo教程](http://gazebosim.org/tutorials)
- [YOLOv11文档](https://docs.ultralytics.com/)

## 🆘 获取帮助

如果遇到无法解决的问题：

1. 📖 检查错误日志：`~/.ros/log/latest/`
2. 🔍 搜索已知问题
3. 💬 联系实验室成员

---

**安装完成后，继续阅读 [QUICK_START.md](QUICK_START.md) 启动系统！**
