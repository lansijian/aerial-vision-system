# 安装配置指南

本指南将指导您从零开始配置多旋翼无人机集群协同搜索仿真系统的完整开发环境。

## ⚠️ 重要说明

**本文档中的命令仅供参考，实际安装请以官方文档为准！**

### 📚 官方文档链接（必读）

- **XTDrone使用文档**: [https://www.yuque.com/xtdrone/manual_cn](https://www.yuque.com/xtdrone/manual_cn)
- **PX4 1.13版本一键安装脚本**: [https://www.yuque.com/xtdrone/manual_cn/px4_1.13_installation](https://www.yuque.com/xtdrone/manual_cn/px4_1.13_installation)

### 📖 本仓库文档说明

仓库中包含的PX4_Firmware和XTDrone文档**仅作为参考**，主要包含：
- ✅ 针对2025年赛题的特殊改动（**重点：typhoon_h480机型的云台改动**）
- ✅ 项目特定的配置说明
- ⚠️ **环境安装请使用上述官方文档，不要直接使用仓库中的安装说明**

### 🔧 关键配置要求

- **ROS工作空间编译**: 必须使用 `catkin build` 命令
- **PX4版本**: PX4 1.13（使用官方一键安装脚本）
- **Python环境**: 使用Conda虚拟环境，Python 3.8.10

---

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

### 步骤 4: 安装PX4 Autopilot（使用官方一键安装脚本）

**⚠️ 重要：请使用XTDrone官方提供的PX4 1.13版本一键安装脚本！**

#### 推荐方法：使用官方一键安装脚本

访问官方文档并按照说明操作：
- 📘 [PX4 1.13版本一键安装脚本（Beta测试版）](https://www.yuque.com/xtdrone/manual_cn/px4_1.13_installation)

**优势**：
- ✅ 自动配置所有依赖
- ✅ 版本兼容性保证
- ✅ 避免常见安装错误
- ✅ 节省时间（相比手动配置）

#### 参考命令（仅供参考，以官方文档为准）

```bash
# 以下命令仅供参考，实际操作请参考官方文档

# 克隆PX4仓库
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.13.3  # PX4 1.13版本

# 运行官方一键安装脚本
bash ./Tools/setup/ubuntu.sh

# 编译PX4 SITL
make px4_sitl_default gazebo
```

**注意**: 
- 首次编译可能需要20-30分钟
- 具体步骤和参数以官方文档为准
- 如遇到问题，请查阅XTDrone官方文档的FAQ部分

### 步骤 5: 安装Conda并配置YOLOv11环境

#### 5.1 安装Miniconda

```bash
# 下载Miniconda安装脚本
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh

# 安装Miniconda
bash Miniconda3-latest-Linux-x86_64.sh

# 按照提示完成安装，建议安装在默认位置: ~/miniconda3
# 安装完成后重启终端或执行:
source ~/.bashrc

# 验证安装
conda --version
```

#### 5.2 创建YOLOv11专用环境 (Python 3.8.10)

**根据您的系统选择：**

##### 选项A: 有GPU的系统（推荐，显著提升检测速度）

```bash
# 创建conda环境，指定Python 3.8.10
conda create -n yolov11 python=3.8.10 -y

# 激活环境
conda activate yolov11

# 安装PyTorch with CUDA支持 (CUDA 11.8)
pip install torch==2.0.1 torchvision==0.15.2 torchaudio==2.0.2 --index-url https://download.pytorch.org/whl/cu118

# 验证GPU可用
python -c "import torch; print(f'CUDA Available: {torch.cuda.is_available()}'); print(f'GPU Name: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else \"N/A\"}')"
```

##### 选项B: 无GPU的系统（虚拟机/双系统CPU版本）

```bash
# 创建conda环境，指定Python 3.8.10
conda create -n yolov11 python=3.8.10 -y

# 激活环境
conda activate yolov11

# 安装PyTorch CPU版本
pip install torch==2.0.1 torchvision==0.15.2 torchaudio==2.0.2 --index-url https://download.pytorch.org/whl/cpu

# 验证安装
python -c "import torch; print(f'PyTorch Version: {torch.__version__}')"
```

#### 5.3 安装YOLOv11和ROS依赖

```bash
# 确保在yolov11环境中
conda activate yolov11

# 安装YOLOv11和计算机视觉依赖
pip install ultralytics==8.0.200
pip install opencv-python==4.8.1.78
pip install numpy==1.24.3

# 安装ROS Python工具
pip install \
    catkin_tools \
    rospkg \
    empy \
    pyyaml

# 验证安装
python -c "from ultralytics import YOLO; print('✅ YOLOv11 安装成功')"
```

#### 5.4 配置环境自动激活

```bash
# 添加到 ~/.bashrc，每次打开终端自动激活yolov11环境
echo "" >> ~/.bashrc
echo "# Auto-activate YOLOv11 conda environment" >> ~/.bashrc
echo "conda activate yolov11" >> ~/.bashrc

# 重新加载配置
source ~/.bashrc
```

**重要提示**:
- 🔴 **统一使用Python 3.8.10**：确保与ROS的兼容性
- 🟢 **GPU版本**：检测速度快3-5倍，适合多无人机系统
- 🟡 **CPU版本**：适合测试和开发，实时性较差
- 🔵 **虚拟机用户**：建议使用CPU版本，虚拟机GPU直通配置复杂

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

**参考官方文档**: [XTDrone使用文档](https://www.yuque.com/xtdrone/manual_cn)

```bash
cd ~/robocup2025/2025参赛项目/XTDrone

# 安装XTDrone依赖（以官方文档为准）
pip3 install numpy matplotlib pillow
```

**注意**: 
- 仓库中的XTDrone文档主要包含针对2025年赛题的特殊改动
- 完整的安装和使用说明请参考XTDrone官方文档

### 步骤 9: 编译ROS工作空间（必须使用catkin build）

**⚠️ 重要：本项目必须使用 `catkin build` 编译，不能使用 `catkin_make`！**

```bash
cd ~/robocup2025/2025参赛项目/catkin_ws

# 确保在yolov11 conda环境中
conda activate yolov11

# 安装catkin_tools（如果未安装）
pip install catkin_tools

# 安装ROS依赖
rosdep install --from-paths src --ignore-src -r -y

# 使用catkin build编译（必须）
catkin build

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
