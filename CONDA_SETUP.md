# Conda环境配置详细说明

## 📋 概述

本文档详细说明如何为YOLOv11目标检测配置conda虚拟环境。

**核心要求**：
- ✅ Python版本: **3.8.10** (统一版本，确保ROS兼容性)
- ✅ 环境名称: **yolov11**
- ✅ 根据GPU可用性选择PyTorch版本

---

## 🎯 为什么使用Conda？

### 优势
1. **版本隔离**: 不影响系统Python和ROS的Python环境
2. **依赖管理**: 精确控制库版本，避免冲突
3. **易于切换**: 可以快速切换不同项目环境
4. **跨平台**: Windows/Linux/macOS统一管理

### YOLOv11特殊需求
- 需要特定版本的PyTorch (2.0.1)
- OpenCV版本要求 (4.8.1.78)
- NumPy版本限制 (1.24.3)

---

## 🚀 完整安装流程

### 步骤 1: 安装Miniconda

#### 下载并安装

```bash
# 进入home目录
cd ~

# 下载Miniconda安装脚本
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh

# 赋予执行权限
chmod +x Miniconda3-latest-Linux-x86_64.sh

# 运行安装程序
./Miniconda3-latest-Linux-x86_64.sh
```

#### 安装过程交互

```
Do you accept the license terms? [yes|no]
>>> yes

Miniconda3 will now be installed into this location:
/home/username/miniconda3
>>> [Enter] (使用默认路径)

Do you wish the installer to initialize Miniconda3 by running conda init? [yes|no]
>>> yes
```

#### 重启终端使配置生效

```bash
# 关闭并重新打开终端，或者执行:
source ~/.bashrc

# 验证conda安装
conda --version
# 输出: conda 23.x.x
```

---

## 🔧 创建YOLOv11环境

### 步骤 2: 检查GPU可用性

**确定您的系统类型**：

```bash
# 检查是否有NVIDIA GPU
lspci | grep -i nvidia

# 如果有输出，说明有NVIDIA显卡
# 检查CUDA是否可用
nvidia-smi
```

**判断标准**：
- ✅ 有输出 → 选择**GPU版本**
- ❌ 无输出或报错 → 选择**CPU版本**
- 🔵 虚拟机 → 一般选择**CPU版本**（除非配置了GPU直通）

---

## 💻 选项A: GPU版本配置（推荐）

**适用于**: 物理机、配置了GPU直通的虚拟机、双系统

### 创建环境

```bash
# 创建环境，指定Python 3.8.10
conda create -n yolov11 python=3.8.10 -y

# 激活环境
conda activate yolov11

# 验证Python版本
python --version
# 输出: Python 3.8.10
```

### 安装PyTorch (CUDA 11.8)

```bash
# 安装GPU版本的PyTorch
pip install torch==2.0.1 torchvision==0.15.2 torchaudio==2.0.2 \
    --index-url https://download.pytorch.org/whl/cu118
```

### 验证GPU可用

```bash
python << EOF
import torch
print(f"PyTorch版本: {torch.__version__}")
print(f"CUDA可用: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"GPU型号: {torch.cuda.get_device_name(0)}")
    print(f"GPU数量: {torch.cuda.device_count()}")
EOF
```

**预期输出**：
```
PyTorch版本: 2.0.1+cu118
CUDA可用: True
GPU型号: NVIDIA GeForce RTX 3090
GPU数量: 1
```

### 安装YOLOv11和依赖

```bash
# 安装Ultralytics (YOLOv11)
pip install ultralytics==8.0.200

# 安装OpenCV
pip install opencv-python==4.8.1.78

# 安装NumPy
pip install numpy==1.24.3

# 安装ROS Python工具
pip install catkin_tools rospkg empy pyyaml

# 验证YOLOv11
python -c "from ultralytics import YOLO; model = YOLO('yolov11n.pt'); print('✅ YOLOv11 GPU版本配置成功')"
```

---

## 🖥️ 选项B: CPU版本配置

**适用于**: 虚拟机、无GPU的物理机、测试环境

### 创建环境

```bash
# 创建环境，指定Python 3.8.10
conda create -n yolov11 python=3.8.10 -y

# 激活环境
conda activate yolov11

# 验证Python版本
python --version
# 输出: Python 3.8.10
```

### 安装PyTorch (CPU版本)

```bash
# 安装CPU版本的PyTorch
pip install torch==2.0.1 torchvision==0.15.2 torchaudio==2.0.2 \
    --index-url https://download.pytorch.org/whl/cpu
```

### 验证安装

```bash
python << EOF
import torch
print(f"PyTorch版本: {torch.__version__}")
print(f"CPU版本: {'+cpu' in torch.__version__}")
EOF
```

**预期输出**：
```
PyTorch版本: 2.0.1+cpu
CPU版本: True
```

### 安装YOLOv11和依赖

```bash
# 安装Ultralytics (YOLOv11)
pip install ultralytics==8.0.200

# 安装OpenCV
pip install opencv-python==4.8.1.78

# 安装NumPy
pip install numpy==1.24.3

# 安装ROS Python工具
pip install catkin_tools rospkg empy pyyaml

# 验证YOLOv11
python -c "from ultralytics import YOLO; print('✅ YOLOv11 CPU版本配置成功')"
```

---

## ⚙️ 环境配置和使用

### 自动激活环境（推荐）

```bash
# 编辑 ~/.bashrc
echo "" >> ~/.bashrc
echo "# Auto-activate YOLOv11 conda environment" >> ~/.bashrc
echo "conda activate yolov11" >> ~/.bashrc

# 重新加载配置
source ~/.bashrc
```

**效果**: 每次打开新终端自动激活yolov11环境

### 手动激活/切换环境

```bash
# 激活yolov11环境
conda activate yolov11

# 退出环境（回到base环境）
conda deactivate

# 查看所有环境
conda env list

# 查看当前环境的包
conda list
pip list
```

---

## 📊 性能对比

### GPU vs CPU检测速度

| 配置 | 单帧检测时间 | 6机系统FPS | 适用场景 |
|------|------------|-----------|---------|
| **RTX 3090** | ~8ms | 120+ FPS | 生产环境、比赛 |
| **GTX 1660** | ~15ms | 60+ FPS | 开发、测试 |
| **CPU i7-10700** | ~45ms | 20 FPS | 基础测试 |
| **CPU i5-8400** | ~80ms | 12 FPS | 功能验证 |

**建议**:
- 🔴 比赛/演示: 必须使用GPU
- 🟡 开发调试: GPU优先，CPU可用
- 🔵 功能测试: CPU即可

---

## 🔍 常见问题

### Q1: conda命令找不到

```bash
# 方法1: 手动添加conda到PATH
export PATH=~/miniconda3/bin:$PATH
source ~/.bashrc

# 方法2: 重新运行conda init
~/miniconda3/bin/conda init bash
source ~/.bashrc
```

### Q2: CUDA版本不匹配

```bash
# 检查系统CUDA版本
nvidia-smi  # 查看右上角的CUDA Version

# 如果是CUDA 11.x，使用cu118
pip install torch==2.0.1 torchvision==0.15.2 --index-url https://download.pytorch.org/whl/cu118

# 如果是CUDA 12.x，使用cu121
pip install torch==2.0.1 torchvision==0.15.2 --index-url https://download.pytorch.org/whl/cu121
```

### Q3: torch.cuda.is_available() 返回 False

**可能原因和解决方法**:

```bash
# 1. 确认NVIDIA驱动已安装
nvidia-smi

# 2. 确认安装的是GPU版本PyTorch
python -c "import torch; print(torch.__version__)"
# 应该包含 +cu118，如果是 +cpu 则需重装

# 3. 重装GPU版本
pip uninstall torch torchvision torchaudio -y
pip install torch==2.0.1 torchvision==0.15.2 torchaudio==2.0.2 \
    --index-url https://download.pytorch.org/whl/cu118
```

### Q4: ImportError: libGL.so.1

```bash
# Ubuntu/Debian
sudo apt-get install -y libgl1-mesa-glx libglib2.0-0

# 或者安装headless版本的opencv
pip uninstall opencv-python -y
pip install opencv-python-headless==4.8.1.78
```

### Q5: YOLOv11版本冲突

```bash
# 卸载所有ultralytics相关包
pip uninstall ultralytics ultralytics-yolov5 yolov5 -y

# 重新安装指定版本
pip install ultralytics==8.0.200

# 验证版本
python -c "import ultralytics; print(ultralytics.__version__)"
```

### Q6: conda环境激活后ROS命令失效

**原因**: conda的Python可能覆盖了系统Python

**解决方法**:

```bash
# 在yolov11环境中重新source ROS
conda activate yolov11
source /opt/ros/melodic/setup.bash  # 或 noetic
source ~/robocup2025/2025参赛项目/catkin_ws/devel/setup.bash

# 验证
roscore  # 应该能正常启动
```

---

## 📝 环境导出和恢复

### 导出环境配置

```bash
# 激活环境
conda activate yolov11

# 导出环境配置
conda env export > yolov11_environment.yml

# 或者只导出pip安装的包
pip freeze > requirements.txt
```

### 在其他机器恢复环境

```bash
# 方法1: 使用conda yml文件
conda env create -f yolov11_environment.yml

# 方法2: 使用requirements.txt
conda create -n yolov11 python=3.8.10 -y
conda activate yolov11
pip install -r requirements.txt
```

---

## ✅ 验证清单

配置完成后，依次检查：

```bash
# 1. Conda环境
conda activate yolov11
python --version  # Python 3.8.10

# 2. PyTorch
python -c "import torch; print(torch.__version__)"  # 2.0.1+cu118 或 2.0.1+cpu

# 3. GPU可用性（如果是GPU版本）
python -c "import torch; print(torch.cuda.is_available())"  # True

# 4. YOLOv11
python -c "from ultralytics import YOLO; print('OK')"  # OK

# 5. OpenCV
python -c "import cv2; print(cv2.__version__)"  # 4.8.1.78

# 6. ROS工具
python -c "import rospkg; print('OK')"  # OK
```

**全部通过 = 环境配置成功！** ✨

---

## 📚 参考资源

- [Conda官方文档](https://docs.conda.io/)
- [PyTorch安装指南](https://pytorch.org/get-started/locally/)
- [Ultralytics YOLOv11](https://docs.ultralytics.com/)
- [CUDA兼容性](https://developer.nvidia.com/cuda-toolkit)

---

## 📞 获取帮助

遇到问题？

1. 检查上面的常见问题部分
2. 查看 [INSTALLATION.md](INSTALLATION.md) 完整安装指南
3. 联系项目负责人：2033374848@qq.com

---

**Copyright © 2025 陈庭宇 & 东华大学 Astraeus 队**
