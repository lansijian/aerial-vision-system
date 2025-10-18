# 训练数据集

本目录包含RoboCup 2025项目的YOLOv11训练数据集和训练脚本。

## 📁 目录结构

```
Mydataset/
├── data/                    # 处理后的训练数据
│   ├── images/             # 图像文件
│   │   ├── train/          # 训练集图像
│   │   └── val/            # 验证集图像
│   └── labels/             # YOLO格式标注
│       ├── train/          # 训练集标注
│       └── val/            # 验证集标注
│
├── original_dataset/        # 原始数据集
│   ├── images/             # 原始图像
│   └── annotations/        # 原始标注（XML格式）
│
├── yolo11s_custom/          # 自定义训练配置
│   ├── args.yaml           # 训练参数
│   ├── data.yaml           # 数据配置
│   └── ...
│
├── runs/                    # 训练结果
│   └── detect/             # 检测任务结果
│       └── train/          # 训练输出
│           ├── weights/    # 模型权重
│           │   ├── best.pt    # 最佳模型
│           │   └── last.pt    # 最后一个epoch
│           ├── results.png    # 训练曲线
│           └── confusion_matrix.png
│
├── train_yolov11.py         # 训练脚本
├── splitDataset.py          # 数据集划分脚本
├── json2xml.py              # JSON转XML工具
├── xml2txt.py               # XML转YOLO格式工具
├── Myvoc.yaml               # VOC格式配置
├── yolo11s.pt               # 预训练模型
└── README.md                # 本文件
```

## 🎯 数据集说明

### 类别定义

数据集包含以下类别（基于衣服颜色的行人检测）：

```python
classes = {
    0: 'red_person',      # 红色衣服的行人
    1: 'green_person',    # 绿色衣服的行人
    2: 'blue_person',     # 蓝色衣服的行人
    3: 'yellow_person',   # 黄色衣服的行人
    4: 'orange_person',   # 橙色衣服的行人
    5: 'purple_person',   # 紫色衣服的行人
}
```

### 数据集统计

- **训练集**: 约 XXX 张图像
- **验证集**: 约 XXX 张图像
- **标注格式**: YOLO (txt)
- **图像格式**: JPG/PNG
- **分辨率**: 640x640 (训练时resize)

## 🚀 快速开始

### 1. 准备数据

如果你有新的原始数据：

```bash
# 1. 将图像放入 original_dataset/images/
# 2. 将XML标注放入 original_dataset/annotations/

# 3. 转换标注格式
python3 xml2txt.py

# 4. 划分训练集和验证集
python3 splitDataset.py
```

### 2. 训练模型

```bash
# 使用默认配置训练
python3 train_yolov11.py

# 或自定义参数
python3 train_yolov11.py \
    --model yolo11s.pt \
    --data Myvoc.yaml \
    --epochs 100 \
    --batch 16 \
    --imgsz 640
```

### 3. 查看结果

训练完成后，查看结果：

```bash
cd runs/detect/train/

# 查看训练曲线
xdg-open results.png

# 查看混淆矩阵
xdg-open confusion_matrix.png

# 最佳模型
ls -lh weights/best.pt
```

### 4. 使用训练好的模型

```bash
# 复制到ROS包
cp runs/detect/train/weights/best.pt \
   ../catkin_ws/src/yolov11_ros/weights/yolo11s_custom.pt
```

## 📝 脚本说明

### train_yolov11.py

主训练脚本，支持自定义参数。

**关键参数**:

```python
model = YOLO('yolo11s.pt')  # 预训练模型
results = model.train(
    data='Myvoc.yaml',      # 数据配置
    epochs=100,             # 训练轮数
    imgsz=640,              # 图像大小
    batch=16,               # 批大小
    device='0',             # GPU设备（'cpu' for CPU）
    project='runs/detect',  # 输出目录
    name='train',           # 实验名称
    patience=50,            # 早停耐心值
    save=True,              # 保存模型
    plots=True,             # 生成图表
)
```

### splitDataset.py

将数据集划分为训练集和验证集。

**默认比例**: 80% 训练，20% 验证

```bash
python3 splitDataset.py
```

### xml2txt.py

将VOC格式的XML标注转换为YOLO格式的txt文件。

**输入**: `original_dataset/annotations/*.xml`  
**输出**: `data/labels/`

```bash
python3 xml2txt.py
```

### json2xml.py

将JSON格式标注转换为XML格式（如果需要）。

```bash
python3 json2xml.py
```

## 🎓 数据增强

训练脚本自动应用以下数据增强：

- **Mosaic**: 图像拼接
- **MixUp**: 图像混合
- **Random Flip**: 随机翻转
- **Random Scale**: 随机缩放
- **Color Jitter**: 颜色抖动
- **HSV Augmentation**: HSV色彩空间增强

在 `train_yolov11.py` 中调整增强参数：

```python
results = model.train(
    ...
    hsv_h=0.015,        # 色调增强
    hsv_s=0.7,          # 饱和度增强
    hsv_v=0.4,          # 明度增强
    degrees=0.0,        # 旋转角度
    translate=0.1,      # 平移
    scale=0.5,          # 缩放
    mosaic=1.0,         # Mosaic概率
    mixup=0.0,          # MixUp概率
)
```

## 📊 模型评估

### 评估已训练模型

```python
from ultralytics import YOLO

# 加载模型
model = YOLO('runs/detect/train/weights/best.pt')

# 评估
metrics = model.val(data='Myvoc.yaml')

print(f"mAP50: {metrics.box.map50}")
print(f"mAP50-95: {metrics.box.map}")
```

### 测试单张图像

```python
model = YOLO('runs/detect/train/weights/best.pt')
results = model.predict('test_image.jpg', save=True)
```

## 🔧 配置文件

### Myvoc.yaml

数据集配置文件：

```yaml
# 路径配置
path: /path/to/Mydataset  # 数据集根目录
train: data/images/train  # 训练集路径
val: data/images/val      # 验证集路径

# 类别配置
nc: 6  # 类别数量
names:
  0: red_person
  1: green_person
  2: blue_person
  3: yellow_person
  4: orange_person
  5: purple_person
```

## 📈 训练监控

### 使用TensorBoard

```bash
# 安装tensorboard
pip3 install tensorboard

# 启动TensorBoard
tensorboard --logdir runs/detect/train

# 浏览器访问: http://localhost:6006
```

### 实时查看训练日志

```bash
tail -f runs/detect/train/train.log
```

## 🎯 性能优化

### 提高训练速度

1. **使用更小的模型**: `yolo11n.pt` (最快) vs `yolo11s.pt` vs `yolo11m.pt`
2. **减小batch size**: 如果显存不足
3. **减小图像大小**: `imgsz=416` 或 `imgsz=320`
4. **使用GPU**: `device='0'`

### 提高检测精度

1. **更多训练数据**: 增加数据量
2. **更好的标注**: 检查标注质量
3. **更多训练轮数**: 增加 `epochs`
4. **更大的模型**: 使用 `yolo11m.pt` 或 `yolo11l.pt`
5. **调整超参数**: 学习率、动量等

## ❓ 常见问题

### Q: CUDA out of memory

```bash
# 减小batch size
python3 train_yolov11.py --batch 8

# 或使用CPU
python3 train_yolov11.py --device cpu
```

### Q: 训练太慢

```bash
# 使用更小的模型
python3 train_yolov11.py --model yolo11n.pt

# 减小图像大小
python3 train_yolov11.py --imgsz 416
```

### Q: mAP很低

1. 检查数据集标注是否正确
2. 增加训练轮数
3. 调整学习率
4. 增加训练数据

### Q: 模型过拟合

```bash
# 增加数据增强
python3 train_yolov11.py --augment

# 使用dropout和正则化
# （在配置文件中调整）
```

## 📚 参考资源

- [YOLOv11官方文档](https://docs.ultralytics.com/)
- [数据标注指南](https://github.com/ultralytics/yolov5/wiki/Train-Custom-Data)
- [超参数调优](https://docs.ultralytics.com/modes/train/#augmentation-settings)

## 🤝 贡献数据

如果要添加新的训练数据：

1. 确保图像质量良好
2. 标注准确完整
3. 遵循现有的类别定义
4. 运行数据验证脚本

## 📄 许可证

版权所有 © 2025 东华大学人工智能创新实验室

详见: [LICENSE](../LICENSE)

---

**训练数据为内部使用，请勿外传！**
