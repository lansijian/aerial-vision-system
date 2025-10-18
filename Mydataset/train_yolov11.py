#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLOv11 训练脚本
专门针对 YOLOv11 版本设计，使用正确的参数格式
"""

import os
import sys
import torch
from ultralytics import YOLO

def check_environment():
    """检查训练环境"""
    print("=== 环境检查 ===")
    print(f"Python版本: {sys.version}")
    print(f"PyTorch版本: {torch.__version__}")
    print(f"CUDA可用: {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"GPU设备: {torch.cuda.get_device_name(0)}")
        print(f"CUDA版本: {torch.version.cuda}")
    print()

def check_files():
    """检查必要的文件是否存在"""
    print("=== 文件检查 ===")
    
    # 配置文件路径
    config_path = os.path.join(os.path.dirname(__file__), 'Myvoc.yaml')
    if not os.path.exists(config_path):
        print(f"❌ 配置文件不存在: {config_path}")
        return False
    print(f"✅ 配置文件存在: {config_path}")
    
    # 模型文件路径
    model_path = os.path.join(os.path.dirname(__file__), 'yolo11s.pt')
    if not os.path.exists(model_path):
        print(f"❌ 预训练模型不存在: {model_path}")
        return False
    print(f"✅ 预训练模型存在: {model_path}")
    
    # 检查数据文件
    data_dir = os.path.join(os.path.dirname(__file__), 'data')
    if not os.path.exists(data_dir):
        print(f"❌ 数据目录不存在: {data_dir}")
        return False
    print(f"✅ 数据目录存在: {data_dir}")
    
    # 检查训练/验证/测试文件
    train_file = os.path.join(data_dir, 'train.txt')
    val_file = os.path.join(data_dir, 'val.txt')
    test_file = os.path.join(data_dir, 'test.txt')
    
    for file_path, file_name in [(train_file, 'train.txt'), 
                                (val_file, 'val.txt'), 
                                (test_file, 'test.txt')]:
        if os.path.exists(file_path):
            print(f"✅ {file_name} 存在")
        else:
            print(f"⚠️  {file_name} 不存在")
    
    print()
    return True

def train_yolov11():
    """训练 YOLOv11 模型"""
    print("=== 开始 YOLOv11 训练 ===")
    
    # 获取当前脚本所在目录
    base_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(base_dir, 'Myvoc.yaml')
    model_path = os.path.join(base_dir, 'yolo11s.pt')
    
    # 检查设备
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"使用设备: {device}")
    
    try:
        # 加载预训练模型
        print("加载预训练模型...")
        model = YOLO(model_path)
        
        # YOLOv11 训练参数配置
        training_config = {
            'data': config_path,           # 数据集配置文件
            'epochs': 100,                 # 训练轮数
            'imgsz': 640,                  # 输入图像尺寸
            'batch': 16,                   # 批次大小
            'device': device,              # 训练设备
            'workers': 4,                  # 数据加载工作进程数
            'patience': 50,                # 早停耐心值
            'save': True,                  # 保存训练结果
            'save_period': 10,             # 每10个epoch保存一次
            'cache': True,                 # 缓存数据
            'name': 'yolo11s_custom',      # 训练结果保存名称
            'exist_ok': True,              # 允许覆盖已有结果
            'pretrained': True,            # 使用预训练权重
            'optimizer': 'auto',           # 优化器自动选择
            'lr0': 0.01,                   # 初始学习率
            'lrf': 0.01,                   # 最终学习率
            'momentum': 0.937,             # 动量
            'weight_decay': 0.0005,        # 权重衰减
            'warmup_epochs': 3.0,          # 预热轮数
            'warmup_momentum': 0.8,        # 预热动量
            'warmup_bias_lr': 0.1,         # 预热偏置学习率
            'box': 7.5,                    # 边框损失权重
            'cls': 0.5,                    # 分类损失权重
            'dfl': 1.5,                    # 分布焦点损失权重
            'val': True,                   # 训练时验证
            'plots': True,                 # 生成图表
            'verbose': True,               # 详细输出
        }
        
        print("训练参数配置:")
        for key, value in training_config.items():
            print(f"  {key}: {value}")
        print()
        
        # 开始训练
        print("开始训练...")
        results = model.train(**training_config)
        
        print("✅ 训练完成!")
        print(f"训练结果保存在: {model.trainer.save_dir}")
        
        return True
        
    except Exception as e:
        print(f"❌ 训练过程中发生错误: {e}")
        print("错误类型:", type(e).__name__)
        import traceback
        traceback.print_exc()
        return False

def main():
    """主函数"""
    print("=" * 50)
    print("YOLOv11 训练脚本")
    print("=" * 50)
    
    # 检查环境
    check_environment()
    
    # 检查文件
    if not check_files():
        print("❌ 文件检查失败，请确保所有必要文件都存在")
        return
    
    # 确认是否开始训练
    print("确认开始训练? (y/n): ")
    choice = input().strip().lower()
    
    if choice != 'y':
        print("训练已取消")
        return
    
    # 开始训练
    success = train_yolov11()
    
    if success:
        print("🎉 训练成功完成!")
    else:
        print("❌ 训练失败")

if __name__ == "__main__":
    main()