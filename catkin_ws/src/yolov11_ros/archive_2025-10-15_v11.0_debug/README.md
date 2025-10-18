# v11.0/v11.1 调试归档 - 2025-10-15

## 归档原因
v11.0模块化架构过于复杂，导致大量问题，浪费了一整天时间调试。

## 主要问题
1. 航点控制方向反向（倒着飞）
2. 追踪控制飞太高、剧烈抖动
3. 模块间通信混乱
4. 文档过多无用

## 最终方案（v11.1）
1. **航点控制**：waypoint_mission直接发布PositionTarget到MAVROS
2. **追踪控制**：target_tracker发布Twist给mission_controller转发
3. **模式切换**：mission_controller负责发布flight_mode
4. **XTDrone算法**：完全复刻官方yolo_human_tracking.py

## 关键修复
- mission_controller发布flight_mode给其他模块
- waypoint停止时target开始，互不干扰
- 简化追踪算法，让检测框保持在图像中心

## 教训
1. 不要过度设计
2. 参考成熟版本（v9.2, v10.1.2）
3. 保持简洁
4. 快速验证

---
归档日期：2025-10-15  
东华大学 Astraeus队

