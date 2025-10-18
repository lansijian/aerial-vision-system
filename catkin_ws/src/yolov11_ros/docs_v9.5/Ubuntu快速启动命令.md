# Ubuntu快速启动命令

## 每次启动前的准备

```bash
# 1. 进入脚本目录
cd ~/catkin_ws/src/yolov11_ros/scripts

# 2. 修复换行符和权限（如果是从Windows复制过来的）
sed -i 's/\r$//' *.py
chmod +x *.py

# 3. 激活conda环境（如果使用conda）
conda activate yolov11
```

## 启动系统

### 终端1 - 启动仿真
```bash
roslaunch px4 robocup.launch
```

等待Gazebo完全加载，看到两架无人机出现（约10秒）

### 终端2 - 启动多机系统
```bash
roslaunch yolov11_ros multi_drone_system.launch
```

## 监控和调试

### 终端3 - 监控话题（可选）
```bash
# 查看所有话题
rostopic list

# 监控YOLO检测
rostopic echo /yolo_detector/drone_0/detections

# 监控飞行模式
rostopic echo /drone_0/flight_mode

# 监控记分系统
rostopic echo /actor_blue_info
```

## 常见问题快速修复

### 1. Python脚本错误
```bash
# 修复所有Python脚本
cd ~/catkin_ws/src/yolov11_ros/scripts
for f in *.py; do sed -i 's/\r$//' "$f" && chmod +x "$f"; done
```

### 2. 权重文件检查
```bash
# 确认权重文件存在
ls ~/catkin_ws/src/yolov11_ros/weights/best.pt
```

### 3. 查看日志
```bash
# 查看最新日志
roscd && cd log/latest
ls -la
```

## 关闭系统

```bash
# 在每个终端按 Ctrl+C
# 或者使用
rosnode kill -a
killall -9 gzserver gzclient
```

---
最后更新：2025-10-12
