# 技术文档

## 目录
1. [系统架构](#系统架构)
2. [坐标系说明](#坐标系说明)
3. [参数配置](#参数配置)
4. [常见问题与解决方案](#常见问题与解决方案)
5. [性能优化建议](#性能优化建议)

本文档记录系统的技术细节、深度配置和关键技术实现。

---

## 最新更新 (2025-10-10)

### v8.0 - 简洁PID控制重构

1. **完全重写追踪控制**
   - 删除所有复杂控制方法（_compute_velocity_control、_compute_simple_control、_compute_visual_servo_control）
   - 实现单一简洁的`_compute_clean_tracking_control`方法
   - 代码量减少70%，参数减少80%

2. **PID控制算法**
   - 横向控制：Kp=0.003, Ki=0.0002, Kd=0.0008
   - 距离控制：基于检测框面积的P控制
   - 偏航控制：最小化，仅在偏差>80px时启用
   
3. **性能提升**
   - 响应时间：0.3-0.5s → 0.1-0.2s
   - 最大速度：前后3.0m/s，横向2.0m/s
   - 稳定性大幅提升，解决了原地打转问题

### 纯视觉伺服控制集成（v7.0，已被v8.0替代）

1. **控制算法升级**: 从plan3_pure_visual_servo.py集成核心算法
   - 基于box_size的PD控制器
   - 自适应距离和速度控制
   - 多级滤波和动态alpha选择
   - 急刹保护机制

2. **优势**:
   - 更智能的控制: PD控制器能更好地响应目标距离和速度变化
   - 更稳定的跟踪: 多级滤波（5帧加权平均 + 动态alpha滤波）
   - 避免坐标系问题: 不依赖复杂坐标变换，直接使用视觉反馈

3. **参数配置**:
   - `use_visual_servo`: 是否启用纯视觉伺服（默认true）
   - `ideal_box_size`: 理想追踪距离对应的box_size（默认2400 px²）
   - `K1`: 距离误差增益（默认2.5）
   - `K2`: 速度变化增益（默认20.0）
   - `max_forward_speed`: 最大前进速度（默认4.5 m/s）

---

## 之前更新 (2025-10-09)

### 颜色目标检测系统改进

1. **YOLO模型升级**: 从人体检测改为基于颜色标签的目标检测
   - 支持5种颜色: blue, green, white, brown, red
   - 模型权重文件: `weights/best.pt`（需要替换为颜色检测模型）

2. **记分系统与官方兼容**:
   - 误差阈值: 2.0m → 1.0m（与官方一致）
   - 检测时间: 8.0s → 15.0s（与官方一致）
   - 传感器成本计算完全采用官方公式
   - 红色目标特殊处理（两个独立回调函数）

3. **多机协调升级**:
   - 基于颜色的目标分配
   - 防止多机追踪同一颜色目标
   - 追踪声明包含颜色信息

---

## 目录

1. [PX4 1.13多机云台配置](#px4-113多机云台配置)
2. [OFFBOARD模式保持机制](#offboard模式保持机制)
3. [分布式协调算法](#分布式协调算法)
4. [记分系统架构](#记分系统架构)
5. [6机扩展指南](#6机扩展指南)
6. [关键参数详解](#关键参数详解)
7. [故障排查技术](#故障排查技术)

---

## PX4 1.13多机云台配置

### 问题根源

**PX4 1.11 → 1.13 云台UDP端口计算方式变更**

| 版本 | 端口计算规则 | 多机支持 |
|------|-------------|---------|
| PX4 1.11 | 固定13030 | ✅ 天然支持（共享端口） |
| PX4 1.13 | 13030 + instance | ❌ 需要独立配置 |

**端口映射表**:
| 无人机ID | PX4监听端口 | SDF配置端口 | 状态 |
|---------|-----------|------------|------|
| 0 | 13030 | 13030 | ✅ 匹配 |
| 1 | 13031 | 13031 | ✅ 匹配 |
| 2 | 13032 | 13032 | ✅ 匹配 |
| 3 | 13033 | 13033 | ✅ 匹配 |
| 4 | 13034 | 13034 | ✅ 匹配 |
| 5 | 13035 | 13035 | ✅ 匹配 |

### 解决方案

#### 1. 创建独立SDF模型

```bash
cd ~/PX4_Firmware/Tools/sitl_gazebo/models
for i in 0 1 2 3 4 5; do
    cp -r typhoon_h480 typhoon_h480_$i
done
```

**目录结构**:
```
models/
├── typhoon_h480/          原始模型（保留参考）
├── typhoon_h480_0/        实例0：端口13030
├── typhoon_h480_1/        实例1：端口13031
├── typhoon_h480_2/        实例2：端口13032
├── typhoon_h480_3/        实例3：端口13033
├── typhoon_h480_4/        实例4：端口13034
└── typhoon_h480_5/        实例5：端口13035
```

#### 2. 修改SDF文件（13处/文件）

每个 `typhoon_h480_X.sdf` 需修改：

1. **Model名称**（第3行）:
```xml
<model name='typhoon_h480_0'>  <!-- 对应实例编号 -->
```

2. **IMU Link**（第674行）:
```xml
<link name='typhoon_h480_0/imu_link'>
```

3. **IMU Joint**（第689-690行）:
```xml
<joint name='typhoon_h480_0/imu_joint' type='revolute'>
    <child>typhoon_h480_0/imu_link</child>
```

4. **Sonar Parent**（第1147行）:
```xml
<parent>typhoon_h480_0::base_link</parent>
```

5. **云台Joint引用**（第1369, 1379, 1389行）:
```xml
<joint_name>typhoon_h480_0::cgo3_camera_joint</joint_name>
<joint_name>typhoon_h480_0::cgo3_camera_joint</joint_name>
<joint_name>typhoon_h480_0::cgo3_vertical_arm_joint</joint_name>
```

6. **IMU Plugin**（第1432行）:
```xml
<linkName>typhoon_h480_0/imu_link</linkName>
```

7. **UDP端口**（第1444行）⭐最关键:
```xml
<!-- typhoon_h480_0 -->
<udp_gimbal_port_remote>13030</udp_gimbal_port_remote>

<!-- typhoon_h480_1 -->
<udp_gimbal_port_remote>13031</udp_gimbal_port_remote>

<!-- 依次递增... -->
```

8. **云台控制器Joint**（第1445-1447行）:
```xml
<joint_yaw>typhoon_h480_0::cgo3_vertical_arm_joint</joint_yaw>
<joint_roll>typhoon_h480_0::cgo3_horizontal_arm_joint</joint_roll>
<joint_pitch>typhoon_h480_0::cgo3_camera_joint</joint_pitch>
```

#### 3. 修改model.config（2处/文件）

```xml
<?xml version="1.0"?>
<model>
  <name>Typhoon H480 RS - Instance 0</name>
  <version>1.0</version>
  <sdf version="1.5">typhoon_h480_0.sdf</sdf>  <!-- 引用正确文件名 -->
  <author>...</author>
  <description>...</description>
</model>
```

#### 4. 修改robocup.launch ⭐关键

**必须显式传递 `udp_gimbal_port` 参数**:

```xml
<!-- 无人机0 -->
<group ns="typhoon_h480_0">
    <arg name="ID" value="0"/>
    <include file="$(find px4)/launch/single_vehicle_spawn_xtd.launch">
        <arg name="vehicle" value="typhoon_h480"/>
        <arg name="sdf" value="typhoon_h480_0"/>
        <arg name="udp_gimbal_port" value="13030"/>  <!-- ⭐必须显式传递 -->
        <arg name="ID" value="0"/>
        ...
    </include>
</group>

<!-- 无人机1 -->
<group ns="typhoon_h480_1">
    <arg name="ID" value="1"/>
    <include file="$(find px4)/launch/single_vehicle_spawn_xtd.launch">
        <arg name="vehicle" value="typhoon_h480"/>
        <arg name="sdf" value="typhoon_h480_1"/>
        <arg name="udp_gimbal_port" value="13031"/>  <!-- ⭐必须显式传递 -->
        <arg name="ID" value="1"/>
        ...
    </include>
</group>
```

**关键发现**: `single_vehicle_spawn_xtd.launch` 使用 xmlstarlet 动态修改SDF文件中的端口配置。如果不传递 `udp_gimbal_port` 参数，所有无人机都会使用默认值13030，导致端口冲突。

### 验证方法

```bash
# 1. 检查Gazebo模型加载
rosservice call /gazebo/get_world_properties

# 应看到:
# model_names: [typhoon_h480_0, typhoon_h480_1]

# 2. 检查MAVROS连接
rostopic echo /typhoon_h480_0/mavros/state -n 1
rostopic echo /typhoon_h480_1/mavros/state -n 1

# 应显示: connected: True

# 3. 检查云台控制频率
rostopic hz /typhoon_h480_0/mavros/mount_control/command
rostopic hz /typhoon_h480_1/mavros/mount_control/command

# 应约80Hz

# 4. 检查相机图像
rostopic hz /typhoon_h480_0/cgo3_camera/image_raw
rostopic hz /typhoon_h480_1/cgo3_camera/image_raw

# 应有10-15Hz
```

---

## 坐标系说明

### 坐标系定义

1. **世界坐标系 (ENU)**
   - East (X) - 东向
   - North (Y) - 北向  
   - Up (Z) - 上向
   - 右手坐标系

2. **机体坐标系 (FLU)**
   - Forward (X) - 前向
   - Left (Y) - 左向
   - Up (Z) - 上向

3. **相机坐标系**
   - 右 (X) - 图像右方向
   - 下 (Y) - 图像下方向
   - 前 (Z) - 光轴方向

### 坐标计算方案（精确版）

```python
# 1. 水平偏移计算（使用相机内参）
pixel_offset = center_u - u_center
horizontal_angle = pixel_offset / focal_length  # 弧度
horizontal_offset = horizontal_angle * distance  # 米

# 2. 前方距离（考虑相机俯仰角）
camera_pitch = -45°  # 相机向下倾斜
forward_distance = distance * cos(camera_pitch)

# 3. 相机物理偏移（来自typhoon_h480.sdf）
camera_offset_x = -0.041  # 向后4.1cm
camera_offset_z = -0.162  # 向下16.2cm

# 4. 目标在机体坐标系中的位置
target_body_x = forward_distance + camera_offset_x
target_body_y = horizontal_offset

# 5. 考虑偏航角的世界坐标
target_x = drone_x + cos(yaw) * target_body_x - sin(yaw) * target_body_y
target_y = drone_y + sin(yaw) * target_body_x + cos(yaw) * target_body_y
```

### 关键参数
- 相机焦距：fx = 205.47（来自相机内参）
- 相机俯仰角：-45°（默认值）
- 相机偏移：X=-0.041m, Z=-0.162m（来自模型文件）

---

## 参数配置

### 追踪控制参数

| 参数名 | 默认值 | 说明 | 推荐范围 |
|--------|--------|------|----------|
| `Kp_xy` | 0.5 | 水平控制增益 | 0.3-0.7 |
| `Kp_z` | 1.0 | 垂直控制增益 | 0.8-1.2 |
| `max_vel` | 2.0 | 最大水平速度(m/s) | 1.0-3.0 |
| `max_vel_z` | 1.0 | 最大垂直速度(m/s) | 0.5-1.5 |
| `control_rate_hz` | 50 | 控制频率(Hz) | 20-50 |
| `detection_confidence` | 0.3 | 检测置信度阈值 | 0.2-0.5 |

### 避障参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `obstacle_avoidance_enabled` | true | 启用避障 |
| `min_obstacle_distance` | 1.5 | 最小障碍物距离(m) |
| `avoidance_strength` | 1.0 | 避障强度系数 |

### 记分系统参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `err_threshold` | 1.0 | 位置误差阈值(m) |
| `detection_time` | 15.0 | 稳定检测时间(s) |
| `height_ui_alarm_min` | 2.5 | 最低高度警告(m) |
| `height_ui_alarm_max` | 5.5 | 最高高度警告(m) |

---

## 常见问题与解决方案

### 1. 追踪抖动问题

**症状**: 无人机在追踪时不停抖动  
**原因**: 控制增益过高，缺少死区和滤波  
**解决**:
- 降低`Kp_xy`到0.35
- 启用死区（15像素）
- 启用低通滤波（α=0.6）

### 2. 坐标偏差问题

**症状**: 记分面板显示坐标偏差很大  
**原因**: 坐标计算未考虑偏航角  
**解决**: 使用简化坐标计算方案（参考单无人机）

### 3. ActorInfo消息错误

**症状**: `AttributeError: 'ActorInfo' object has no attribute 'z'`  
**解决**: 代码已添加try-except兼容处理

### 4. 避障不工作

**症状**: 飞行中声纳避障无响应  
**原因**: 避障逻辑未集成到速度控制  
**解决**: 在`_compute_velocity_control`中添加避障逻辑

---

## 性能优化建议

### 1. 降低系统负载
```bash
# 降低YOLO推理频率
<param name="inference_rate" value="10"/>  # 10Hz instead of 30Hz

# 降低控制频率
<param name="control_rate_hz" value="30"/>  # 30Hz instead of 50Hz
```

### 2. 优化追踪性能
- 使用简化的坐标计算
- 减少不必要的坐标变换
- 启用运动预测和滤波

### 3. 提高稳定性
- 使用合适的控制增益
- 添加速度限制
- 实施异常处理

---

## OFFBOARD模式保持机制

### PX4 OFFBOARD要求

根据PX4官方文档：
1. **Setpoint频率**: 必须 ≥ 2Hz
2. **超时时间**: 0.5秒无setpoint → 自动退出OFFBOARD
3. **解锁条件**: 必须在OFFBOARD模式下才能解锁

### 后台Setpoint线程（借鉴XTDrone）

**核心实现** (`waypoint_flight.py`):

```python
def _background_setpoint_sender(self):
    """后台线程持续发送setpoint保持OFFBOARD模式"""
    rate = rospy.Rate(10)  # 10Hz，远高于2Hz最低要求
    
    while not rospy.is_shutdown() and self.background_setpoint_active:
        if self.current_position:
            target = PositionTarget()
            target.header.stamp = rospy.Time.now()
            target.header.frame_id = "map"
            target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            
            # 保持当前位置
            target.position.x = self.current_position.x
            target.position.y = self.current_position.y
            target.position.z = self.current_position.z
            target.yaw = self.current_yaw
            
            # 只在主循环未活动时发送（避免冲突）
            if not self.mission_active:
                self.setpoint_pub.publish(target)
        
        rate.sleep()

def start_mission(self):
    """启动任务"""
    # 1. 启动后台线程
    self.background_setpoint_active = True
    self.background_thread = threading.Thread(
        target=self._background_setpoint_sender,
        daemon=True
    )
    self.background_thread.start()
    rospy.loginfo("✅ 后台setpoint发送器已启动")
    
    # 2. 发送初始setpoint
    self._send_initial_setpoints(count=100)
    
    # 3. 切换OFFBOARD模式
    self._set_mode("OFFBOARD")
    
    # 4. 解锁
    self._arm_vehicle()
    
    # 5. 起飞后，停止后台线程
    self.mission_active = True  # 主循环接管
    self.background_setpoint_active = False
```

**时序图**:
```
0s    → 系统启动
5s    → 无人机0启动
        ├─ 启动后台setpoint线程（10Hz持续发送）
        ├─ 发送初始setpoint（100次，5秒）
        ├─ 切换OFFBOARD模式
        ├─ 解锁（后台线程保持OFFBOARD）✅
        └─ 起飞完成，主循环接管

6s    → 无人机1启动（并行，仅延迟1秒）
        └─ 同样流程...
```

### 并行启动策略（参考XTDrone）

**XTDrone官方做法** (`multi_vehicle_communication.sh`):
```bash
while(( $vehicle_num < 6 ))
do
    python3 multirotor_communication.py iris $vehicle_num &  # 后台并行
    let "vehicle_num++"
done
```

**我们的实现**:
```python
# 错峰启动间隔仅1秒（避免瞬时冲突）
stagger_delay = 5.0 + drone_id * 1.0

# 无人机0: 5秒
# 无人机1: 6秒（仅延迟1秒）
# 无人机2: 7秒（仅延迟1秒）
```

**关键**: 使用后台setpoint线程保持OFFBOARD，只需很小延迟避免资源竞争。

### 解锁重试机制

```python
def _arm_vehicle(self):
    """解锁无人机（带自动重设OFFBOARD）"""
    max_attempts = 10
    
    for attempt in range(max_attempts):
        # 检查MAVROS状态
        if self.mavros_state is None:
            rospy.sleep(1.0)
            continue
        
        # 检查OFFBOARD模式
        if self.mavros_state.mode != "OFFBOARD":
            rospy.logwarn(f"当前模式: {self.mavros_state.mode}, 重设OFFBOARD...")
            self._send_initial_setpoints(count=50)
            self._set_mode("OFFBOARD")
            rospy.sleep(1.5)
            continue
        
        # 尝试解锁
        if self.arming_client(True).success:
            rospy.loginfo("✅ 无人机已解锁")
            return True
        
        rospy.sleep(1.0)
    
    rospy.logerr("❌ 无人机解锁失败（10次重试后）")
    return False
```

---

## 分布式协调算法

### 架构设计

**核心思想**: 无中央控制器，每架无人机独立运行相同的协调算法。

**协调器节点** (`multi_drone_coordinator.py`):
```
每架无人机运行一个独立的coordinator节点
├── 订阅其他无人机的tracking_claim
├── 发布自己的tracking_claim
├── 计算到目标的距离
├── 比较所有声明，决定是否获得许可
└── 发布coordinator_status
```

### 话题设计

#### 1. 追踪声明话题
```python
# 话题: /drone_<id>/tracking_claim
# 消息类型: PointStamped

class TrackingClaim:
    header: Header
    point: Point  # 目标位置(x, y, z)
```

**发布条件**:
- 检测到人体目标
- 开始追踪
- 追踪期间持续发布（1Hz）

**超时机制**:
- 3秒无更新 → 自动释放许可
- 追踪结束 → 停止发布

#### 2. 协调器状态话题
```python
# 话题: /drone_<id>/coordinator_status
# 消息类型: String

状态值:
- "IDLE" - 空闲
- "TRACKING_PERMITTED" - 获得追踪许可
- "TRACKING_DENIED" - 追踪被拒绝（距离远）
```

### 核心算法

```python
def _check_coordination(self, target_position):
    """检查是否获得追踪许可"""
    my_distance = self._calculate_distance(
        self.current_position, 
        target_position
    )
    
    # 检查其他无人机的声明
    for other_drone_id, claim_data in self.other_drones_claims.items():
        # 计算是否是同一目标（距离<5米）
        target_distance = self._calculate_distance(
            target_position,
            claim_data['position']
        )
        
        if target_distance < self.same_target_threshold:  # 5米
            # 比较距离
            other_distance = claim_data['distance']
            
            if other_distance < my_distance - 0.5:  # 0.5米滞后
                # 其他无人机更近，让位
                rospy.loginfo(f"让位给无人机{other_drone_id}")
                return False
    
    # 获得许可
    return True
```

**距离优先规则**:
1. 计算每架无人机到目标的距离
2. 距离最近的无人机获得追踪许可
3. 其他无人机继续巡检
4. 0.5米滞后避免频繁切换

### 超时与恢复

```python
def _update_other_drones_claims(self):
    """更新其他无人机的声明（超时检查）"""
    current_time = rospy.Time.now()
    
    for drone_id in list(self.other_drones_claims.keys()):
        claim = self.other_drones_claims[drone_id]
        
        # 检查超时（3秒）
        if (current_time - claim['timestamp']).to_sec() > 3.0:
            rospy.loginfo(f"无人机{drone_id}声明超时，释放")
            del self.other_drones_claims[drone_id]
```

---

## 记分系统架构

### 单例防重复机制

**双重锁设计**:

#### 1. 文件锁
```python
import fcntl

lock_file = os.path.expanduser('~/.robocup_score_cal.lock')
lock_fd = open(lock_file, 'w')

try:
    fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    rospy.loginfo("✅ 记分系统文件锁获取成功")
except IOError:
    rospy.logerr("❌ 记分系统已在运行（文件锁）")
    sys.exit(1)
```

#### 2. 端口锁
```python
import socket

port_lock_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    port_lock_socket.bind(('localhost', 46321))
    rospy.loginfo("✅ 记分系统端口锁获取成功")
except OSError:
    rospy.logerr("❌ 记分系统已在运行（端口锁）")
    sys.exit(1)
```

### 记分规则

```python
def calculate_score(self):
    """计算得分"""
    if self.all_actors_eliminated():
        # 全部完成：基础分1200 - 用时 - 传感器成本
        score = 1200 - self.time_used - self.sensor_cost * 0.003
    else:
        # 进行中：(2 + 完成数) × 60 - 传感器成本
        score = (2 + self.eliminated_count) * 60 - self.sensor_cost * 0.003
    
    return max(0, int(score))
```

### Actor消除条件

**触发条件**（3个条件同时满足）:
1. **距离误差** < 2.0米
2. **连续检测** ≥ 8秒
3. **话题频率** > 1Hz

```python
def _check_actor_elimination(self, actor_id, detection_info):
    """检查是否满足消除条件"""
    # 1. 计算距离误差
    error = math.sqrt(
        (detection_info.x - actor_info.true_x)**2 +
        (detection_info.y - actor_info.true_y)**2
    )
    
    if error < self.err_threshold:  # 2.0米
        # 2. 累积检测时间
        actor_info.stable_detection_time += dt
        
        # 3. 检查是否达到8秒
        if actor_info.stable_detection_time >= self.detection_time:
            self._eliminate_actor(actor_id)
```

### 可视化界面

**OpenCV界面结构**:
```
┌─────────────────────────────────────────┐
│   RoboCup Score Visualization          │
├─────────────────────────────────────────┤
│ Score: 150    Time: 120/600s            │
│ Left: 4       Done: 2       Alt: 3.2m   │
├─────────────────────────────────────────┤
│ Actor   Status    Pos      Error  Time  │
│ actor_0 TRACK     (10,20)  1.2m   5.3s  │
│ actor_1 SEARCH    (30,40)  -      -     │
│ actor_2 DEL       -         -      -     │
│ actor_3 TRACK     (50,60)  0.8m   7.1s  │
│ actor_4 SEARCH    (70,80)  -      -     │
│ actor_5 SEARCH    (90,10)  -      -     │
└─────────────────────────────────────────┘
```

**状态颜色**:
- 绿色: DEL（已消除）
- 黄色: TRACK（追踪中）
- 白色: SEARCH（搜索中）
- 红色: 警告（高度超限）

---

## 6机扩展指南

### 步骤1: 生成航点

```bash
cd ~/catkin_ws/src/yolov11_ros/scripts
python3 multi_drone_waypoint_planner.py --num_drones 6 --height 3.0 --output_dir ../waypoints
```

**生成文件**:
```
waypoints/
├── waypoints_drone_0.json  (区域1)
├── waypoints_drone_1.json  (区域2)
├── waypoints_drone_2.json  (区域3)
├── waypoints_drone_3.json  (区域4)
├── waypoints_drone_4.json  (区域5)
├── waypoints_drone_5.json  (区域6)
└── waypoints_summary.json  (总览)
```

### 步骤2: 修改robocup.launch

添加无人机2-5的配置：

```xml
<!-- typhoon_h480_2 -->
<group ns="typhoon_h480_2">
    <arg name="ID" value="2"/>
    <arg name="ID_in_group" value="2"/>
    <arg name="fcu_url" default="udp://:24542@localhost:34582"/>
    <include file="$(find px4)/launch/single_vehicle_spawn_xtd.launch">
        <arg name="x" value="-10"/>
        <arg name="y" value="-5"/>
        <arg name="z" value="1"/>
        <arg name="R" value="0"/>
        <arg name="P" value="0"/>
        <arg name="Y" value="0"/>
        <arg name="vehicle" value="typhoon_h480"/>
        <arg name="sdf" value="typhoon_h480_2"/>
        <arg name="udp_gimbal_port" value="13032"/>  <!-- 端口递增 -->
        <arg name="mavlink_udp_port" value="18572"/>
        <arg name="mavlink_tcp_port" value="4562"/>
        <arg name="ID" value="$(arg ID)"/>
        <arg name="ID_in_group" value="$(arg ID_in_group)"/>
    </include>
    <include file="$(find mavros)/launch/px4.launch">
        <arg name="fcu_url" value="$(arg fcu_url)"/>
        <arg name="gcs_url" value=""/>
        <arg name="tgt_system" value="$(eval 1 + arg('ID'))"/>
        <arg name="tgt_component" value="1"/>
    </include>
</group>

<!-- 类似地添加typhoon_h480_3, 4, 5 -->
```

**端口规则**:
```
无人机ID=2:
- 云台UDP: 13032 (13030 + 2)
- MAVLink UDP: 18572 (18570 + 2)
- MAVLink TCP: 4562 (4560 + 2)
- FCU: 24542 (24540 + 2)
```

### 步骤3: 修改multi_drone_flight.launch

```xml
<!-- 无人机2 -->
<group ns="drone_2">
    <!-- 云台控制 -->
    <node name="gimbal_control_typhoon_h480_2" pkg="yolov11_ros" 
          type="gimbal_control.py" args="typhoon_h480 2" output="screen" />
    
    <!-- 协调器 -->
    <node name="coordinator" pkg="yolov11_ros" 
          type="multi_drone_coordinator.py" output="screen">
        <param name="drone_id" value="2"/>
        <param name="num_drones" value="6"/>
    </node>
    
    <!-- 航点飞行 -->
    <node name="waypoint_flight" pkg="yolov11_ros" 
          type="waypoint_flight.py" output="screen">
        <param name="drone_id" value="2"/>
        <param name="waypoint_file" value="$(find yolov11_ros)/waypoints/waypoints_drone_2.json"/>
    </node>
    
    <!-- 人体跟踪 -->
    <node name="human_tracker" pkg="yolov11_ros" 
          type="human_tracker.py" output="screen">
        <param name="drone_id" value="2"/>
        <param name="detection_topic" value="/yolov11/bounding_boxes_drone2"/>
    </node>
    
    <!-- 避障系统 -->
    <node name="obstacle_avoidance" pkg="yolov11_ros" 
          type="obstacle_avoidance.py" output="screen">
        <param name="drone_id" value="2"/>
    </node>
</group>

<!-- 类似地添加drone_3, 4, 5 -->
```

### 步骤4: 测试启动

```bash
# 1. 清理环境
cd ~/catkin_ws/src/yolov11_ros/scripts
./cleanup_all.sh

# 2. 启动Gazebo（等待90秒，6机加载时间更长）
roslaunch px4 robocup.launch

# 3. 启动6机系统
roslaunch yolov11_ros multi_drone_flight.launch num_drones:=6
```

**预期启动时序**:
```
5s  → 无人机0启动
6s  → 无人机1启动
7s  → 无人机2启动
8s  → 无人机3启动
9s  → 无人机4启动
10s → 无人机5启动
30s → 全部起飞完成
```

---

## 纯视觉伺服技术细节

### 算法架构

**核心思想**: 直接使用目标在图像中的大小（box_size）作为距离反馈，实现自适应速度控制。

```
检测框大小 (box_size)
    ↓
距离估计 (size_ratio)
    ↓
PD控制器
    ├── P项: 距离误差 → 基础速度
    └── D项: 距离变化率 → 速度增量
         ↓
    期望速度
         ↓
    多级滤波
         ↓
    速度命令
```

### 控制公式

```python
# 1. 距离比例和误差
size_ratio = smoothed_box_size / ideal_box_size
size_error_ratio = (ideal_box_size - smoothed_box_size) / ideal_box_size

# 2. PD控制器
base_speed = K1 * size_error_ratio           # P控制
speed_increment = -K2 * size_change_rate      # D控制
desired_speed = base_speed + speed_increment

# 3. 动态滤波
x_velocity = alpha * desired_speed + (1 - alpha) * x_velocity_prev
```

### 多级滤波机制

1. **box_size平滑** (5帧加权平均)
   ```python
   weights = [0.05, 0.10, 0.15, 0.25, 0.45]  # 最近帧权重最大
   smoothed_size = Σ(weight_i * size_i)
   ```

2. **动态alpha选择**
   - 极稳定状态: α = 0.02
   - 正常追踪: α = 0.25
   - 误差大: α = 0.50
   - 人加速: α = 0.80

3. **急刹保护**
   ```python
   if size_ratio > 1.20:  # 距离太近
       desired_speed = min(desired_speed, 0.4)
   ```

### 参数调试指南

#### 测量ideal_box_size
1. 让目标站在理想距离（如2米）
2. 运行系统，记录稳定时的box_size值
3. 设置为ideal_box_size参数

#### 调整增益参数
- **追快速目标**: 提高K1、K2和max_forward_speed
- **追慢速目标**: 降低K1、K2和max_forward_speed
- **抖动严重**: 降低alpha值或减小增益

---

## 关键参数详解

### 协调器参数

```python
# multi_drone_coordinator.py

same_target_threshold = 5.0  # 米
# 说明: 两个目标位置距离<5米时，认为是同一目标
# 影响: 值过小→频繁冲突；值过大→漏掉相邻目标

claim_timeout = 3.0  # 秒
# 说明: 追踪声明超时时间
# 影响: 值过小→频繁释放；值过大→无人机故障后长时间占用

position_update_rate = 10  # Hz
# 说明: 位置信息发布频率
# 影响: 值过低→协调延迟；值过高→网络负载
```

### 航点飞行参数

```python
# waypoint_flight.py

takeoff_altitude = 3.0  # 米
# 说明: 起飞高度
# 影响: 高度过低→视野小；高度过高→检测精度降低

waypoint_velocity = 5.0  # m/s
# 说明: 巡航速度
# 影响: 速度过快→检测漏失；速度过慢→巡检时间长

waypoint_reached_threshold = 2.0  # 米
# 说明: 到达航点的判定距离
# 影响: 值过小→频繁调整；值过大→路径粗糙

setpoint_publish_rate = 20  # Hz
# 说明: 控制指令发送频率
# 影响: 必须≥2Hz保持OFFBOARD，推荐10-30Hz
```

### 人体跟踪参数

```python
# human_tracker.py

detection_confidence = 0.3  # 0-1
# 说明: YOLO检测置信度阈值
# 影响: 值过高→漏检；值过低→误检

Kp_xy = 0.5  # 比例增益
# 说明: 水平方向PID控制增益
# 影响: 值过大→振荡；值过小→响应慢

max_vel = 2.0  # m/s
# 说明: 追踪时最大速度
# 影响: 值过大→不稳定；值过小→跟不上

tracking_height = 3.0  # 米
# 说明: 追踪时保持高度
# 影响: 同takeoff_altitude
```

### 避障参数

```python
# obstacle_avoidance.py

waypoint_safe_distance = 3.5  # 米
# 说明: 航点模式下的安全距离
# 影响: 值过小→碰撞风险；值过大→过度绕行

tracking_safe_distance = 2.5  # 米
# 说明: 追踪模式下的安全距离
# 影响: 追踪时需更灵活，距离可稍小

emergency_distance = 1.5  # 米
# 说明: 紧急避障触发距离
# 影响: 立即停止并后退

avoidance_velocity = 1.0  # m/s
# 说明: 避障移动速度
# 影响: 避障时降低速度提高安全性
```

### 记分系统参数

```python
# robocup_score_cal.py

actor_num = 6  # 数量
# 说明: 目标actor数量

err_threshold = 2.0  # 米
# 说明: 位置误差阈值，<2米才计入检测时间
# 影响: 值过严→难以消除；值过宽→精度不够

detection_time = 8.0  # 秒
# 说明: 连续稳定检测时间要求
# 影响: 值过短→误判；值过长→消除慢

sensor_cost = 700.0  # 成本
# 说明: 传感器固定成本

timeout_sec = 600  # 秒
# 说明: 任务超时时间（10分钟）
```

---

## 故障排查技术

### 诊断脚本

创建快速诊断脚本：

```bash
#!/bin/bash
# diagnose_system.sh

echo "========================================="
echo "系统诊断工具"
echo "========================================="

# 1. ROS核心
echo -e "\n1. ROS Master:"
if rostopic list &>/dev/null; then
    echo "✅ ROS Master运行正常"
else
    echo "❌ ROS Master未运行"
    exit 1
fi

# 2. Gazebo
echo -e "\n2. Gazebo模型:"
for i in 0 1; do
    result=$(rosservice call /gazebo/get_model_state "model_name: 'typhoon_h480_$i'" 2>&1)
    if echo "$result" | grep -q "success: True"; then
        echo "✅ typhoon_h480_$i 已加载"
    else
        echo "❌ typhoon_h480_$i 未加载"
    fi
done

# 3. MAVROS连接
echo -e "\n3. MAVROS连接:"
for i in 0 1; do
    connected=$(rostopic echo /typhoon_h480_$i/mavros/state -n 1 2>&1 | grep "connected:" | awk '{print $2}')
    if [ "$connected" = "True" ]; then
        echo "✅ typhoon_h480_$i MAVROS已连接"
    else
        echo "❌ typhoon_h480_$i MAVROS未连接"
    fi
done

# 4. 云台控制
echo -e "\n4. 云台控制节点:"
for i in 0 1; do
    if rosnode list | grep -q "gimbal_control_typhoon_h480_$i"; then
        echo "✅ gimbal_control_$i 运行中"
        # 检查频率
        hz=$(rostopic hz /typhoon_h480_$i/mavros/mount_control/command -d 5 2>&1 | grep "average rate:" | awk '{print $3}')
        echo "   频率: ${hz}Hz"
    else
        echo "❌ gimbal_control_$i 未运行"
    fi
done

# 5. 协调器
echo -e "\n5. 协调器节点:"
for i in 0 1; do
    if rosnode list | grep -q "/drone_$i/coordinator"; then
        echo "✅ coordinator_$i 运行中"
    else
        echo "❌ coordinator_$i 未运行"
    fi
done

# 6. 记分系统
echo -e "\n6. 记分系统:"
score_count=$(ps aux | grep robocup_score_cal.py | grep -v grep | wc -l)
if [ $score_count -eq 1 ]; then
    echo "✅ 记分系统运行正常（1个实例）"
elif [ $score_count -gt 1 ]; then
    echo "⚠️  记分系统有${score_count}个实例（应只有1个）"
else
    echo "❌ 记分系统未运行"
fi

# 7. 锁文件
echo -e "\n7. 锁文件检查:"
if [ -f ~/.robocup_score_cal.lock ]; then
    echo "⚠️  锁文件存在: ~/.robocup_score_cal.lock"
else
    echo "✅ 无陈旧锁文件"
fi

echo -e "\n========================================="
echo "诊断完成"
echo "========================================="
```

### 日志分析

**查看关键日志**:

```bash
# 1. 实时查看所有日志
rqt_console

# 2. 查看特定节点日志
rosnode info /drone_0/waypoint_flight

# 3. 过滤ERROR日志
rqt_console | grep ERROR

# 4. 查看OFFBOARD模式切换
rostopic echo /typhoon_h480_0/mavros/state | grep mode
```

**常见错误模式**:

| 错误信息 | 原因 | 解决方法 |
|---------|------|---------|
| `AUTO.RTL` | OFFBOARD超时 | 检查后台setpoint线程 |
| `GetModelState: model does not exist` | Gazebo模型未加载 | 检查SDF文件和launch配置 |
| `Service not available` | 服务未就绪 | 等待Gazebo完全加载 |
| `记分系统已在运行` | 重复实例 | 清理锁文件和进程 |

### 性能监控

```bash
# CPU使用率
top -p $(pgrep -d',' -f 'yolo_v11\|waypoint_flight\|human_tracker')

# 网络带宽
rostopic bw /typhoon_h480_0/cgo3_camera/image_raw

# 话题延迟
rostopic delay /typhoon_h480_0/mavros/local_position/pose

# 节点图可视化
rqt_graph
```

---

## 技术参考

### XTDrone借鉴内容

1. **OFFBOARD保持机制**: `coordination/fixed_wing_affine_formation_control/`
2. **并行启动策略**: `communication/multi_vehicle_communication.sh`
3. **云台控制脚本**: `control/gimbal_control.py`

### PX4官方文档

- [OFFBOARD Mode](https://docs.px4.io/main/en/flight_modes/offboard.html)
- [Multi-Vehicle Simulation](https://docs.px4.io/main/en/simulation/multi-vehicle-simulation.html)
- [Gimbal Control](https://docs.px4.io/main/en/advanced/gimbal_control.html)

### Gazebo SDF规范

- [SDF Format](http://sdformat.org/)
- [Model Database](https://github.com/osrf/gazebo_models)

---

**维护**: 东华大学 Astraeus队  
**最后更新**: 2025-10-06  
**版本**: v6.0

