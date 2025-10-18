# PX4 Firmware - RoboCup 2025 配置

本目录为PX4官方飞控固件，已针对RoboCup 2025多无人机仿真进行配置。

## ⚠️ 重要说明

- 这是**官方PX4仓库**，大部分文件保持原样
- 仅修改了仿真相关的配置文件
- 详细文档请参考[PX4官方文档](https://dev.px4.io/)

## 🔧 项目修改内容

### 1. 多无人机启动配置

**文件**: `launch/robocup.launch`

针对RoboCup 2025配置的6架无人机启动文件：

```xml
<!-- 6架Typhoon H480无人机配置 -->
<!-- UAV 0-5，每架有独立的端口配置 -->
```

**关键配置**:
- 起飞位置（ENU坐标系）
- MAVROS端口映射
- Gazebo模型加载

详见: [docs/6机协同配置说明.md](../docs/6机协同配置说明.md)

### 2. 无人机模型文件

**目录**: `Tools/sitl_gazebo/models/`

包含项目使用的无人机模型：
- `typhoon_h480_0/` 到 `typhoon_h480_5/`
- 每架无人机有独立的sdf模型文件
- 配置了相机、云台等传感器

### 3. 世界文件

**目录**: `Tools/sitl_gazebo/worlds/`

RoboCup官方比赛地图：
- `base0.world` 到 `base16.world`
- 包含建筑物、道路、行人等

## 🚀 使用方法

### 启动仿真

```bash
cd PX4_Firmware

# 配置环境
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

# 启动6架无人机
roslaunch launch/robocup.launch

# 指定地图（默认base0.world）
roslaunch launch/robocup.launch world:=$(pwd)/Tools/sitl_gazebo/worlds/base1.world
```

### 单机测试

```bash
# 启动单架无人机（默认Iris）
make px4_sitl gazebo

# 启动Typhoon H480
make px4_sitl gazebo_typhoon_h480
```

## 📁 重要目录

```
PX4_Firmware/
├── launch/
│   └── robocup.launch          # RoboCup多机配置 ⭐
│
├── Tools/
│   └── sitl_gazebo/
│       ├── models/              # 无人机模型
│       │   ├── typhoon_h480_0/  # UAV 0模型 ⭐
│       │   ├── typhoon_h480_1/  # UAV 1模型 ⭐
│       │   └── ...
│       └── worlds/              # 仿真世界
│           ├── base0.world      # 地图0 ⭐
│           └── ...
│
├── build/                       # 编译输出
│   └── px4_sitl_default/        # SITL编译结果
│
└── src/                         # PX4源码
    └── ...                      # 官方代码，未修改
```

## 🔍 端口配置

### MAVROS端口映射

6架无人机的端口配置（ID=0到5）：

| UAV | fcu_url | mavlink_udp | mavlink_tcp | udp_gimbal | tgt_system |
|-----|---------|-------------|-------------|------------|------------|
| 0   | 24540   | 18570       | 4560        | 13030      | 1          |
| 1   | 24541   | 18571       | 4561        | 13031      | 2          |
| 2   | 24542   | 18572       | 4562        | 13032      | 3          |
| 3   | 24543   | 18573       | 4563        | 13033      | 4          |
| 4   | 24544   | 18574       | 4564        | 13034      | 5          |
| 5   | 24545   | 18575       | 4565        | 13035      | 6          |

公式: `port = base_port + drone_id`

### 起飞位置配置

ENU坐标系（单位：米）：

```
UAV0: (-17, -3, 1)    UAV1: (-14, -3, 1)
UAV2: (-17,  0, 1)    UAV3: (-14,  0, 1)
UAV4: (-17,  3, 1)    UAV5: (-14,  3, 1)
```

## 🛠️ 编译

### 首次编译

```bash
cd PX4_Firmware

# 编译SITL版本
make px4_sitl_default gazebo

# 或只编译不启动
make px4_sitl_default
```

### 清理重编译

```bash
# 清理编译文件
make clean

# 完全清理
make distclean

# 重新编译
make px4_sitl_default
```

## 🐛 调试

### 查看PX4日志

```bash
# 日志目录
ls ~/.ros/log/latest/

# 实时查看
tail -f ~/.ros/log/latest/px4-*.log
```

### 检查MAVROS连接

```bash
# 查看连接状态
rostopic echo /mavros_0/state

# 应该显示: connected: true
```

### 端口占用检查

```bash
# 检查端口是否被占用
netstat -tulpn | grep 14540
netstat -tulpn | grep 18570
```

## ⚙️ 配置修改

### 修改无人机数量

编辑 `launch/robocup.launch`:

```xml
<!-- 添加新的无人机 -->
<group ns="uav6">
    <arg name="ID" value="6"/>
    <!-- 配置端口和位置 -->
</group>
```

### 修改起飞位置

```xml
<arg name="x" default="-17.0"/>
<arg name="y" default="-3.0"/>
<arg name="z" default="1.0"/>
```

### 切换地图

```bash
roslaunch launch/robocup.launch world:=$(pwd)/Tools/sitl_gazebo/worlds/base5.world
```

## 📚 官方文档

- [PX4用户手册](https://docs.px4.io/)
- [PX4开发指南](https://dev.px4.io/)
- [Gazebo仿真](https://dev.px4.io/master/en/simulation/gazebo.html)
- [多机仿真](https://dev.px4.io/master/en/simulation/multi-vehicle-simulation.html)

## ❓ 常见问题

### Q: 编译失败

```bash
# 更新子模块
git submodule update --init --recursive

# 安装依赖
bash ./Tools/setup/ubuntu.sh
```

### Q: Gazebo崩溃

```bash
# 重置Gazebo
killall -9 gzserver gzclient
rm -rf ~/.gazebo

# 重新启动
roslaunch launch/robocup.launch
```

### Q: 无人机不出现

检查模型路径：
```bash
echo $GAZEBO_MODEL_PATH
# 应该包含: Tools/sitl_gazebo/models
```

## 🔄 更新PX4

**⚠️ 警告**: 更新PX4可能导致配置丢失！

```bash
# 备份配置
cp launch/robocup.launch ~/robocup_backup/

# 更新PX4
git pull origin master

# 恢复配置
cp ~/robocup_backup/robocup.launch launch/
```

## 📄 许可证

PX4 Autopilot采用BSD-3-Clause许可证。

项目配置文件版权：
**东华大学人工智能创新实验室**

详见: [LICENSE](../LICENSE) 和 [PX4 LICENSE](LICENSE)

---

**仅修改launch文件和模型配置，核心PX4代码保持原样！**
