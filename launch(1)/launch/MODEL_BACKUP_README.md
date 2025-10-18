# 无人机模型备份说明

## 备份日期
2025-10-16

## 备份内容

已备份6台无人机模型到此目录：
- `typhoon_h480_0_backup/` - 无人机0模型（含meshes）
- `typhoon_h480_1_backup/` - 无人机1模型（含meshes）
- `typhoon_h480_2_backup/` - 无人机2模型（含meshes）
- `typhoon_h480_3_backup/` - 无人机3模型（含meshes）
- `typhoon_h480_4_backup/` - 无人机4模型（含meshes）
- `typhoon_h480_5_backup/` - 无人机5模型（含meshes）

## 修改内容

### 删除了2d激光雷达

**typhoon_h480_0 和 typhoon_h480_1**：
- 删除了 `laser_rangefinder` 配置
- 位置：sdf文件末尾

**typhoon_h480_2/3/4/5**：
- 删除了 `hokuyo_lidar` 配置（2d激光雷达）
- 位置：sdf文件末尾

### 修改前的配置

```xml
<!-- typhoon_h480_0/1 -->
<include>
    <uri>model://laser_rangefinder</uri>
    <pose>0 0 -0.05 0 1.570796 0</pose>
</include>
<joint name="laser_rangefinder_joint" type="revolute">
  <child>laser_rangefinder::link</child>
  <parent>base_link</parent>
  ...
</joint>

<!-- typhoon_h480_2/3/4/5 -->
<include>
  <uri>model://hokuyo_lidar</uri>
  <pose>0 0 0.15 0 0 0</pose>
</include>
<joint name="hokuyo_lidar_joint" type="revolute">
  <child>hokuyo_lidar::link</child>
  <parent>base_link</parent>
  ...
</joint>
```

### 修改后的配置

```xml
<!-- 所有模型 -->
<!-- 2d激光雷达已移除 -->

</model>
</sdf>
```

## 原模型位置

```
PX4_Firmware/Tools/sitl_gazebo/models/
├── typhoon_h480_0/
├── typhoon_h480_1/
├── typhoon_h480_2/
├── typhoon_h480_3/
├── typhoon_h480_4/
└── typhoon_h480_5/
```

## 恢复方法

如果需要恢复原始模型（包含激光雷达）：

```bash
# 恢复单个模型（例如无人机0）
xcopy launch(1)\launch\typhoon_h480_0_backup PX4_Firmware\Tools\sitl_gazebo\models\typhoon_h480_0 /E /I /Y

# 恢复所有模型
for i in {0..5}; do
  xcopy launch(1)\launch\typhoon_h480_${i}_backup PX4_Firmware\Tools\sitl_gazebo\models\typhoon_h480_${i} /E /I /Y
done
```

## 注意事项

⚠️ **重要**：
1. 备份文件包含完整的模型定义和mesh文件
2. 仅删除了激光雷达传感器，其他配置完全保留
3. 相机、云台、电机等所有其他组件未修改
4. 如有需要，可以随时从备份恢复

## 修改原因

- 移除2d激光雷达以简化系统
- 减少传感器计算负担
- 专注于视觉目标检测和追踪
- 避免不必要的传感器数据

---

**备份人员**：AI助手  
**日期**：2025-10-16  
**东华大学 Astraeus队**

