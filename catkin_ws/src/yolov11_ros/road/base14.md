base14.world

垂直道路 (Y方向)
**road_y_1**: x = -45, y ∈ [-48.7, 49.2]
**road_y_2**: x = -15, y ∈ [-48.7, 49.2]  
**road_y_3**: x = 55-35, y ∈ [-48.7, 49.2] (斜向道路)
**road_y_4**: x = 110, y ∈ [-48.7, 49.2]
**road_y_5**: x = 120, y ∈ [-48.7, 49.2]

水平道路 (X方向)
**road_x_1**: y = -45, x ∈ [-41.3, 116.3]
**road_x_2**: y = 0, x ∈ [-41.3, 116.3]
**road_x_3**: y = 45, x ∈ [-41.3, 116.3]

9个街道转弯处坐标 (x, y, z)
1. **I1**: (-15, -45, 0.01) - road_y_2 与 road_x_1 交点
2. **I2**: (55, -45, 0.01) - road_y_3 与 road_x_1 交点  
3. **I3**: (110, -45, 0.01) - road_y_4 与 road_x_1 交点
4. **I4**: (-15, 0, 0.015) - road_y_2 与 road_x_2 交点
5. **I5**: (55, 0, 0.015) - road_y_3 与 road_x_2 交点
6. **I6**: (110, 0, 0.015) - road_y_4 与 road_x_2 交点
7. **I7**: (-15, 45, 0.015) - road_y_2 与 road_x_3 交点
8. **I8**: (55, 45, 0.015) - road_y_3 与 road_x_3 交点
9. **I9**: (110, 45, 0.015) - road_y_4 与 road_x_3 交点

2. 地图四角坐标

**C1_左下**: (-45, -48.7, 0)
**C2_左上**: (-45, 49.2, 0)  
**C3_右下**: (120, -48.7, 0)
**C4_右上**: (120, 49.2, 0)

3. 六区域无人机巡航方案

区域划分
**UAV1_R1**: x ∈ [-45, 20], y ∈ [-48.7, -15] - 左下区域
**UAV2_R2**: x ∈ [20, 82.5], y ∈ [-48.7, -15] - 中下区域  
**UAV3_R3**: x ∈ [82.5, 120], y ∈ [-48.7, -15] - 右下区域
**UAV4_R4**: x ∈ [-45, 20], y ∈ [-15, 49.2] - 左上区域
**UAV5_R5**: x ∈ [20, 82.5], y ∈ [-15, 49.2] - 中上区域
**UAV6_R6**: x ∈ [82.5, 120], y ∈ [-15, 49.2] - 右上区域

道路覆盖分析
**UAV1_R1**: 覆盖 road_y_1, road_y_2, road_x_1 部分
**UAV2_R2**: 覆盖 road_y_3, road_x_1, road_x_2 部分
**UAV3_R3**: 覆盖 road_y_4, road_y_5, road_x_1 部分
**UAV4_R4**: 覆盖 road_y_1, road_y_2, road_x_2, road_x_3 部分
**UAV5_R5**: 覆盖 road_y_3, road_x_2, road_x_3 部分
**UAV6_R6**: 覆盖 road_y_4, road_y_5, road_x_2, road_x_3 部分

## 4. Python 航点生成代码

```python
import math

# 9 个街道转弯处坐标 (x, y, z)
intersections = [
    (-15, -45, 0.01), (55, -45, 0.01), (110, -45, 0.01),
    (-15, 0, 0.015), (55, 0, 0.015), (110, 0, 0.015),
    (-15, 45, 0.015), (55, 45, 0.015), (110, 45, 0.015)
]

# 地图四角坐标 (min_x, min_y, max_x, max_y)
map_corners = {
    "C1_左下": (-45, -48.7, 0),
    "C2_左上": (-45, 49.2, 0),
    "C3_右下": (120, -48.7, 0),
    "C4_右上": (120, 49.2, 0)
}

# 定义六个区域的边界 (min_x, max_x, min_y, max_y)
regions = {
    "UAV1_R1": (-45, 20, -48.7, -15),
    "UAV2_R2": (20, 82.5, -48.7, -15),
    "UAV3_R3": (82.5, 120, -48.7, -15),
    "UAV4_R4": (-45, 20, -15, 49.2),
    "UAV5_R5": (20, 82.5, -15, 49.2),
    "UAV6_R6": (82.5, 120, -15, 49.2)
}

# 无人机飞行高度
flight_altitude = 5.5  # meters, < 6m

def generate_waypoints(region_name, region_bounds, altitude):
    min_x, max_x, min_y, max_y = region_bounds
    waypoints = []
    
    # 为每个区域生成网格航点
    step = 20  # 航点间隔
    for x in range(math.ceil(min_x), math.floor(max_x) + 1, step):
        for y in range(math.ceil(min_y), math.floor(max_y) + 1, step):
            waypoints.append((x, y, altitude))
    return waypoints

# 生成所有无人机的航点
all_uav_waypoints = {}
for uav, bounds in regions.items():
    all_uav_waypoints[uav] = generate_waypoints(uav, bounds, flight_altitude)

# 打印结果
print("=== base14.world 无人机巡航方案 ===")
print(f"飞行高度: {flight_altitude}m")
print(f"街道转弯点数量: {len(intersections)}")
print(f"区域数量: {len(regions)}")
print()

for uav, waypoints in all_uav_waypoints.items():
    print(f"{uav} 航点数量: {len(waypoints)}")
    for i, wp in enumerate(waypoints[:5]):  # 显示前5个航点
        print(f"  - 航点{i+1}: {wp}")
    if len(waypoints) > 5:
        print(f"  - ... 还有 {len(waypoints)-5} 个航点")
    print()
```