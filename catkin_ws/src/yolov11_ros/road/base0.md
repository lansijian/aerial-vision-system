base0.world

**道路元素**:
垂直道路: `x = -45, -15, 45, 110, 120`，范围 `y ∈ [-48.7, 49.2]`
水平道路: `y = -45, 0, 45`，范围 `x ∈ [-41.3, 116.3]`

说明: 水平道路仅覆盖 `x ∈ [-41.3, 116.3]`，因此与 `x=120` 的垂直道路不相交；同时 `x=-45` 小于水平道路最小 `x=-41.3`，因此也不相交。有效相交（即街道转弯处/路口）来自 `x ∈ {-15, 45, 110}` 与 `y ∈ {-45, 0, 45}` 的笛卡尔积，共 9 个。

1) 9 个路口坐标
**I1**: (-15, -45, 0.01)
**I2**: (45, -45, 0.01)
**I3**: (110, -45, 0.01)
**I4**: (-15, 0, 0.015)
**I5**: (45, 0, 0.015)
**I6**: (110, 0, 0.015)
**I7**: (-15, 45, 0.015)
**I8**: (45, 45, 0.015)
**I9**: (110, 45, 0.015)

注: 地面高程以 SDF 中道路的 z 值为准（垂直道路 0.01，水平道路 0.015），飞行时将目标高度设为相对地面高度 < 6 m。

2) 地图四角坐标（覆盖所有道路）
**C1 左下**: (-45, -48.7, 0)
**C2 左上**: (-45, 49.2, 0)
**C3 右下**: (120, -48.7, 0)
**C4 右上**: (120, 49.2, 0)

3) 六区域划分方案（六架无人机互不干扰且覆盖所有道路）
将道路外包矩形按 `x` 方向分成左右两列、按 `y` 方向分成上下中三行，共六块。各区域带 2.5 m 的边界冗余避免遗漏道路。

基础边界：
  `xmin = -45`, `xmax = 120`
  `ymin = -48.7`, `ymax = 49.2`
分割参考：
  竖向分割中线 `x_split = 47.5`（介于 `x=45` 与 `x=110`）
  横向分割两条：`y_split1 = -22.5`（介于 `-45` 与 `0`），`y_split2 = 22.5`（介于 `0` 与 `45`）
冗余边带：`margin = 2.5` m

六个区域定义（闭区间，已扩 margin 并截断到整体边界）：
- **R1 左-下**: x ∈ [max(xmin, -45 - margin), min(x_split, 47.5 + margin)] × y ∈ [ymin, min(y_split1, -22.5 + margin)] → 取值范围约 x ∈ [-45, 50.0], y ∈ [-48.7, -20.0]

- **R2 左-中**: x ∈ [-45, 50.0], y ∈ [max(ymin, -22.5 - margin), min(y_split2, 22.5 + margin)] → y ∈ [-25.0, 25.0]

- **R3 左-上**: x ∈ [-45, 50.0], y ∈ [max(y_split2, 22.5 - margin), ymax] → y ∈ [20.0, 49.2]

- **R4 右-下**: x ∈ [max(x_split, 47.5 - margin), xmax] × y ∈ [-48.7, -20.0] → x ∈ [45.0, 120]

- **R5 右-中**: x ∈ [45.0, 120], y ∈ [-25.0, 25.0]

- **R6 右-上**: x ∈ [45.0, 120], y ∈ [20.0, 49.2]

这些区域保证：
- 三条水平路（y=-45,0,45）在上下中三行中分别完全覆盖；
- 三条有效垂直路（x=-15,45,110）在左右两列中被完整覆盖；
- 区域两两不重叠，仅共享边界；可为每架 UAV 选不同高度轨道（例如 5.0 m 和 5.5 m 交错）与反向巡航方向避免边界同时到达，进一步降低碰撞概率。

- 本方案严格依据 `base0.world` 中 `<road>` 定义的坐标构建，确保三条水平路和三条有效垂直路的全覆盖；`x=120` 的垂直路因水平路范围止于 `x=116.3` 不形成路口，但仍在右列区域内被纵向覆盖。


4) 航点规划代码（Python，可直接运行）

```python
import math

# 9 个街道转弯处坐标 (x, y, z)
intersections = [
    (-15, -45, 0.01), (45, -45, 0.01), (110, -45, 0.01),
    (-15, 0, 0.015), (45, 0, 0.015), (110, 0, 0.015),
    (-15, 45, 0.015), (45, 45, 0.015), (110, 45, 0.015)
]

# 地图四角坐标
map_corners = {
    "C1_左下": (-45, -48.7, 0),
    "C2_左上": (-45, 49.2, 0),
    "C3_右下": (120, -48.7, 0),
    "C4_右上": (120, 49.2, 0),
}

# 六个区域边界 (闭区间)
regions = {
    "R1_左下": (-45.0, 50.0, -48.7, -20.0),
    "R2_左中": (-45.0, 50.0, -25.0, 25.0),
    "R3_左上": (-45.0, 50.0, 20.0, 49.2),
    "R4_右下": (45.0, 120.0, -48.7, -20.0),
    "R5_右中": (45.0, 120.0, -25.0, 25.0),
    "R6_右上": (45.0, 120.0, 20.0, 49.2),
}

# 无人机飞行高度 (< 6m)
flight_altitude_m = 5.5

def generate_grid_waypoints(bounds, altitude_m, step_m=20):
    min_x, max_x, min_y, max_y = bounds
    waypoints = []
    # 从边界内侧开始，确保闭区间覆盖
    x_start = math.ceil(min_x)
    x_end = math.floor(max_x)
    y_start = math.ceil(min_y)
    y_end = math.floor(max_y)
    for x in range(x_start, x_end + 1, step_m):
        # 采用蛇形扫描，减少急转弯
        if ((x - x_start) // step_m) % 2 == 0:
            y_iter = range(y_start, y_end + 1, step_m)
        else:
            y_iter = range(y_end, y_start - 1, -step_m)
        for y in y_iter:
            waypoints.append((float(x), float(y), float(altitude_m)))
    return waypoints

def plan_all_regions(regions_dict, altitude_m):
    return {name: generate_grid_waypoints(b, altitude_m) for name, b in regions_dict.items()}

if __name__ == "__main__":
    all_wps = plan_all_regions(regions, flight_altitude_m)
    print("=== base0.world 航点规划 ===")
    print(f"飞行高度: {flight_altitude_m} m")
    for region_name, waypoints in all_wps.items():
        print(f"{region_name}: {len(waypoints)} 个航点，示例前 5 个：")
        for wp in waypoints[:5]:
            print("  ", wp)
        if len(waypoints) > 5:
            print(f"  ... 还有 {len(waypoints) - 5} 个")
```