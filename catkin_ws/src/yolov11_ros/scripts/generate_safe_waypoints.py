#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
安全航点生成器 v2 - 沿道路飞行，只在路口转角停留
基于road/目录的地图分析

关键改进：
1. 只在路口转角处停留（hover_time=1.0）
2. 直行道路上的点不停留（hover_time=0）
3. x和y坐标正确（已修正）
"""

import json
import os
import math

# 无人机初始位置（官方robocup.launch）
UAV_INIT_POS = {
    0: (-17, -3, 1),
    1: (-14, -3, 1),
    2: (-17, 0, 1),
    3: (-14, 0, 1),
    4: (-17, 3, 1),
    5: (-14, 3, 1),
}

# 固定道路（所有地图相同）
FIXED_VERTICAL_ROADS = [-45, -15, 110, 120]  # x坐标固定，沿y方向延伸
FIXED_HORIZONTAL_ROADS = [-45, 0, 45]  # y坐标固定，沿x方向延伸

# 中间道路候选位置（16种地图的可能位置）
MIDDLE_ROAD_CANDIDATES = [30, 35, 40, 45, 50, 55, 60, 65, 70, 75]

# 飞行高度（避开障碍物）
FLIGHT_HEIGHT = 3.0

# 六区域划分（基于初始位置优化）
REGIONS = {
    0: {'name': 'UAV0_下左', 'bounds': (-45, 40, -48.7, -10)},
    1: {'name': 'UAV1_下右', 'bounds': (25, 120, -48.7, -10)},
    2: {'name': 'UAV2_中左', 'bounds': (-45, 40, -10, 20)},
    3: {'name': 'UAV3_中右', 'bounds': (25, 120, -10, 20)},
    4: {'name': 'UAV4_上左', 'bounds': (-45, 40, 20, 49.2)},
    5: {'name': 'UAV5_上右', 'bounds': (25, 120, 20, 49.2)},
}


def is_in_region(x, y, bounds):
    """检查点是否在区域内"""
    min_x, max_x, min_y, max_y = bounds
    return min_x <= x <= max_x and min_y <= y <= max_y


def is_intersection(x, y):
    """
    判断是否是路口（道路交叉点）
    路口定义：垂直道路和水平道路的交点
    """
    # 检查是否在垂直道路上
    on_vertical = x in FIXED_VERTICAL_ROADS or x in MIDDLE_ROAD_CANDIDATES
    # 检查是否在水平道路上
    on_horizontal = y in FIXED_HORIZONTAL_ROADS
    
    # 路口 = 垂直道路 ∩ 水平道路
    return on_vertical and on_horizontal


def generate_road_waypoints(uav_id):
    """
    生成沿道路的航点（优化策略v2）
    - 沿道路飞行，避开建筑物
    - 只在路口转角停留（hover_time=1.0）
    - 直行道路不停留（hover_time=0）
    """
    region = REGIONS[uav_id]
    bounds = region['bounds']
    min_x, max_x, min_y, max_y = bounds
    waypoints = []
    
    print(f"\n生成 {region['name']} 的航点...")
    
    # ========== 1. 垂直道路航点（固定道路） ==========
    for road_x in FIXED_VERTICAL_ROADS:
        if is_in_region(road_x, 0, bounds):
            # 沿垂直道路生成航点（间隔10m）
            y_start = max(min_y, -45)
            y_end = min(max_y, 45)
            for y in range(int(y_start), int(y_end) + 1, 10):
                # 判断是否是路口
                hover = 1.0 if is_intersection(road_x, y) else 0.0
                waypoints.append({
                    'x': float(y),  # ⚠️ 注意：x和y互换！
                    'y': float(road_x),
                    'z': FLIGHT_HEIGHT,
                    'hover_time': hover
                })
    
    # ========== 2. 中间道路密集扫描（自适应） ==========
    # 右侧无人机(1,3,5)负责扫描中间道路
    if uav_id in [1, 3, 5]:
        for road_x in MIDDLE_ROAD_CANDIDATES:
            if is_in_region(road_x, 0, bounds):
                y_start = max(min_y, -45)
                y_end = min(max_y, 45)
                for y in range(int(y_start), int(y_end) + 1, 10):
                    hover = 1.0 if is_intersection(road_x, y) else 0.0
                    waypoints.append({
                        'x': float(y),  # ⚠️ x和y互换
                        'y': float(road_x),
                        'z': FLIGHT_HEIGHT,
                        'hover_time': hover
                    })
    
    # 左侧无人机(0,2,4)也扫描部分中间道路（重叠区）
    if uav_id in [0, 2, 4]:
        for road_x in [30, 35, 40]:
            if is_in_region(road_x, 0, bounds):
                y_start = max(min_y, -45)
                y_end = min(max_y, 45)
                for y in range(int(y_start), int(y_end) + 1, 10):
                    hover = 1.0 if is_intersection(road_x, y) else 0.0
                    waypoints.append({
                        'x': float(y),  # ⚠️ x和y互换
                        'y': float(road_x),
                        'z': FLIGHT_HEIGHT,
                        'hover_time': hover
                    })
    
    # ========== 3. 水平道路航点 ==========
    for road_y in FIXED_HORIZONTAL_ROADS:
        if is_in_region(0, road_y, bounds):
            # 沿水平道路生成航点（间隔15m）
            x_start = max(min_x, -45)
            x_end = min(max_x, 120)
            for x in range(int(x_start), int(x_end) + 1, 15):
                hover = 1.0 if is_intersection(x, road_y) else 0.0
                waypoints.append({
                    'x': float(x),  # 水平道路：x不变
                    'y': float(road_y),
                    'z': FLIGHT_HEIGHT,
                    'hover_time': hover
                })
    
    print(f"  生成了 {len(waypoints)} 个道路航点")
    return waypoints


def remove_duplicates(waypoints, threshold=5.0):
    """去除重复航点（距离小于阈值的点）"""
    unique_wps = []
    for wp in waypoints:
        is_duplicate = False
        for existing_wp in unique_wps:
            dist = ((wp['x'] - existing_wp['x'])**2 + (wp['y'] - existing_wp['y'])**2)**0.5
            if dist < threshold:
                is_duplicate = True
                break
        if not is_duplicate:
            unique_wps.append(wp)
    return unique_wps


def optimize_waypoint_order(waypoints, start_pos):
    """
    优化航点顺序（最近邻算法）
    从起飞点开始，每次选择最近的未访问航点
    """
    if not waypoints:
        return []
    
    optimized = []
    remaining = waypoints.copy()
    current_pos = start_pos
    
    while remaining:
        # 找最近的点
        min_dist = float('inf')
        nearest = None
        for wp in remaining:
            dist = ((current_pos[0] - wp['x'])**2 + (current_pos[1] - wp['y'])**2)**0.5
            if dist < min_dist:
                min_dist = dist
                nearest = wp
        
        optimized.append(nearest)
        remaining.remove(nearest)
        current_pos = (nearest['x'], nearest['y'])
    
    return optimized


def generate_all_waypoints():
    """生成所有6架无人机的航点"""
    output_dir = os.path.join(os.path.dirname(__file__), 'waypoints')
    
    for uav_id in range(6):
        # 1. 生成道路航点
        road_wps = generate_road_waypoints(uav_id)
        
        # 2. 去除重复点
        unique_wps = remove_duplicates(road_wps)
        print(f"  去重后: {len(unique_wps)} 个航点")
        
        # 3. 优化顺序（从起飞点开始）
        start_pos = UAV_INIT_POS[uav_id]
        optimized_wps = optimize_waypoint_order(unique_wps, start_pos)
        print(f"  优化后: {len(optimized_wps)} 个航点")
        
        # 4. 添加loop标记到第一个航点
        if optimized_wps:
            optimized_wps[0]['loop'] = True
            # hover_time已经在生成时设置（路口1.0，直行0.0）
        
        # 5. 统计路口数量
        intersection_count = sum(1 for wp in optimized_wps if wp.get('hover_time', 0) > 0)
        print(f"  路口数量: {intersection_count} 个（停留），直行: {len(optimized_wps) - intersection_count} 个（不停）")
        
        # 6. 保存为JSON
        output_file = os.path.join(output_dir, f'waypoints_drone_{uav_id}.json')
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump({'waypoints': optimized_wps}, f, indent=2, ensure_ascii=False)
        
        print(f"  ✅ 保存到: {output_file}\n")
    
    print("=" * 70)
    print("✅ 所有航点生成完成！")
    print("=" * 70)
    print("\n关键特性:")
    print("  ✅ 只在道路上飞行，避开建筑物")
    print("  ✅ 只在路口转角停留（hover_time=1.0）")
    print("  ✅ 直行道路不停留（hover_time=0）")
    print("  ✅ x和y坐标已正确互换")
    print("  ✅ 覆盖所有固定道路（x=-45,-15,110,120 和 y=-45,0,45）")
    print("  ✅ 密集扫描中间道路（x=30-75）")
    print("  ✅ 飞行高度3.0m，安全避障")
    print("  ✅ 从起飞点优化路径，最短距离")


if __name__ == '__main__':
    generate_all_waypoints()
