#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""地图分析工具 - 提取所有房屋和道路位置
用于规划安全的无人机巡航路线
"""

import re
import os
from collections import defaultdict

def parse_world_file(world_file):
    """解析world文件，提取房屋和道路位置"""
    with open(world_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 提取房屋位置
    houses = []
    house_pattern = r"<model name='(house_[^']+)'.*?<pose frame=''>([^<]+)</pose>"
    for match in re.finditer(house_pattern, content, re.DOTALL):
        name = match.group(1)
        pose_str = match.group(2)
        coords = pose_str.strip().split()
        if len(coords) >= 3:
            x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
            houses.append({'name': name, 'x': x, 'y': y, 'z': z})
    
    # 提取道路信息
    roads = []
    road_pattern = r"<road name='([^']+)'>.*?<width>([^<]+)</width>.*?<point>([^<]+)</point>.*?<point>([^<]+)</point>"
    for match in re.finditer(road_pattern, content, re.DOTALL):
        name = match.group(1)
        width = float(match.group(2))
        point1 = [float(x) for x in match.group(3).strip().split()]
        point2 = [float(x) for x in match.group(4).strip().split()]
        roads.append({
            'name': name,
            'width': width,
            'point1': point1,
            'point2': point2
        })
    
    return houses, roads

def analyze_all_maps():
    """分析所有地图"""
    world_dir = r"e:\robocup2025\catkin_ws\src\yolov11_ros\world"
    
    all_maps_data = {}
    
    for i in range(17):  # base0 到 base16
        world_file = os.path.join(world_dir, f"base{i}.world")
        if os.path.exists(world_file):
            print(f"\n{'='*80}")
            print(f"分析地图: base{i}.world")
            print(f"{'='*80}")
            
            houses, roads = parse_world_file(world_file)
            
            print(f"\n房屋数量: {len(houses)}")
            print(f"道路数量: {len(roads)}")
            
            # 统计房屋分布
            if houses:
                x_coords = [h['x'] for h in houses]
                y_coords = [h['y'] for h in houses]
                print(f"\n房屋X坐标范围: [{min(x_coords):.1f}, {max(x_coords):.1f}]")
                print(f"房屋Y坐标范围: [{min(y_coords):.1f}, {max(y_coords):.1f}]")
                
                print(f"\n前10个房屋位置:")
                for house in houses[:10]:
                    print(f"  {house['name']:20s} ({house['x']:7.2f}, {house['y']:7.2f})")
            
            # 统计道路
            if roads:
                print(f"\n道路信息:")
                for road in roads[:10]:
                    p1, p2 = road['point1'], road['point2']
                    print(f"  {road['name']:15s} 宽度:{road['width']:.1f}m  "
                          f"从({p1[0]:.1f},{p1[1]:.1f}) 到({p2[0]:.1f},{p2[1]:.1f})")
            
            all_maps_data[f"base{i}"] = {
                'houses': houses,
                'roads': roads
            }
    
    return all_maps_data

def find_safe_corridors(houses):
    """找出安全的飞行走廊（避开房屋区域）"""
    if not houses:
        return []
    
    # 获取所有房屋的边界
    x_coords = sorted(set([h['x'] for h in houses]))
    y_coords = sorted(set([h['y'] for h in houses]))
    
    # 假设房屋大小约为10x10m
    house_size = 10
    safe_margin = 5  # 安全边距
    
    # 找出X方向的安全走廊
    safe_x_corridors = []
    for i in range(len(x_coords) - 1):
        gap = x_coords[i+1] - x_coords[i]
        if gap > house_size + 2 * safe_margin:
            corridor_x = (x_coords[i] + x_coords[i+1]) / 2
            safe_x_corridors.append(corridor_x)
    
    # 找出Y方向的安全走廊
    safe_y_corridors = []
    for i in range(len(y_coords) - 1):
        gap = y_coords[i+1] - y_coords[i]
        if gap > house_size + 2 * safe_margin:
            corridor_y = (y_coords[i] + y_coords[i+1]) / 2
            safe_y_corridors.append(corridor_y)
    
    return safe_x_corridors, safe_y_corridors

if __name__ == "__main__":
    print("="*80)
    print("RoboCup 2025 地图分析工具")
    print("="*80)
    
    all_maps = analyze_all_maps()
    
    print(f"\n\n{'='*80}")
    print("总结")
    print(f"{'='*80}")
    print(f"共分析 {len(all_maps)} 个地图")
    
    # 找出通用的安全区域
    print(f"\n建议的安全飞行策略:")
    print(f"1. 起飞位置: (-17, -3), (-14, -3), (-17, 0), (-14, 0), (-17, 3), (-14, 3)")
    print(f"2. 巡航高度: 3.0m (避免与建筑物碰撞)")
    print(f"3. 安全边距: 距离建筑物至少5m")
    print(f"4. 优先使用道路上空飞行")
