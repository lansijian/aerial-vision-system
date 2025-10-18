#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
多无人机航点规划器
功能：
1. 为N架无人机生成巡检航点
2. 无人机0使用正向航点，无人机1使用反向航点
3. 支持2-6架无人机的扩展
4. 基于原有航点数据，确保覆盖完整区域
"""

import json
import argparse
import os
from typing import List, Dict

class MultiDroneWaypointPlanner:
    """多无人机航点规划器"""
    
    def __init__(self, num_drones: int = 2, height: float = 3.0):
        """
        初始化规划器
        
        Args:
            num_drones: 无人机数量（2-6架）
            height: 飞行高度（固定3米）
        """
        self.num_drones = min(max(num_drones, 2), 6)  # 限制在2-6架
        self.height = height
        
        # 基础航点 - 基于原有的航点数据，完整覆盖搜索区域
        self.base_waypoints = [
            {'x': 0, 'y': 0, 'z': self.height},
            {'x': 105, 'y': 0, 'z': self.height},
            {'x': 105, 'y': 45, 'z': self.height},
            {'x': 45, 'y': 45, 'z': self.height},
            {'x': -45, 'y': 45, 'z': self.height},
            {'x': -45, 'y': 0, 'z': self.height},
            {'x': -15, 'y': 0, 'z': self.height},
            {'x': -15, 'y': -45, 'z': self.height},
            {'x': 45, 'y': -45, 'z': self.height},
            {'x': 45, 'y': 45, 'z': self.height},
            {'x': -15, 'y': 45, 'z': self.height},
            {'x': -15, 'y': 0, 'z': self.height},
            {'x': -45, 'y': 0, 'z': self.height},
            {'x': -45, 'y': -45, 'z': self.height},
            {'x': 105, 'y': -45, 'z': self.height},
            {'x': 105, 'y': 45, 'z': self.height},
            {'x': 45, 'y': 45, 'z': self.height},
            {'x': 45, 'y': -45, 'z': self.height},
            {'x': -15, 'y': -45, 'z': self.height},
            {'x': -15, 'y': 0, 'z': self.height},
            {'x': 0, 'y': 0, 'z': self.height}
        ]
        
        print(f"🚁 多机航点规划器初始化")
        print(f"   无人机数量: {self.num_drones}")
        print(f"   飞行高度: {self.height}m")
        print(f"   基础航点数: {len(self.base_waypoints)}")
    
    def generate_drone_waypoints(self, drone_id: int) -> List[Dict]:
        """
        为指定无人机生成航点
        
        Args:
            drone_id: 无人机ID（0开始）
            
        Returns:
            航点列表
        """
        if drone_id == 0:
            # 无人机0：正向航点（原始顺序）
            waypoints = self.base_waypoints.copy()
            print(f"   无人机{drone_id}: 正向航点 ({len(waypoints)}个)")
            
        elif drone_id == 1:
            # 无人机1：反向航点（逆序）
            waypoints = list(reversed(self.base_waypoints.copy()))
            print(f"   无人机{drone_id}: 反向航点 ({len(waypoints)}个)")
            
        elif drone_id < self.num_drones:
            # 无人机2-5：根据ID分配不同起始点的正向或反向
            # 偶数ID正向，奇数ID反向
            if drone_id % 2 == 0:
                waypoints = self.base_waypoints.copy()
                # 循环偏移起始点
                offset = (drone_id // 2) * (len(self.base_waypoints) // 3)
                waypoints = waypoints[offset:] + waypoints[:offset]
                print(f"   无人机{drone_id}: 正向航点(偏移{offset}) ({len(waypoints)}个)")
            else:
                waypoints = list(reversed(self.base_waypoints.copy()))
                offset = ((drone_id - 1) // 2) * (len(self.base_waypoints) // 3)
                waypoints = waypoints[offset:] + waypoints[:offset]
                print(f"   无人机{drone_id}: 反向航点(偏移{offset}) ({len(waypoints)}个)")
        else:
            # 超出范围，返回空列表
            waypoints = []
            print(f"   ⚠️ 无人机{drone_id}: 超出支持范围")
        
        return waypoints
    
    def save_waypoints(self, output_dir: str = "waypoints"):
        """
        保存所有无人机的航点到文件
        
        Args:
            output_dir: 输出目录
        """
        # 创建输出目录
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            print(f"📁 创建输出目录: {output_dir}")
        
        summary = {
            "num_drones": self.num_drones,
            "height": self.height,
            "total_base_waypoints": len(self.base_waypoints),
            "drones": []
        }
        
        # 为每架无人机生成并保存航点
        for drone_id in range(self.num_drones):
            waypoints = self.generate_drone_waypoints(drone_id)
            
            # 保存航点文件
            filename = os.path.join(output_dir, f"waypoints_drone_{drone_id}.json")
            waypoint_data = {
                "drone_id": drone_id,
                "num_waypoints": len(waypoints),
                "height": self.height,
                "waypoints": waypoints
            }
            
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(waypoint_data, f, indent=2, ensure_ascii=False)
            
            print(f"✅ 保存: {filename}")
            
            # 添加到摘要
            summary["drones"].append({
                "drone_id": drone_id,
                "waypoint_file": f"waypoints_drone_{drone_id}.json",
                "num_waypoints": len(waypoints),
                "direction": "forward" if drone_id == 0 else "reverse" if drone_id == 1 else "offset"
            })
        
        # 保存摘要文件
        summary_file = os.path.join(output_dir, "waypoints_summary.json")
        with open(summary_file, 'w', encoding='utf-8') as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)
        
        print(f"📋 航点规划摘要已保存: {summary_file}")
        print(f"🎯 共生成 {self.num_drones} 架无人机的航点数据")
    
    def visualize_waypoints(self):
        """可视化航点分布（控制台输出）"""
        print("\n" + "="*60)
        print("航点分布可视化")
        print("="*60)
        
        for drone_id in range(min(self.num_drones, 2)):  # 只显示前两架
            waypoints = self.generate_drone_waypoints(drone_id)
            print(f"\n无人机 {drone_id} 航点列表:")
            print(f"{'序号':<6} {'X坐标':<10} {'Y坐标':<10} {'Z坐标':<10}")
            print("-" * 40)
            
            # 显示前5个和后5个航点
            display_count = min(5, len(waypoints))
            for i in range(display_count):
                wp = waypoints[i]
                print(f"{i:<6} {wp['x']:<10.1f} {wp['y']:<10.1f} {wp['z']:<10.1f}")
            
            if len(waypoints) > 10:
                print("  ... ...")
                for i in range(len(waypoints) - display_count, len(waypoints)):
                    wp = waypoints[i]
                    print(f"{i:<6} {wp['x']:<10.1f} {wp['y']:<10.1f} {wp['z']:<10.1f}")
        
        print("="*60 + "\n")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="多无人机航点规划器")
    parser.add_argument('--num_drones', type=int, default=2, 
                       help='无人机数量（2-6架，默认2）')
    parser.add_argument('--height', type=float, default=3.0,
                       help='飞行高度（米，默认3.0）')
    parser.add_argument('--output_dir', type=str, default='waypoints',
                       help='输出目录（默认: waypoints）')
    parser.add_argument('--visualize', action='store_true',
                       help='显示航点可视化')
    
    args = parser.parse_args()
    
    # 创建规划器
    planner = MultiDroneWaypointPlanner(
        num_drones=args.num_drones,
        height=args.height
    )
    
    # 生成并保存航点
    planner.save_waypoints(output_dir=args.output_dir)
    
    # 可视化
    if args.visualize:
        planner.visualize_waypoints()
    
    print("\n✅ 航点规划完成！")
    print(f"📂 航点文件位置: {args.output_dir}/")
    print(f"🚁 无人机数量: {args.num_drones}")
    print(f"📍 飞行高度: {args.height}m")


if __name__ == '__main__':
    main()
