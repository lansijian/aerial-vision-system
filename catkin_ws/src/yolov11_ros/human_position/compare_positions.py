#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
实时对比Actor真值位置和检测估计位置
用于验证人体位置检测算法的准确性
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped
import sys
from collections import deque

class PositionComparator:
    def __init__(self, vehicle_type='typhoon_h480', vehicle_id='0'):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        
        # Actor真值位置
        self.ground_truth_pos = None
        self.ground_truth_time = None
        
        # 检测估计位置
        self.estimated_pos = None
        self.estimated_time = None
        
        # 统计数据（用于计算平均误差）
        self.error_history = deque(maxlen=100)  # 保存最近100个误差值
        
        print("=" * 90)
        print("  位置对比工具 - Actor真值 vs 检测估计")
        print("=" * 90)
        print("  车辆: %s_%s" % (vehicle_type, vehicle_id))
        print("=" * 90)
        print()
        
        # 订阅Actor真值
        rospy.Subscriber(
            '/actor/ground_truth/position',
            PointStamped,
            self.ground_truth_callback,
            queue_size=10
        )
        
        # 订阅检测估计值
        topic_name = '/xtdrone/' + vehicle_type + '_' + vehicle_id + '/human_position_local'
        rospy.Subscriber(
            topic_name,
            PointStamped,
            self.estimated_callback,
            queue_size=10
        )
        
        print("[就绪] 等待数据...")
        print("  真值话题: /actor/ground_truth/position")
        print("  估计话题: %s" % topic_name)
        print()
        print("-" * 90)
        print()
    
    def ground_truth_callback(self, msg):
        """接收Actor真值位置"""
        self.ground_truth_pos = msg.point
        self.ground_truth_time = msg.header.stamp
    
    def estimated_callback(self, msg):
        """接收检测估计位置"""
        self.estimated_pos = msg.point
        self.estimated_time = msg.header.stamp
    
    def calculate_error(self):
        """计算位置误差"""
        if self.ground_truth_pos is None or self.estimated_pos is None:
            return None
        
        # 各轴误差
        dx = self.estimated_pos.x - self.ground_truth_pos.x
        dy = self.estimated_pos.y - self.ground_truth_pos.y
        dz = self.estimated_pos.z - self.ground_truth_pos.z
        
        # 欧氏距离（3D误差）
        distance_3d = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # 水平距离（2D误差，忽略Z轴）
        distance_2d = math.sqrt(dx**2 + dy**2)
        
        return {
            'dx': dx,
            'dy': dy,
            'dz': dz,
            'distance_3d': distance_3d,
            'distance_2d': distance_2d
        }
    
    def calculate_statistics(self):
        """计算统计数据"""
        if len(self.error_history) == 0:
            return None
        
        errors_3d = [e['distance_3d'] for e in self.error_history]
        errors_2d = [e['distance_2d'] for e in self.error_history]
        
        return {
            'mean_3d': np.mean(errors_3d),
            'std_3d': np.std(errors_3d),
            'max_3d': np.max(errors_3d),
            'min_3d': np.min(errors_3d),
            'mean_2d': np.mean(errors_2d),
            'std_2d': np.std(errors_2d),
            'samples': len(errors_3d)
        }
    
    def print_comparison(self):
        """打印对比结果"""
        if self.ground_truth_pos is None:
            print("\r[等待] 未收到Actor真值数据...              ", end='')
            sys.stdout.flush()
            return
        
        if self.estimated_pos is None:
            print("\r[等待] 未收到检测估计数据...              ", end='')
            sys.stdout.flush()
            return
        
        # 计算误差
        error = self.calculate_error()
        if error is None:
            return
        
        # 保存误差到历史记录
        self.error_history.append(error)
        
        # 计算统计数据
        stats = self.calculate_statistics()
        
        # 清屏并重新打印（使用ANSI转义码）
        # print("\033[2J\033[H", end='')  # 清屏
        
        # 打印标题
        print("\r" + "=" * 90)
        print("  实时位置对比 (ENU坐标系)")
        print("=" * 90)
        
        # 打印真值
        print("  【Actor真值】")
        print("    X: %8.3f m  |  Y: %8.3f m  |  Z: %8.3f m" % (
            self.ground_truth_pos.x,
            self.ground_truth_pos.y,
            self.ground_truth_pos.z
        ))
        
        # 打印估计值
        print()
        print("  【检测估计】")
        print("    X: %8.3f m  |  Y: %8.3f m  |  Z: %8.3f m" % (
            self.estimated_pos.x,
            self.estimated_pos.y,
            self.estimated_pos.z
        ))
        
        # 打印误差
        print()
        print("  【位置误差】")
        print("    ΔX: %+7.3f m  |  ΔY: %+7.3f m  |  ΔZ: %+7.3f m" % (
            error['dx'], error['dy'], error['dz']
        ))
        
        # 打印距离误差
        print()
        print("  【距离误差】")
        print("    2D误差: %7.3f m  |  3D误差: %7.3f m" % (
            error['distance_2d'], error['distance_3d']
        ))
        
        # 打印统计数据
        if stats and stats['samples'] >= 10:
            print()
            print("  【统计数据】(最近%d个样本)" % stats['samples'])
            print("    3D误差: 平均=%.3f m, 标准差=%.3f m, 最大=%.3f m, 最小=%.3f m" % (
                stats['mean_3d'], stats['std_3d'], stats['max_3d'], stats['min_3d']
            ))
            print("    2D误差: 平均=%.3f m, 标准差=%.3f m" % (
                stats['mean_2d'], stats['std_2d']
            ))
        
        # 打印精度评估
        print()
        print("  【精度评估】")
        if error['distance_3d'] < 0.5:
            assessment = "★★★★★ 优秀"
        elif error['distance_3d'] < 1.0:
            assessment = "★★★★☆ 良好"
        elif error['distance_3d'] < 2.0:
            assessment = "★★★☆☆ 一般"
        elif error['distance_3d'] < 5.0:
            assessment = "★★☆☆☆ 较差"
        else:
            assessment = "★☆☆☆☆ 很差"
        print("    当前精度: %s (3D误差: %.3f m)" % (assessment, error['distance_3d']))
        
        print("=" * 90)
        print()
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(2)  # 2Hz刷新显示
        
        print("开始实时对比...")
        print("按 Ctrl+C 停止")
        print()
        
        while not rospy.is_shutdown():
            self.print_comparison()
            rate.sleep()
        
        # 退出时打印最终统计
        print()
        print("=" * 90)
        print("  最终统计报告")
        print("=" * 90)
        
        stats = self.calculate_statistics()
        if stats and stats['samples'] > 0:
            print()
            print("  总样本数: %d" % stats['samples'])
            print()
            print("  3D误差统计:")
            print("    平均值: %.4f m" % stats['mean_3d'])
            print("    标准差: %.4f m" % stats['std_3d'])
            print("    最大值: %.4f m" % stats['max_3d'])
            print("    最小值: %.4f m" % stats['min_3d'])
            print()
            print("  2D误差统计:")
            print("    平均值: %.4f m" % stats['mean_2d'])
            print("    标准差: %.4f m" % stats['std_2d'])
            print()
            
            # 计算精度等级分布
            excellent = sum(1 for e in self.error_history if e['distance_3d'] < 0.5)
            good = sum(1 for e in self.error_history if 0.5 <= e['distance_3d'] < 1.0)
            fair = sum(1 for e in self.error_history if 1.0 <= e['distance_3d'] < 2.0)
            poor = sum(1 for e in self.error_history if 2.0 <= e['distance_3d'] < 5.0)
            bad = sum(1 for e in self.error_history if e['distance_3d'] >= 5.0)
            
            print("  精度分布:")
            print("    优秀 (<0.5m):  %3d个 (%.1f%%)" % (excellent, 100.0*excellent/stats['samples']))
            print("    良好 (<1.0m):  %3d个 (%.1f%%)" % (good, 100.0*good/stats['samples']))
            print("    一般 (<2.0m):  %3d个 (%.1f%%)" % (fair, 100.0*fair/stats['samples']))
            print("    较差 (<5.0m):  %3d个 (%.1f%%)" % (poor, 100.0*poor/stats['samples']))
            print("    很差 (>=5.0m): %3d个 (%.1f%%)" % (bad, 100.0*bad/stats['samples']))
            
        print()
        print("=" * 90)


if __name__ == "__main__":
    # 解析参数
    if len(sys.argv) >= 3:
        vehicle_type = sys.argv[1]
        vehicle_id = sys.argv[2]
    else:
        # 默认值
        vehicle_type = 'typhoon_h480'
        vehicle_id = '0'
        print("[提示] 未指定车辆，使用默认: %s_%s" % (vehicle_type, vehicle_id))
        print("[用法] python compare_positions.py <vehicle_type> <vehicle_id>")
        print()
    
    # 初始化节点
    rospy.init_node('position_comparator')
    
    # 创建对比器
    comparator = PositionComparator(vehicle_type, vehicle_id)
    
    try:
        comparator.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print()
        print("[退出] 用户中断")

