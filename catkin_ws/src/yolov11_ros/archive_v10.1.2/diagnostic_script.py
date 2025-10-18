#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
诊断脚本 - v10.1.2
用于分析无人机0控制反向问题
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
import tf.transformations as tf_trans
import math

class DiagnosticTool:
    def __init__(self):
        rospy.init_node('diagnostic_tool')
        
        self.drone_data = {}
        
        # 订阅两架无人机的状态
        for drone_id in [0, 1]:
            vehicle_ns = f"typhoon_h480_{drone_id}"
            self.drone_data[drone_id] = {
                'pose': None,
                'state': None,
                'first_yaw': None,
                'first_quaternion': None
            }
            
            # 订阅位姿
            rospy.Subscriber(
                f'/{vehicle_ns}/mavros/local_position/pose',
                PoseStamped,
                lambda msg, id=drone_id: self._pose_callback(msg, id)
            )
            
            # 订阅状态
            rospy.Subscriber(
                f'/{vehicle_ns}/mavros/state',
                State,
                lambda msg, id=drone_id: self._state_callback(msg, id)
            )
    
    def _pose_callback(self, msg, drone_id):
        """位姿回调"""
        self.drone_data[drone_id]['pose'] = msg.pose
        
        # 记录第一次的姿态
        if self.drone_data[drone_id]['first_quaternion'] is None:
            q = msg.pose.orientation
            self.drone_data[drone_id]['first_quaternion'] = (q.x, q.y, q.z, q.w)
            
            # 计算偏航角
            _, _, yaw = tf_trans.euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.drone_data[drone_id]['first_yaw'] = yaw
            
            rospy.loginfo(f"\n{'='*60}")
            rospy.loginfo(f"无人机{drone_id} 初始姿态:")
            rospy.loginfo(f"  位置: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})")
            rospy.loginfo(f"  四元数: x={q.x:.4f}, y={q.y:.4f}, z={q.z:.4f}, w={q.w:.4f}")
            rospy.loginfo(f"  偏航角: {math.degrees(yaw):.2f}度 ({yaw:.4f}弧度)")
            rospy.loginfo(f"{'='*60}\n")
    
    def _state_callback(self, msg, drone_id):
        """状态回调"""
        if self.drone_data[drone_id]['state'] is None:
            self.drone_data[drone_id]['state'] = msg
            rospy.loginfo(f"无人机{drone_id} 状态: mode={msg.mode}, armed={msg.armed}, connected={msg.connected}")
    
    def analyze(self):
        """分析差异"""
        rospy.sleep(2.0)  # 等待数据
        
        rospy.loginfo("\n" + "="*80)
        rospy.loginfo("诊断分析结果:")
        rospy.loginfo("="*80)
        
        # 比较两架无人机的初始偏航角
        if all(self.drone_data[i]['first_yaw'] is not None for i in [0, 1]):
            yaw0 = self.drone_data[0]['first_yaw']
            yaw1 = self.drone_data[1]['first_yaw']
            diff = yaw0 - yaw1
            
            rospy.loginfo(f"\n偏航角对比:")
            rospy.loginfo(f"  无人机0: {math.degrees(yaw0):.2f}度")
            rospy.loginfo(f"  无人机1: {math.degrees(yaw1):.2f}度")
            rospy.loginfo(f"  差值: {math.degrees(diff):.2f}度")
            
            if abs(abs(diff) - math.pi) < 0.1:  # 接近180度
                rospy.logwarn("⚠️ 检测到180度偏航角差异！这可能是控制反向的原因")
            elif abs(diff) < 0.1:  # 几乎相同
                rospy.loginfo("✅ 偏航角基本一致")
            else:
                rospy.logwarn(f"⚠️ 偏航角有{math.degrees(diff):.2f}度差异")
        
        # 比较四元数
        if all(self.drone_data[i]['first_quaternion'] is not None for i in [0, 1]):
            q0 = self.drone_data[0]['first_quaternion']
            q1 = self.drone_data[1]['first_quaternion']
            
            rospy.loginfo(f"\n四元数对比:")
            rospy.loginfo(f"  无人机0: ({q0[0]:.4f}, {q0[1]:.4f}, {q0[2]:.4f}, {q0[3]:.4f})")
            rospy.loginfo(f"  无人机1: ({q1[0]:.4f}, {q1[1]:.4f}, {q1[2]:.4f}, {q1[3]:.4f})")
            
            # 检查是否是相反的四元数（代表相同旋转）
            neg_q0 = (-q0[0], -q0[1], -q0[2], -q0[3])
            if all(abs(neg_q0[i] - q1[i]) < 0.01 for i in range(4)):
                rospy.loginfo("✅ 四元数是相反的（但代表相同旋转）")
            elif all(abs(q0[i] - q1[i]) < 0.01 for i in range(4)):
                rospy.loginfo("✅ 四元数基本相同")
            else:
                rospy.logwarn("⚠️ 四元数有显著差异")
        
        # 诊断建议
        rospy.loginfo(f"\n{'='*80}")
        rospy.loginfo("诊断建议:")
        rospy.loginfo("="*80)
        
        if self.drone_data[0]['first_yaw'] is not None and self.drone_data[1]['first_yaw'] is not None:
            diff = abs(self.drone_data[0]['first_yaw'] - self.drone_data[1]['first_yaw'])
            if abs(diff - math.pi) < 0.1:
                rospy.loginfo("\n🔧 建议解决方案:")
                rospy.loginfo("1. 在waypoint_navigator.py中，为无人机0的偏航角添加180度补偿")
                rospy.loginfo("2. 或者在drone_controller.py中，为无人机0反转X和Y速度")
                rospy.loginfo("3. 最好的方案：检查PX4/Gazebo配置，找出为什么初始姿态不同")
                
                rospy.loginfo("\n📝 参数化方案示例:")
                rospy.loginfo("  drone_0:")
                rospy.loginfo("    yaw_offset: 3.14159  # 180度")
                rospy.loginfo("    velocity_invert_x: true")
                rospy.loginfo("    velocity_invert_y: true")
        
        rospy.loginfo(f"\n{'='*80}")
        rospy.loginfo("诊断完成")
        rospy.loginfo("="*80)
    
    def run(self):
        """运行诊断"""
        rospy.loginfo("="*80)
        rospy.loginfo("无人机控制反向问题诊断工具 v10.1.2")
        rospy.loginfo("="*80)
        rospy.loginfo("正在收集数据...")
        
        # 分析
        rospy.Timer(rospy.Duration(3.0), lambda e: self.analyze(), oneshot=True)
        
        # 保持运行
        rospy.spin()

if __name__ == '__main__':
    try:
        tool = DiagnosticTool()
        tool.run()
    except rospy.ROSInterruptException:
        pass
