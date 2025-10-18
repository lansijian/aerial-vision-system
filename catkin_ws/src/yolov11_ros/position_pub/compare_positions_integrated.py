#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
集成式位置对比工具
自动获取Actor真值位置并与检测估计位置进行实时对比
无需手动运行真值发布器
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from gazebo_msgs.srv import GetModelState, GetLinkState
from gazebo_msgs.msg import ModelStates
import sys
from collections import deque
import tf2_ros


class ActorGroundTruthPublisher:
    """Actor真值位置发布器（后台运行，无输出）"""
    
    def __init__(self, actor_name='actor', publish_rate=30, use_service=True):
        self.actor_name = actor_name
        self.publish_rate = publish_rate
        self.use_service = use_service
        
        # 当前位置
        self.current_position = None
        self.current_orientation = None
        
        # 发布器
        self.point_pub = rospy.Publisher(
            '/actor/ground_truth/position',
            PointStamped,
            queue_size=10
        )
        
        self.pose_pub = rospy.Publisher(
            '/actor/ground_truth/pose',
            PoseStamped,
            queue_size=10
        )
        
        # TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        if use_service:
            self.setup_service()
        else:
            self.setup_subscriber()
    
    def setup_service(self):
        """使用Gazebo服务获取位置"""
        try:
            rospy.wait_for_service('/gazebo/get_model_state', timeout=5.0)
            self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            
            try:
                rospy.wait_for_service('/gazebo/get_link_state', timeout=2.0)
                self.get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
                self.use_link_state = True
            except:
                self.use_link_state = False
                
        except rospy.ROSException:
            rospy.logerr("[错误] Gazebo服务不可用，请确保仿真正在运行")
            sys.exit(1)
    
    def setup_subscriber(self):
        """订阅Gazebo话题获取位置"""
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)
    
    def model_states_callback(self, msg):
        """ModelStates话题回调"""
        try:
            index = msg.name.index(self.actor_name)
            self.current_position = msg.pose[index].position
            self.current_orientation = msg.pose[index].orientation
        except ValueError:
            pass  # 静默处理
    
    def get_actor_pose_from_service(self):
        """通过服务获取actor位姿"""
        try:
            if self.use_link_state:
                link_name = self.actor_name + '::actor_pose'
                response = self.get_link_state(link_name, '')
                return response.link_state.pose
            else:
                response = self.get_model_state(self.actor_name, '')
                return response.pose
        except:
            return None
    
    def publish_actor_pose(self):
        """发布actor位置（静默）"""
        if self.use_service:
            pose = self.get_actor_pose_from_service()
            if pose is None:
                return
            position = pose.position
            orientation = pose.orientation
        else:
            if self.current_position is None:
                return
            position = self.current_position
            orientation = self.current_orientation
        
        stamp = rospy.Time.now()
        
        # 发布PointStamped
        point_msg = PointStamped()
        point_msg.header.stamp = stamp
        point_msg.header.frame_id = "map"
        point_msg.point = position
        self.point_pub.publish(point_msg)
        
        # 发布PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position = position
        pose_msg.pose.orientation = orientation
        self.pose_pub.publish(pose_msg)
        
        # 发布TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = "actor_ground_truth"
        tf_msg.transform.translation.x = position.x
        tf_msg.transform.translation.y = position.y
        tf_msg.transform.translation.z = position.z
        tf_msg.transform.rotation = orientation
        self.tf_broadcaster.sendTransform(tf_msg)


class PositionComparator:
    """位置对比器"""
    
    def __init__(self, vehicle_type='typhoon_h480', vehicle_id='0', actor_name='actor'):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        
        # Actor真值位置
        self.ground_truth_pos = None
        self.ground_truth_time = None
        
        # 检测估计位置
        self.estimated_pos = None
        self.estimated_time = None
        
        # 【修复】数据过期检测
        self.data_timeout = 1.0  # 1秒无数据视为过期
        
        # 统计数据
        self.error_history = deque(maxlen=100)
        
        # 低误差持续时间跟踪
        self.low_error_tracking = False
        self.low_error_start_time = None
        self.low_error_threshold = 1.0
        
        print("=" * 90)
        print("  集成式位置对比工具 - Actor真值 vs 检测估计")
        print("=" * 90)
        print("  飞行器: %s_%s" % (vehicle_type, vehicle_id))
        print("  Actor名称: %s" % actor_name)
        print("=" * 90)
        print()
        
        # 创建真值发布器（后台运行）
        print("[初始化] 启动Actor真值发布器...")
        self.truth_publisher = ActorGroundTruthPublisher(
            actor_name=actor_name,
            publish_rate=30,
            use_service=True
        )
        print("[✓] 真值发布器已启动")
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
        
        # 时间戳同步检查
        if self.ground_truth_time is not None and self.estimated_time is not None:
            time_diff = abs((self.ground_truth_time - self.estimated_time).to_sec())
            if time_diff > 0.5:
                rospy.logwarn_throttle(5.0, 
                    "[警告] 时间戳不同步！时间差: %.2f秒" % time_diff)
        
        # 各轴误差
        dx = self.estimated_pos.x - self.ground_truth_pos.x
        dy = self.estimated_pos.y - self.ground_truth_pos.y
        dz = self.estimated_pos.z - self.ground_truth_pos.z
        
        # 欧氏距离
        distance_3d = math.sqrt(dx**2 + dy**2 + dz**2)
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
        current_time = rospy.Time.now()
        
        if self.ground_truth_pos is None:
            rospy.loginfo_throttle(2.0, "[等待] 未收到Actor真值数据...")
            return
        
        if self.estimated_pos is None:
            rospy.loginfo_throttle(2.0, "[等待] 未收到检测估计数据...")
            return
        
        # 【修复】检查估计数据是否过期
        if self.estimated_time is not None:
            time_since_last_estimate = (current_time - self.estimated_time).to_sec()
            if time_since_last_estimate > self.data_timeout:
                rospy.logwarn_throttle(2.0, 
                    "[警告] 检测估计数据已过期！最后收到数据: %.1f秒前" % time_since_last_estimate)
                rospy.logwarn_throttle(2.0, 
                    "[提示] 位置发布器可能已停止，请检查发布器是否正在运行")
                # 停止低误差跟踪
                if self.low_error_tracking:
                    rospy.loginfo("[停止] 低误差跟踪已停止（数据过期）")
                    self.low_error_tracking = False
                    self.low_error_start_time = None
                return
        
        # 计算误差
        error = self.calculate_error()
        if error is None:
            return
        
        # 保存误差到历史记录
        self.error_history.append(error)
        
        # 计算统计数据
        stats = self.calculate_statistics()
        
        # 检查是否低误差
        is_low_error = (abs(error['dx']) < self.low_error_threshold and 
                       abs(error['dy']) < self.low_error_threshold)
        
        # 更新低误差跟踪状态
        low_error_duration = 0.0
        
        if is_low_error:
            if not self.low_error_tracking:
                self.low_error_tracking = True
                self.low_error_start_time = current_time
                rospy.loginfo("[✓] 开始低误差跟踪: |ΔX|<%.1f米, |ΔY|<%.1f米" % 
                             (self.low_error_threshold, self.low_error_threshold))
            
            if self.low_error_start_time is not None:
                low_error_duration = (current_time - self.low_error_start_time).to_sec()
        else:
            if self.low_error_tracking:
                duration = (current_time - self.low_error_start_time).to_sec()
                rospy.logwarn("[✗] 误差超出阈值，重新计时 (之前持续: %.1f秒)" % duration)
                self.low_error_tracking = False
                self.low_error_start_time = None
        
        # 终端输出
        print("\n" + "=" * 90)
        print("  实时位置对比 (ENU坐标系)")
        print("=" * 90)
        
        # 【新增】显示数据时间戳信息
        if self.estimated_time is not None:
            time_since_estimate = (current_time - self.estimated_time).to_sec()
            print("  【数据状态】")
            print("    估计数据时延: %.3f 秒 %s" % (
                time_since_estimate,
                "✓" if time_since_estimate < 0.2 else "⚠" if time_since_estimate < 0.5 else "✗"
            ))
        print()
        
        # 打印真值
        print("  【Actor真值位置】")
        print("    X: %8.3f 米  |  Y: %8.3f 米  |  Z: %8.3f 米" % (
            self.ground_truth_pos.x,
            self.ground_truth_pos.y,
            self.ground_truth_pos.z
        ))
        
        # 打印估计值
        print()
        print("  【检测估计位置】")
        print("    X: %8.3f 米  |  Y: %8.3f 米  |  Z: %8.3f 米" % (
            self.estimated_pos.x,
            self.estimated_pos.y,
            self.estimated_pos.z
        ))
        
        # 打印误差
        print()
        print("  【位置误差】")
        print("    ΔX: %+7.3f 米  |  ΔY: %+7.3f 米  |  ΔZ: %+7.3f 米" % (
            error['dx'], error['dy'], error['dz']
        ))
        
        # 打印距离误差
        print()
        print("  【距离误差】")
        print("    2D误差: %7.3f 米  |  3D误差: %7.3f 米" % (
            error['distance_2d'], error['distance_3d']
        ))
        
        # 打印统计数据
        if stats and stats['samples'] >= 10:
            print()
            print("  【统计数据】(最近%d个样本)" % stats['samples'])
            print("    3D误差: 平均=%.3f 米, 标准差=%.3f 米, 最大=%.3f 米, 最小=%.3f 米" % (
                stats['mean_3d'], stats['std_3d'], stats['max_3d'], stats['min_3d']
            ))
            print("    2D误差: 平均=%.3f 米, 标准差=%.3f 米" % (
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
        print("    当前精度: %s (3D误差: %.3f 米)" % (assessment, error['distance_3d']))
        
        # 打印低误差持续时间
        print()
        print("  【低误差跟踪】(|ΔX|<%.1f米 且 |ΔY|<%.1f米)" % 
              (self.low_error_threshold, self.low_error_threshold))
        if self.low_error_tracking and low_error_duration > 0:
            # 格式化持续时间
            if low_error_duration < 60:
                duration_str = "%.1f秒" % low_error_duration
            elif low_error_duration < 3600:
                minutes = int(low_error_duration / 60)
                seconds = low_error_duration % 60
                duration_str = "%d分%.1f秒" % (minutes, seconds)
            else:
                hours = int(low_error_duration / 3600)
                minutes = int((low_error_duration % 3600) / 60)
                seconds = low_error_duration % 60
                duration_str = "%d时%d分%.1f秒" % (hours, minutes, seconds)
            
            print("    状态: ✓ 持续低误差")
            print("    持续时间: %s" % duration_str)
            print("    当前误差: ΔX=%.3f米, ΔY=%.3f米" % (error['dx'], error['dy']))
        else:
            print("    状态: ✗ 误差超出阈值或等待数据")
            print("    当前误差: ΔX=%.3f米, ΔY=%.3f米" % (error['dx'], error['dy']))
            if not is_low_error:
                issues = []
                if abs(error['dx']) >= self.low_error_threshold:
                    issues.append("|ΔX|=%.3f米 ≥ %.1f米" % (abs(error['dx']), self.low_error_threshold))
                if abs(error['dy']) >= self.low_error_threshold:
                    issues.append("|ΔY|=%.3f米 ≥ %.1f米" % (abs(error['dy']), self.low_error_threshold))
                if issues:
                    print("    原因: %s" % ", ".join(issues))
        
        print("=" * 90)
        print()
    
    def update_truth_publisher(self):
        """更新真值发布器（后台运行）"""
        self.truth_publisher.publish_actor_pose()
    
    def run(self):
        """主循环"""
        # 使用更高的频率来保证真值发布
        rate = rospy.Rate(30)  # 30Hz
        
        # 显示刷新计数器
        display_counter = 0
        display_interval = 15  # 每15次循环显示一次（约0.5秒）
        
        print("开始实时对比...")
        print("按 Ctrl+C 停止")
        print()
        
        while not rospy.is_shutdown():
            # 每次循环都更新真值发布器
            self.update_truth_publisher()
            
            # 降低显示频率
            display_counter += 1
            if display_counter >= display_interval:
                self.print_comparison()
                display_counter = 0
            
            rate.sleep()
        
        # 退出时打印最终统计
        print()
        print("=" * 90)
        print("  最终统计报告")
        print("=" * 90)
        
        # 如果还在低误差状态，记录最后的持续时间
        if self.low_error_tracking and self.low_error_start_time is not None:
            final_duration = (rospy.Time.now() - self.low_error_start_time).to_sec()
            print()
            print("  低误差跟踪结果:")
            print("    最终状态: 持续低误差 (|ΔX|<%.1f米, |ΔY|<%.1f米)" % 
                  (self.low_error_threshold, self.low_error_threshold))
            print("    总持续时间: %.1f秒 (%.1f分钟)" % (final_duration, final_duration/60.0))
        
        stats = self.calculate_statistics()
        if stats and stats['samples'] > 0:
            print()
            print("  总样本数: %d" % stats['samples'])
            print()
            print("  3D误差统计:")
            print("    平均值: %.4f 米" % stats['mean_3d'])
            print("    标准差: %.4f 米" % stats['std_3d'])
            print("    最大值: %.4f 米" % stats['max_3d'])
            print("    最小值: %.4f 米" % stats['min_3d'])
            print()
            print("  2D误差统计:")
            print("    平均值: %.4f 米" % stats['mean_2d'])
            print("    标准差: %.4f 米" % stats['std_2d'])
            print()
            
            # 计算精度等级分布
            excellent = sum(1 for e in self.error_history if e['distance_3d'] < 0.5)
            good = sum(1 for e in self.error_history if 0.5 <= e['distance_3d'] < 1.0)
            fair = sum(1 for e in self.error_history if 1.0 <= e['distance_3d'] < 2.0)
            poor = sum(1 for e in self.error_history if 2.0 <= e['distance_3d'] < 5.0)
            bad = sum(1 for e in self.error_history if e['distance_3d'] >= 5.0)
            
            print("  精度分布:")
            print("    优秀 (<0.5米):  %3d个 (%.1f%%)" % (excellent, 100.0*excellent/stats['samples']))
            print("    良好 (<1.0米):  %3d个 (%.1f%%)" % (good, 100.0*good/stats['samples']))
            print("    一般 (<2.0米):  %3d个 (%.1f%%)" % (fair, 100.0*fair/stats['samples']))
            print("    较差 (<5.0米):  %3d个 (%.1f%%)" % (poor, 100.0*poor/stats['samples']))
            print("    很差 (>=5.0米): %3d个 (%.1f%%)" % (bad, 100.0*bad/stats['samples']))
            
        print()
        print("=" * 90)


if __name__ == "__main__":
    # 解析参数
    if len(sys.argv) >= 3:
        vehicle_type = sys.argv[1]
        vehicle_id = sys.argv[2]
        actor_name = sys.argv[3] if len(sys.argv) >= 4 else 'actor'
    else:
        # 默认值
        vehicle_type = 'typhoon_h480'
        vehicle_id = '0'
        actor_name = 'actor'
        print("[提示] 未指定参数，使用默认值")
        print("  飞行器: %s_%s" % (vehicle_type, vehicle_id))
        print("  Actor: %s" % actor_name)
        print("[用法] python compare_positions_integrated.py <vehicle_type> <vehicle_id> [actor_name]")
        print()
    
    # 初始化节点
    rospy.init_node('position_comparator_integrated')
    
    # 创建对比器
    comparator = PositionComparator(vehicle_type, vehicle_id, actor_name)
    
    try:
        comparator.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print()
        print("[退出] 用户中断")

