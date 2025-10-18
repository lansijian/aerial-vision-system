#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Waypoint Navigator v10.1.2-refactored - 航点导航器（位置控制版）
负责航点飞行控制，使用位置控制替代速度控制

⚠️ 关键修复：使用PositionTarget（位置控制）代替Twist（速度控制）
这解决了v10.1.2中无人机反向飞行的根本问题

功能：
1. 加载航点文件
2. 按顺序飞行到各航点
3. 使用位置控制（PositionTarget）
4. 不再发布速度命令到drone_controller

作者: 东华大学 Astraeus队
日期: 2025-10-16
版本: v10.1.2-refactored
"""

import rospy
import json
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from std_msgs.msg import String, Bool, Int32
import tf.transformations as tf_trans


class WaypointNavigator:
    def __init__(self):
        rospy.init_node('waypoint_navigator')
        
        # 参数
        self.drone_id = rospy.get_param('~drone_id', 0)
        self.vehicle_type = rospy.get_param('~vehicle_type', 'typhoon_h480')
        self.vehicle_ns = f"{self.vehicle_type}_{self.drone_id}"
        self.waypoint_reached_threshold = rospy.get_param('~waypoint_reached_threshold', 0.5)
        
        # 航点数据
        self.waypoints = []
        self.current_waypoint_index = 0
        self.waypoint_reached = False
        self.waypoint_start_time = 0
        
        # 状态变量
        self.current_position = None
        self.current_yaw = 0.0
        self.navigation_active = False
        self.mode = "IDLE"
        
        # 订阅者
        self.pose_sub = rospy.Subscriber(
            f'/{self.vehicle_ns}/mavros/local_position/pose',
            PoseStamped, self._pose_callback
        )
        self.mode_sub = rospy.Subscriber(
            f'/drone_{self.drone_id}/flight_mode',
            String, self._mode_callback
        )
        
        # 发布者 - 直接发布位置控制命令到MAVROS
        self.position_pub = rospy.Publisher(
            f'/{self.vehicle_ns}/mavros/setpoint_raw/local',
            PositionTarget, queue_size=1
        )
        self.status_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/waypoint_status',
            String, queue_size=1
        )
        self.waypoint_index_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/current_waypoint_index',
            Int32, queue_size=1
        )
        
        # 加载航点
        self._load_waypoints()
        
        rospy.loginfo("="*60)
        rospy.loginfo(f"✅ 无人机{self.drone_id}航点导航器初始化完成 (v10.1.2-refactored)")
        rospy.loginfo(f"   航点数量: {len(self.waypoints)}")
        rospy.loginfo(f"   ⚠️ 使用位置控制（修复速度控制导致的反向问题）")
        rospy.loginfo("="*60)
        
    def _load_waypoints(self):
        """加载航点文件"""
        # 获取ROS包路径
        import rospkg
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('yolov11_ros')
        
        # 从参数服务器获取航点文件名
        waypoint_file = rospy.get_param(
            f'/drone_{self.drone_id}/drone_controller/waypoint_file',
            f'waypoints/waypoints_drone_{self.drone_id}.json'
        )
        
        # 如果是相对路径，转换为绝对路径
        import os
        if not os.path.isabs(waypoint_file):
            waypoint_file = os.path.join(pkg_path, 'scripts', waypoint_file)
        
        try:
            rospy.loginfo(f"加载航点文件: {waypoint_file}")
            with open(waypoint_file, 'r') as f:
                data = json.load(f)
                self.waypoints = data['waypoints']
                
            rospy.loginfo(f"✅ 加载了{len(self.waypoints)}个航点")
            # 显示前几个航点以确认路径
            rospy.loginfo(f"   无人机{self.drone_id}航点路径:")
            for i in range(min(5, len(self.waypoints))):
                wp = self.waypoints[i]
                rospy.loginfo(f"   航点{i}: ({wp['x']}, {wp['y']}, {wp['z']})")
            
        except Exception as e:
            rospy.logerr(f"加载航点文件失败: {e}")
            self.waypoints = []
            
    def _pose_callback(self, msg):
        """位置回调"""
        self.current_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        # 计算偏航角
        orientation = msg.pose.orientation
        _, _, self.current_yaw = tf_trans.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        
    def _mode_callback(self, msg):
        """飞行模式回调"""
        old_mode = self.mode
        self.mode = msg.data
        if self.mode == "WAYPOINT":
            self.navigation_active = True
            if old_mode != "WAYPOINT":
                # 刚切换到WAYPOINT模式
                self.waypoint_reached = False
                rospy.loginfo(f"🚁 无人机{self.drone_id}: 开始航点飞行")
                rospy.loginfo(f"   当前航点索引: {self.current_waypoint_index}")
                if self.waypoints:
                    waypoint = self._get_current_waypoint()
                    if waypoint:
                        rospy.loginfo(f"   目标航点: ({waypoint['x']}, {waypoint['y']}, {waypoint['z']})")
        else:
            self.navigation_active = False
            
    def _get_current_waypoint(self):
        """获取当前航点"""
        if 0 <= self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        return None
        
    def _create_position_target(self, x, y, z, yaw=None):
        """创建位置目标消息"""
        cmd = PositionTarget()
        cmd.header.stamp = rospy.Time.now()
        cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        # 使用位置控制
        cmd.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        
        cmd.position.x = x
        cmd.position.y = y
        cmd.position.z = z
        
        if yaw is not None:
            cmd.yaw = yaw
        else:
            # 计算朝向航点的偏航角
            if self.current_position is not None:
                dx = x - self.current_position[0]
                dy = y - self.current_position[1]
                if abs(dx) > 0.1 or abs(dy) > 0.1:
                    cmd.yaw = math.atan2(dy, dx)
                else:
                    cmd.yaw = self.current_yaw
            else:
                cmd.yaw = 0.0
        
        return cmd
    
    def _check_waypoint_reached(self):
        """检查是否到达航点"""
        if self.current_position is None or self.current_waypoint_index >= len(self.waypoints):
            return False
        
        current_wp = self.waypoints[self.current_waypoint_index]
        dx = current_wp['x'] - self.current_position[0]
        dy = current_wp['y'] - self.current_position[1]
        dz = current_wp['z'] - self.current_position[2]
        
        # 使用3D距离判断
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        return distance < self.waypoint_reached_threshold
        
    def _on_waypoint_reached(self):
        """到达航点时的处理"""
        waypoint = self._get_current_waypoint()
        rospy.loginfo(f"✅ 无人机{self.drone_id}: 到达航点{self.current_waypoint_index}")
        
        # 发布状态
        self.status_pub.publish(f"到达航点{self.current_waypoint_index}")
        
        # 记录到达时间（用于悬停）
        self.waypoint_start_time = rospy.Time.now().to_sec()
        
    def run(self):
        """主循环 - v9.2纯定点飞行方式（持续发送保持OFFBOARD）"""
        rate = rospy.Rate(10)  # 10Hz - 保持OFFBOARD稳定性
        
        rospy.loginfo(f"🚀 无人机{self.drone_id}航点导航器开始运行 (v9.2纯定点飞行)")
        rospy.loginfo(f"   持续发送位置目标以维持OFFBOARD模式")
        
        # 当前目标航点命令（持续发送）
        current_target_cmd = None
        waypoint_just_changed = False
        
        while not rospy.is_shutdown():
            if self.navigation_active and self.mode == "WAYPOINT":
                # 检查位置是否有效
                if self.current_position is None:
                    rospy.logwarn_throttle(1.0, f"无人机{self.drone_id}: 等待位置信息...")
                else:
                    # 检查航点索引
                    if self.current_waypoint_index >= len(self.waypoints):
                        # 循环执行
                        self.current_waypoint_index = 0
                        waypoint_just_changed = True
                        rospy.loginfo(f"🔄 无人机{self.drone_id}: 开始新一轮巡检")
                    
                    waypoint = self._get_current_waypoint()
                    if waypoint:
                        # 检查是否到达航点
                        if self._check_waypoint_reached():
                            if not self.waypoint_reached:
                                self.waypoint_reached = True
                                self._on_waypoint_reached()
                            
                            # 在航点停留 - 持续发送当前位置保持
                            if current_target_cmd:
                                self.position_pub.publish(current_target_cmd)
                            
                            hover_time_elapsed = rospy.Time.now().to_sec() - self.waypoint_start_time
                            if hover_time_elapsed >= waypoint.get('hover_time', 0.5):
                                # 停留时间结束，前往下一个航点
                                self.current_waypoint_index += 1
                                self.waypoint_reached = False
                                waypoint_just_changed = True  # 标记航点已改变
                                self.waypoint_index_pub.publish(self.current_waypoint_index)
                                
                                if self.current_waypoint_index < len(self.waypoints):
                                    next_wp = self.waypoints[self.current_waypoint_index]
                                    rospy.loginfo(f"➡️ 无人机{self.drone_id}: 前往航点{self.current_waypoint_index}: "
                                                f"({next_wp['x']:.1f}, {next_wp['y']:.1f}, {next_wp['z']:.1f})")
                        else:
                            # v9.2纯定点飞行：航点改变时创建新命令，然后持续发送
                            if waypoint_just_changed or current_target_cmd is None:
                                current_target_cmd = self._create_position_target(waypoint['x'], waypoint['y'], waypoint['z'])
                                waypoint_just_changed = False
                                rospy.loginfo(f"📍 无人机{self.drone_id}: 设定航点{self.current_waypoint_index} "
                                            f"({waypoint['x']:.1f}, {waypoint['y']:.1f}, {waypoint['z']:.1f})")
                            
                            # 持续发送当前目标（保持OFFBOARD模式）
                            if current_target_cmd:
                                current_target_cmd.header.stamp = rospy.Time.now()  # 更新时间戳
                                self.position_pub.publish(current_target_cmd)
                                
                                # 定期输出控制权信息
                                rospy.loginfo_throttle(10.0, 
                                    f"[控制权-WAYPOINT] 无人机{self.drone_id} 位置控制中 (10Hz持续发送)")
                            
                            # 调试信息
                            dx = waypoint['x'] - self.current_position[0]
                            dy = waypoint['y'] - self.current_position[1]
                            dz = waypoint['z'] - self.current_position[2]
                            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                            
                            rospy.loginfo_throttle(3.0, 
                                f"[航点飞行] 无人机{self.drone_id} → WP{self.current_waypoint_index} "
                                f"({waypoint['x']:.1f},{waypoint['y']:.1f},{waypoint['z']:.1f}) "
                                f"距离:{distance:.1f}m")
            else:
                # 非WAYPOINT模式，不发布任何命令（避免冲突）
                current_target_cmd = None
                waypoint_just_changed = False
                    
            rate.sleep()
            
        rospy.loginfo("航点导航器停止")


if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass

