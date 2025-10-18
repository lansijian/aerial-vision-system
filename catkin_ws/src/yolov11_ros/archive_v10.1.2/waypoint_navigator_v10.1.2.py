#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Waypoint Navigator v9.5 - 航点导航器
负责航点飞行控制，支持一正一反巡检策略

功能：
1. 加载航点文件
2. 按顺序飞行到各航点
3. 支持正向/反向飞行模式
4. 发布速度命令到主控制器

作者: 东华大学 Astraeus队
日期: 2025-10-12
"""

import rospy
import json
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from std_msgs.msg import String, Bool, Int32
import tf.transformations as tf_trans


class WaypointNavigator:
    def __init__(self):
        rospy.init_node('waypoint_navigator')
        
        # 参数
        self.drone_id = rospy.get_param('~drone_id', 0)
        self.vehicle_type = rospy.get_param('~vehicle_type', 'typhoon_h480')
        self.vehicle_ns = f"{self.vehicle_type}_{self.drone_id}"
        self.cruise_speed = rospy.get_param('~cruise_speed', 5.0)
        self.waypoint_reached_threshold = rospy.get_param('~waypoint_reached_threshold', 2.0)
        
        # 航点数据
        self.waypoints = []
        self.current_waypoint_index = 0
        self.waypoint_reached = False
        self.reverse_mode = False  # 不再需要反转，waypoints_drone_1.json已经是反向路径
        
        # 状态变量
        self.current_position = None
        self.current_yaw = 0.0
        self.navigation_active = False
        self.mode = "IDLE"
        
        # PID控制参数
        self.Kp_pos = 1.0
        self.Kd_pos = 0.2
        self.Ki_pos = 0.05
        self.position_error_integral = np.array([0.0, 0.0, 0.0])
        self.last_position_error = np.array([0.0, 0.0, 0.0])
        
        # 订阅者
        self.pose_sub = rospy.Subscriber(
            f'/{self.vehicle_ns}/mavros/local_position/pose',
            PoseStamped, self._pose_callback
        )
        self.mode_sub = rospy.Subscriber(
            f'/drone_{self.drone_id}/flight_mode',
            String, self._mode_callback
        )
        
        # 发布者
        self.cmd_vel_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/waypoint_navigator/cmd_vel',
            TwistStamped, queue_size=1
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
        
        rospy.loginfo(f"✅ 无人机{self.drone_id}航点导航器初始化完成")
        rospy.loginfo(f"   飞行文件: waypoints_drone_{self.drone_id}.json")
        rospy.loginfo(f"   航点数量: {len(self.waypoints)}")
        
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
                # 刚切换到WAYPOINT模式，重置一些状态
                self.waypoint_reached = False
                self.position_error_integral = np.array([0.0, 0.0, 0.0])
                self.last_position_error = np.array([0.0, 0.0, 0.0])
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
        
    def _compute_navigation_command(self):
        """计算导航命令"""
        if self.current_position is None:
            return TwistStamped()
            
        waypoint = self._get_current_waypoint()
        if waypoint is None:
            return TwistStamped()
            
        # 目标位置
        target_position = np.array([waypoint['x'], waypoint['y'], waypoint['z']])
        
        # 位置误差
        position_error = target_position - self.current_position
        distance = np.linalg.norm(position_error[:2])  # 水平距离
        
        # 检查是否到达航点
        if distance < self.waypoint_reached_threshold:
            if not self.waypoint_reached:
                self._on_waypoint_reached()
            self.waypoint_reached = True
        else:
            self.waypoint_reached = False
            
        # PID控制
        # P项
        p_term = self.Kp_pos * position_error
        
        # I项
        self.position_error_integral += position_error * 0.02  # 假设50Hz
        self.position_error_integral = np.clip(self.position_error_integral, -5.0, 5.0)
        i_term = self.Ki_pos * self.position_error_integral
        
        # D项
        position_error_derivative = (position_error - self.last_position_error) / 0.02
        d_term = self.Kd_pos * position_error_derivative
        self.last_position_error = position_error.copy()
        
        # 总控制量
        control = p_term + i_term + d_term
        
        # 转换到机体坐标系
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        
        body_vel_x = control[0] * cos_yaw + control[1] * sin_yaw
        body_vel_y = -control[0] * sin_yaw + control[1] * cos_yaw
        body_vel_z = control[2]
        
        # 速度限制
        horizontal_speed = math.sqrt(body_vel_x**2 + body_vel_y**2)
        if horizontal_speed > self.cruise_speed:
            scale = self.cruise_speed / horizontal_speed
            body_vel_x *= scale
            body_vel_y *= scale
            
        body_vel_z = np.clip(body_vel_z, -1.0, 1.0)
        
        # 计算期望偏航角
        if distance > 0.1:
            desired_yaw = math.atan2(position_error[1], position_error[0])
            yaw_error = self._normalize_angle(desired_yaw - self.current_yaw)
            yaw_rate = np.clip(2.0 * yaw_error, -1.0, 1.0)
        else:
            yaw_rate = 0.0
            
        # 创建速度命令
        cmd = TwistStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.twist.linear.x = body_vel_x
        cmd.twist.linear.y = body_vel_y
        cmd.twist.linear.z = body_vel_z
        cmd.twist.angular.z = yaw_rate
        
        return cmd
        
    def _normalize_angle(self, angle):
        """规范化角度到[-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def _on_waypoint_reached(self):
        """到达航点时的处理"""
        waypoint = self._get_current_waypoint()
        rospy.loginfo(f"✅ 无人机{self.drone_id}: 到达航点{self.current_waypoint_index}")
        
        # 发布状态
        self.status_pub.publish(f"到达航点{self.current_waypoint_index}")
        
        # 在航点悬停
        if waypoint.get('hover_time', 0) > 0:
            rospy.sleep(waypoint['hover_time'])
            
        # 移动到下一个航点
        self.current_waypoint_index += 1
        
        # 检查是否完成所有航点
        if self.current_waypoint_index >= len(self.waypoints):
            if waypoint.get('loop', True):
                # 循环飞行
                self.current_waypoint_index = 0
                rospy.loginfo(f"🔄 无人机{self.drone_id}: 开始新一轮巡检")
            else:
                # 结束任务
                self.navigation_active = False
                rospy.loginfo(f"🏁 无人机{self.drone_id}: 完成所有航点")
                
        # 发布当前航点索引
        self.waypoint_index_pub.publish(self.current_waypoint_index)
        
    def run(self):
        """主循环"""
        rate = rospy.Rate(50)  # 50Hz
        
        rospy.loginfo(f"🚀 无人机{self.drone_id}航点导航器开始运行")
        
        while not rospy.is_shutdown():
            if self.navigation_active and self.mode == "WAYPOINT":
                # 检查位置是否有效
                if self.current_position is None:
                    rospy.logwarn_throttle(1.0, f"无人机{self.drone_id}: 等待位置信息...")
                else:
                    # 计算并发布导航命令
                    cmd = self._compute_navigation_command()
                    self.cmd_vel_pub.publish(cmd)
                    
                    # 发布状态信息
                    waypoint = self._get_current_waypoint()
                    if waypoint:
                        distance = 0
                        if self.current_position is not None:
                            target = np.array([waypoint['x'], waypoint['y'], waypoint['z']])
                            distance = np.linalg.norm(target - self.current_position)
                            
                        status = f"航点{self.current_waypoint_index}/{len(self.waypoints)}, 距离: {distance:.1f}m"
                        self.status_pub.publish(status)
                        
                        # 调试日志
                        rospy.loginfo_throttle(2.0, 
                            f"无人机{self.drone_id} 航点导航: 目标({waypoint['x']:.1f},{waypoint['y']:.1f},{waypoint['z']:.1f}) "
                            f"当前({self.current_position[0]:.1f},{self.current_position[1]:.1f},{self.current_position[2]:.1f}) "
                            f"距离:{distance:.1f}m")
                    
            rate.sleep()
            
        rospy.loginfo("航点导航器停止")


if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
