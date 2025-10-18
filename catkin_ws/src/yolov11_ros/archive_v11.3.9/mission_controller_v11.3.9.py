#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
任务控制器 - 基于v11架构（位置控制版本）
负责无人机任务的总体控制，包括起飞、降落、模式切换等
注意：waypoint_mission现在直接发布到MAVROS，不需要转发

功能：
1. 起飞控制
2. 模式切换通知
3. 追踪命令转发（追踪仍使用速度控制）
"""

import rospy
import threading
from geometry_msgs.msg import TwistStamped, PoseStamped, Twist
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Bool, String, Empty
import numpy as np

class MissionController:
    def __init__(self):
        rospy.init_node('mission_controller')
        
        # 无人机参数
        self.drone_id = rospy.get_param('~drone_id', 0)
        self.vehicle_type = rospy.get_param('~vehicle_type', 'typhoon_h480')
        self.vehicle_ns = f'{self.vehicle_type}_{self.drone_id}'
        
        # 控制参数
        self.takeoff_altitude = rospy.get_param('~takeoff_altitude', 3.0)
        self.initial_delay = rospy.get_param('~initial_delay', 1.0)
        self.waypoint_file = rospy.get_param('~waypoint_file', 
                                           f'waypoints/waypoints_drone_{self.drone_id}.json')
        
        # 状态管理
        self.state = 'IDLE'  # IDLE, ARMING, TAKEOFF, WAYPOINT, TRACKING, LANDING
        self.current_mode = None
        self.is_armed = False
        self.current_position = None
        self.current_altitude = 0.0
        self.offboard_enabled = False
        self.flight_mode = "IDLE"  # 当前飞行模式
        
        # 服务客户端
        self.arming_service = rospy.ServiceProxy(f'/{self.vehicle_ns}/mavros/cmd/arming', CommandBool)
        self.set_mode_service = rospy.ServiceProxy(f'/{self.vehicle_ns}/mavros/set_mode', SetMode)
        
        # 订阅者
        self.mavros_state_sub = rospy.Subscriber(
            f'/{self.vehicle_ns}/mavros/state', 
            State, self._mavros_state_callback
        )
        self.pose_sub = rospy.Subscriber(
            f'/{self.vehicle_ns}/mavros/local_position/pose',
            PoseStamped, self._pose_callback
        )
        
        # 追踪速度命令（来自waypoint_mission转发）
        self.tracker_cmd_sub = rospy.Subscriber(
            f'/drone_{self.drone_id}/tracking_control',
            Twist, self._tracker_cmd_callback
        )
        
        # 追踪状态（来自waypoint_mission）
        self.tracking_status_sub = rospy.Subscriber(
            f'/drone_{self.drone_id}/tracking_status',
            Bool, self._tracking_status_callback
        )
        
        # 发布者
        self.cmd_vel_pub = rospy.Publisher(
            f'/{self.vehicle_ns}/mavros/setpoint_velocity/cmd_vel',
            TwistStamped, queue_size=1
        )
        # 位置控制发布者（用于起飞）
        self.position_pub = rospy.Publisher(
            f'/{self.vehicle_ns}/mavros/setpoint_raw/local',
            PositionTarget, queue_size=1
        )
        self.flight_mode_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/flight_mode',
            String, queue_size=1
        )
        self.status_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/status',
            String, queue_size=1
        )
        
        # 当前速度命令
        self.current_cmd = Twist()
        self.cmd_lock = threading.Lock()
        
        rospy.loginfo(f"[任务控制器] 无人机{self.drone_id}初始化完成")
        rospy.loginfo(f"[任务控制器] 命名空间: {self.vehicle_ns}")
        rospy.loginfo(f"[任务控制器] 起飞高度: {self.takeoff_altitude}米")
        rospy.loginfo(f"[任务控制器] 初始延迟: {self.initial_delay}秒")
        rospy.loginfo(f"[任务控制器] 航点任务直接由waypoint_mission控制")
    
    def _mavros_state_callback(self, msg):
        """MAVROS状态回调"""
        self.current_mode = msg.mode
        self.is_armed = msg.armed
    
    def _pose_callback(self, msg):
        """位置回调"""
        self.current_position = msg.pose.position
        self.current_altitude = msg.pose.position.z
    
    def _tracker_cmd_callback(self, msg):
        """追踪速度命令回调"""
        rospy.loginfo_throttle(2.0, f"[任务控制器] 收到追踪命令: vx={msg.linear.x:.2f} vy={msg.linear.y:.2f} vz={msg.linear.z:.2f} yaw={msg.angular.z:.2f}")
        if self.flight_mode == "TRACKING":
            with self.cmd_lock:
                self.current_cmd = msg
        else:
            rospy.logwarn_throttle(5.0, f"[任务控制器] 收到追踪命令但飞行模式为{self.flight_mode}")
    
    def _tracking_status_callback(self, msg):
        """追踪状态回调"""
        # 只要解锁后就可以切换模式（不限制在EXECUTING状态）
        if self.is_armed:
            if msg.data:
                # 开始追踪
                rospy.loginfo(f"[任务控制器] 收到追踪状态，切换到追踪模式")
                self.flight_mode = "TRACKING"
                self.flight_mode_pub.publish("TRACKING")
                # 重置当前命令以避免使用旧命令
                with self.cmd_lock:
                    self.current_cmd = Twist()
            else:
                # 停止追踪，恢复航点
                rospy.loginfo(f"[任务控制器] 追踪结束，恢复航点模式")
                self.flight_mode = "WAYPOINT"
                self.flight_mode_pub.publish("WAYPOINT")
        else:
            rospy.logwarn_throttle(5.0, f"[任务控制器] 收到追踪状态但无人机未解锁")
    
    def _send_velocity_setpoint(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
        """发送速度设定点"""
        cmd = TwistStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.linear.z = vz
        cmd.twist.angular.z = yaw_rate
        self.cmd_vel_pub.publish(cmd)
    
    def _send_position_setpoint(self, x, y, z, yaw=0.0):
        """发送位置设定点"""
        cmd = PositionTarget()
        cmd.header.stamp = rospy.Time.now()
        cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        cmd.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        cmd.position.x = x
        cmd.position.y = y
        cmd.position.z = z
        cmd.yaw = yaw
        self.position_pub.publish(cmd)
    
    def _arm_drone(self):
        """解锁无人机"""
        rospy.loginfo(f"[任务控制器] 开始解锁...")
        
        # 确保在OFFBOARD模式
        max_attempts = 3
        for i in range(max_attempts):
            try:
                if self.current_mode != "OFFBOARD":
                    rospy.loginfo(f"[任务控制器] 设置OFFBOARD模式...")
                    resp = self.set_mode_service(custom_mode="OFFBOARD")
                    if resp.mode_sent:
                        rospy.sleep(0.5)
                        if self.current_mode == "OFFBOARD":
                            rospy.loginfo(f"[任务控制器] OFFBOARD模式设置成功")
                            break
                else:
                    break
            except Exception as e:
                rospy.logwarn(f"[任务控制器] 设置模式失败: {e}")
                
            if i == max_attempts - 1:
                rospy.logerr(f"[任务控制器] OFFBOARD模式设置失败")
                return False
        
        # 解锁
        try:
            resp = self.arming_service(True)
            if resp.success:
                rospy.loginfo(f"[任务控制器] 解锁成功")
                return True
            else:
                rospy.logerr(f"[任务控制器] 解锁失败")
                return False
        except Exception as e:
            rospy.logerr(f"[任务控制器] 解锁服务调用失败: {e}")
            return False
    
    def _handle_idle(self):
        """处理空闲状态"""
        if self.current_position:
            # 位置保持
            self._send_position_setpoint(
                self.current_position.x,
                self.current_position.y,
                self.current_position.z
            )
    
    def _handle_arming(self):
        """处理解锁状态"""
        if not self.is_armed:
            # 发送一些设定点以准备OFFBOARD模式
            if self.current_position:
                self._send_position_setpoint(
                    self.current_position.x,
                    self.current_position.y,
                    self.current_position.z + 0.1
                )
            else:
                self._send_position_setpoint(0.0, 0.0, 0.5)
            
            # 尝试解锁
            if hasattr(self, '_arm_attempts'):
                self._arm_attempts += 1
            else:
                self._arm_attempts = 0
                
            if self._arm_attempts % 20 == 0:  # 每秒尝试一次
                if self._arm_drone():
                    self.state = 'TAKEOFF'
                    rospy.loginfo(f"[任务控制器] 进入起飞状态")
                elif self._arm_attempts > 100:  # 5秒超时
                    rospy.logerr(f"[任务控制器] 解锁超时")
                    self.state = 'IDLE'
        else:
            self.state = 'TAKEOFF'
    
    def _handle_takeoff(self):
        """处理起飞状态（使用位置控制）"""
        # 起飞开始时就设置WAYPOINT模式（让target_tracker知道当前状态）
        if self.flight_mode != "WAYPOINT":
            self.flight_mode = "WAYPOINT"
            self.flight_mode_pub.publish("WAYPOINT")
            rospy.loginfo(f"[任务控制器] 设置初始飞行模式: WAYPOINT")
        
        if self.current_altitude < self.takeoff_altitude - 0.3:
            # 起飞中 - 使用位置控制
            if self.current_position:
                self._send_position_setpoint(
                    self.current_position.x,
                    self.current_position.y,
                    self.takeoff_altitude
                )
            else:
                self._send_position_setpoint(0.0, 0.0, self.takeoff_altitude)
                
            rospy.loginfo_throttle(1.0, 
                f"[任务控制器] 起飞中...当前高度: {self.current_altitude:.1f}m / 目标: {self.takeoff_altitude}m")
        else:
            # 起飞完成
            rospy.loginfo(f"[任务控制器] 起飞完成，开始航点任务")
            self.state = 'EXECUTING'
            # 发布航点文件路径供waypoint_mission使用
            rospy.set_param(f'/drone_{self.drone_id}/mission_controller/waypoint_file', self.waypoint_file)
    
    def _handle_executing(self):
        """处理执行状态"""
        if self.flight_mode == "TRACKING":
            # 追踪模式 - 转发速度命令到MAVROS
            with self.cmd_lock:
                cmd_stamped = TwistStamped()
                cmd_stamped.header.stamp = rospy.Time.now()
                cmd_stamped.header.frame_id = "base_link"
                
                if self.current_cmd and (abs(self.current_cmd.linear.x) > 0.01 or 
                                        abs(self.current_cmd.linear.y) > 0.01 or
                                        abs(self.current_cmd.angular.z) > 0.01):
                    # 有有效的追踪命令
                    cmd_stamped.twist = self.current_cmd
                    self.cmd_vel_pub.publish(cmd_stamped)
                    rospy.loginfo_throttle(2.0, 
                        f"[任务控制器] 追踪模式 - 转发速度命令到MAVROS: "
                        f"vx={self.current_cmd.linear.x:.2f} vy={self.current_cmd.linear.y:.2f} "
                        f"vz={self.current_cmd.linear.z:.2f} yaw={self.current_cmd.angular.z:.2f}")
                else:
                    # 没有追踪命令时，发送零速度保持OFFBOARD模式
                    cmd_stamped.twist.linear.x = 0.0
                    cmd_stamped.twist.linear.y = 0.0
                    cmd_stamped.twist.linear.z = 0.0
                    cmd_stamped.twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd_stamped)
                    rospy.loginfo_throttle(3.0, "[任务控制器] 追踪模式 - 等待追踪命令，发送零速度")
        elif self.flight_mode == "WAYPOINT":
            # 航点模式 - waypoint_mission会直接发布位置控制命令
            # 这里只需保持OFFBOARD模式（通过发送当前位置）
            if self.current_position:
                self._send_position_setpoint(
                    self.current_position.x,
                    self.current_position.y,
                    self.current_position.z
                )
            rospy.loginfo_throttle(5.0, "[任务控制器] 航点模式 - 由waypoint_mission控制")
    
    def _control_loop(self):
        """主控制循环"""
        rate = rospy.Rate(20)  # 20Hz
        
        # 等待初始延迟
        rospy.sleep(self.initial_delay)
        
        # 发送初始设定点以启动OFFBOARD模式
        rospy.loginfo(f"[任务控制器] 发送初始设定点...")
        for i in range(30):  # 1.5秒
            if self.current_position:
                self._send_position_setpoint(
                    self.current_position.x,
                    self.current_position.y,
                    self.current_position.z + 0.1
                )
            else:
                self._send_position_setpoint(0.0, 0.0, 0.5)
            rate.sleep()
        
        # 开始任务
        self.state = 'ARMING'
        rospy.loginfo(f"[任务控制器] 开始任务序列")
        
        while not rospy.is_shutdown():
            # 状态机
            if self.state == 'IDLE':
                self._handle_idle()
            elif self.state == 'ARMING':
                self._handle_arming()
            elif self.state == 'TAKEOFF':
                self._handle_takeoff()
            elif self.state == 'EXECUTING':
                self._handle_executing()
            elif self.state == 'LANDING':
                self._handle_landing()
            
            # 发布状态
            self.status_pub.publish(self.state)
            
            # 确保OFFBOARD模式
            if self.is_armed and self.current_mode != "OFFBOARD":
                try:
                    self.set_mode_service(custom_mode="OFFBOARD")
                except:
                    pass
            
            rate.sleep()
    
    def _handle_landing(self):
        """处理降落状态"""
        if self.current_altitude > 0.2:
            if self.current_position:
                self._send_position_setpoint(
                    self.current_position.x,
                    self.current_position.y,
                    0.0
                )
        else:
            # 降落完成，锁定
            try:
                self.arming_service(False)
                rospy.loginfo(f"[任务控制器] 降落完成，已锁定")
                self.state = 'IDLE'
            except:
                pass
    
    def run(self):
        """运行控制器"""
        try:
            self._control_loop()
        except rospy.ROSInterruptException:
            rospy.loginfo(f"[任务控制器] 节点关闭")


if __name__ == '__main__':
    controller = MissionController()
    controller.run()