#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
无人机控制器 - v10.0.1-restored
负责无人机起飞、模式切换、航点导航和追踪协调

功能：
1. 自动起飞到指定高度
2. 起飞后自动切换到航点巡检模式
3. 协调航点导航和人体追踪的模式切换
4. 转发速度命令到MAVROS

作者：东华大学 Astraeus队
版本：v10.0.1-restored
日期：2025-10-14
"""

import rospy
import json
import threading
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from std_msgs.msg import String, Bool, Int32
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller')
        
        # ============ 参数配置 ============
        self.drone_id = rospy.get_param('~drone_id', 0)
        self.vehicle_type = rospy.get_param('~vehicle_type', 'typhoon_h480')
        self.vehicle_ns = f"{self.vehicle_type}_{self.drone_id}"
        self.takeoff_altitude = rospy.get_param('~takeoff_altitude', 3.0)
        self.initial_delay = rospy.get_param('~initial_delay', 0.5)
        self.waypoint_file = rospy.get_param('~waypoint_file', 
                                            f'waypoints/waypoints_drone_{self.drone_id}.json')
        
        # ============ 状态变量 ============
        self.mavros_state = None
        self.current_mode = "MANUAL"
        self.armed = False
        self.is_flying = False
        self.flight_mode = "IDLE"  # IDLE, WAYPOINT, TRACKING
        self.current_altitude = 0.0
        self.current_position = None
        self.lock = threading.Lock()
        
        # ============ 服务代理 ============
        self.arming_client = rospy.ServiceProxy(
            f'/{self.vehicle_ns}/mavros/cmd/arming', CommandBool
        )
        self.set_mode_client = rospy.ServiceProxy(
            f'/{self.vehicle_ns}/mavros/set_mode', SetMode
        )
        
        # ============ 订阅者 ============
        # MAVROS状态
        rospy.Subscriber(
            f'/{self.vehicle_ns}/mavros/state',
            State, self._state_callback
        )
        
        # 位置信息
        rospy.Subscriber(
            f'/{self.vehicle_ns}/mavros/local_position/pose',
            PoseStamped, self._pose_callback
        )
        
        # 航点导航速度命令
        rospy.Subscriber(
            f'/drone_{self.drone_id}/waypoint_navigator/cmd_vel',
            TwistStamped, self._waypoint_cmd_callback
        )
        
        # 人体追踪速度命令（接收Twist格式）
        rospy.Subscriber(
            f'/drone_{self.drone_id}/human_tracker/cmd_vel',
            Twist, self._tracker_cmd_callback_twist
        )
        
        # 追踪模式请求
        rospy.Subscriber(
            f'/drone_{self.drone_id}/request_tracking_mode',
            Bool, self._tracking_request_callback
        )
        
        # 恢复巡检请求
        rospy.Subscriber(
            f'/drone_{self.drone_id}/request_waypoint_mode',
            Bool, self._waypoint_request_callback
        )
        
        # ============ 发布者 ============
        # 转发到MAVROS的速度命令
        self.cmd_vel_pub = rospy.Publisher(
            f'/{self.vehicle_ns}/mavros/setpoint_velocity/cmd_vel',
            TwistStamped, queue_size=1
        )
        
        # 飞行模式发布（给waypoint_navigator和human_tracker）
        self.flight_mode_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/flight_mode',
            String, queue_size=1
        )
        
        # 状态发布
        self.status_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/controller_status',
            String, queue_size=1
        )
        
        # 起飞完成标志
        self.takeoff_complete_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/takeoff_complete',
            Bool, queue_size=1
        )
        
        # 当前速度命令缓存
        self.current_cmd = TwistStamped()
        
        rospy.loginfo(f"✅ 无人机{self.drone_id}控制器初始化完成 (v10.0.1-restored)")
        
    def _state_callback(self, msg):
        """MAVROS状态回调"""
        with self.lock:
            self.mavros_state = msg
            self.current_mode = msg.mode
            self.armed = msg.armed
            
    def _pose_callback(self, msg):
        """位置回调"""
        with self.lock:
            self.current_altitude = msg.pose.position.z
            self.current_position = msg.pose.position
            
    def _waypoint_cmd_callback(self, msg):
        """航点导航速度命令回调"""
        if self.flight_mode == "WAYPOINT":
            self.current_cmd = msg
            
    def _tracker_cmd_callback_twist(self, msg):
        """人体追踪速度命令回调（Twist格式）"""
        if self.flight_mode == "TRACKING":
            # 转换Twist为TwistStamped
            cmd_stamped = TwistStamped()
            cmd_stamped.header.stamp = rospy.Time.now()
            cmd_stamped.header.frame_id = "base_link"
            cmd_stamped.twist = msg
            self.current_cmd = cmd_stamped
            
    def _tracking_request_callback(self, msg):
        """追踪模式请求回调"""
        if msg.data and self.is_flying:
            rospy.loginfo(f"🎯 无人机{self.drone_id}: 切换到追踪模式")
            self.flight_mode = "TRACKING"
            self.flight_mode_pub.publish("TRACKING")
            
    def _waypoint_request_callback(self, msg):
        """恢复巡检请求回调"""
        if msg.data and self.is_flying:
            rospy.loginfo(f"🔄 无人机{self.drone_id}: 恢复航点巡检")
            self.flight_mode = "WAYPOINT"
            self.flight_mode_pub.publish("WAYPOINT")
            
    def arm_and_takeoff(self):
        """解锁并起飞"""
        rospy.loginfo(f"🚁 无人机{self.drone_id}: 准备起飞到{self.takeoff_altitude}米")
        
        # 等待MAVROS连接
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and (self.mavros_state is None or not self.mavros_state.connected):
            rospy.loginfo_once(f"等待无人机{self.drone_id} MAVROS连接...")
            rate.sleep()
            
        rospy.loginfo(f"✅ 无人机{self.drone_id} MAVROS已连接")
        
        # 发送设定点并切换到OFFBOARD模式
        rospy.loginfo(f"切换无人机{self.drone_id}到OFFBOARD模式...")
        offboard_switched = False
        for i in range(100):
            if rospy.is_shutdown():
                break
                
            # 发送初始设定点
            cmd = TwistStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "base_link"
            cmd.twist.linear.z = 0.5  # 向上
            self.cmd_vel_pub.publish(cmd)
            
            # 尝试切换模式（只打印一次成功消息）
            if self.current_mode != "OFFBOARD" and i > 10:
                try:
                    mode_resp = self.set_mode_client(custom_mode="OFFBOARD")
                    if mode_resp.mode_sent and not offboard_switched:
                        rospy.loginfo(f"✅ 无人机{self.drone_id}已切换到OFFBOARD模式")
                        offboard_switched = True
                except Exception as e:
                    if i == 11:  # 只打印一次警告
                        rospy.logwarn(f"设置OFFBOARD模式失败: {e}")
                    
            rate.sleep()
            
        # 解锁前多发送一些设定点，确保PX4准备就绪
        rospy.loginfo(f"准备解锁无人机{self.drone_id}...")
        for i in range(20):  # 再发送1秒的设定点
            cmd = TwistStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "base_link"
            cmd.twist.linear.z = 0.5
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()
        
        # 解锁
        if not self.armed:
            rospy.loginfo(f"解锁无人机{self.drone_id}...")
            for retry in range(3):  # 重试3次
                try:
                    arm_resp = self.arming_client(True)
                    if arm_resp.success:
                        rospy.loginfo(f"✅ 无人机{self.drone_id}解锁成功")
                        break
                    else:
                        rospy.logwarn(f"解锁尝试 {retry+1}/3 失败")
                        rospy.sleep(0.5)
                except Exception as e:
                    rospy.logwarn(f"解锁服务调用失败: {e}")
                    rospy.sleep(0.5)
            
            # 检查是否最终解锁成功
            rospy.sleep(0.5)
            if not self.armed:
                rospy.logerr(f"❌ 无人机{self.drone_id}解锁失败")
                return False
                
        # 起飞到目标高度
        rospy.loginfo(f"🚁 无人机{self.drone_id}起飞中...")
        self.is_flying = True
        
        while not rospy.is_shutdown() and self.current_altitude < self.takeoff_altitude * 0.9:
            cmd = TwistStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "base_link"
            cmd.twist.linear.z = 1.0  # 上升速度
            self.cmd_vel_pub.publish(cmd)
            
            # 输出高度信息
            if int(self.current_altitude * 10) % 5 == 0:  # 每0.5米输出一次
                rospy.loginfo(f"无人机{self.drone_id}当前高度: {self.current_altitude:.1f}m")
                
            rate.sleep()
            
        # 悬停稳定
        rospy.loginfo(f"无人机{self.drone_id}到达目标高度，悬停稳定...")
        for i in range(40):  # 2秒
            cmd = TwistStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "base_link"
            cmd.twist.linear.z = 0.0  # 悬停
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()
            
        rospy.loginfo(f"✅ 无人机{self.drone_id}起飞完成！")
        
        # 发布起飞完成信号
        self.takeoff_complete_pub.publish(True)
        self.status_pub.publish("READY")
        
        # 自动切换到航点巡检模式
        rospy.loginfo(f"🎯 无人机{self.drone_id}: 自动切换到航点巡检模式")
        self.flight_mode = "WAYPOINT"
        self.flight_mode_pub.publish("WAYPOINT")
        
        return True
        
    def run(self):
        """主循环"""
        # 初始延迟
        rospy.sleep(self.initial_delay)
        
        # 起飞
        if not self.arm_and_takeoff():
            rospy.logerr(f"无人机{self.drone_id}起飞失败")
            return
            
        rospy.loginfo(f"🎯 无人机{self.drone_id}开始任务执行...")
        
        # 主控制循环
        rate = rospy.Rate(20)  # 20Hz
        status_counter = 0
        
        while not rospy.is_shutdown():
            # 确保在OFFBOARD模式
            if self.current_mode != "OFFBOARD" and self.armed:
                try:
                    self.set_mode_client(custom_mode="OFFBOARD")
                except:
                    pass
                    
            # 发布当前速度命令到MAVROS
            if self.is_flying:
                # 确保命令有时间戳和正确的frame_id
                self.current_cmd.header.stamp = rospy.Time.now()
                # MAVROS期望的frame_id
                self.current_cmd.header.frame_id = "base_link"
                self.cmd_vel_pub.publish(self.current_cmd)
                
            # 定期发布飞行模式（确保各模块同步）
            if status_counter % 20 == 0:  # 每秒发布一次
                self.flight_mode_pub.publish(self.flight_mode)
                
                # 发布状态信息
                if self.current_position:
                    status = (f"Mode: {self.flight_mode} | "
                            f"Alt: {self.current_altitude:.1f}m | "
                            f"Pos: ({self.current_position.x:.1f}, "
                            f"{self.current_position.y:.1f})")
                    self.status_pub.publish(status)
                    
                    # 调试日志
                    rospy.loginfo_throttle(5.0, 
                        f"无人机{self.drone_id} - {status}")
                        
            status_counter += 1
            rate.sleep()
            
        rospy.loginfo(f"无人机{self.drone_id}控制器停止")

if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("控制器被中断")
    except Exception as e:
        rospy.logerr(f"控制器异常: {e}")
