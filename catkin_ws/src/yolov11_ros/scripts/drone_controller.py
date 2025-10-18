#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
无人机控制器 - v10.1.2-refactored
负责无人机起飞、模式切换、航点导航和追踪协调

功能：
1. 自动起飞到指定高度（使用位置控制）
2. 起飞后自动切换到航点巡检模式
3. 协调航点导航和人体追踪的模式切换
4. 转发速度命令到MAVROS

作者：东华大学 Astraeus队
版本：v10.1.2-refactored
日期：2025-10-16
"""

import rospy
import threading
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from std_msgs.msg import String, Bool
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State, PositionTarget
import tf.transformations as tf_trans

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller')
        
        # ============ 参数配置 ============
        self.drone_id = rospy.get_param('~drone_id', 0)
        self.vehicle_type = rospy.get_param('~vehicle_type', 'typhoon_h480')
        self.vehicle_ns = f"{self.vehicle_type}_{self.drone_id}"
        self.takeoff_altitude = rospy.get_param('~takeoff_altitude', 3.0)
        self.initial_delay = rospy.get_param('~initial_delay', 1.0)
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
        
        # 航点导航速度命令（注意：waypoint_navigator发布TwistStamped）
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
        # TwistStamped（用于起飞）
        self.cmd_vel_pub = rospy.Publisher(
            f'/{self.vehicle_ns}/mavros/setpoint_velocity/cmd_vel',
            TwistStamped, queue_size=1
        )
        
        # PositionTarget（用于追踪，世界坐标系）
        self.setpoint_raw_pub = rospy.Publisher(
            f'/{self.vehicle_ns}/mavros/setpoint_raw/local',
            PositionTarget, queue_size=1
        )
        
        # 当前位姿（用于坐标转换）
        self.current_pose = None
        
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
        
        rospy.loginfo(f"✅ 无人机{self.drone_id}控制器初始化完成 (v10.1.2-v9.2-fixed)")
        rospy.loginfo(f"   命名空间: {self.vehicle_ns}")
        rospy.loginfo(f"   起飞高度: {self.takeoff_altitude}米")
        rospy.loginfo(f"   航点文件: {self.waypoint_file}")
        rospy.loginfo(f"   速度控制：PositionTarget/FRAME_BODY_NED（机体坐标系，参考v9.2稳定版本）")
        
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
            self.current_pose = msg.pose  # 保存完整位姿（用于yaw转换）
            
    def _waypoint_cmd_callback(self, msg):
        """航点导航速度命令回调"""
        if self.flight_mode == "WAYPOINT":
            self.current_cmd = msg
            
    def _tracker_cmd_callback_twist(self, msg):
        """
        人体追踪速度命令回调（Twist格式，FLU坐标系）
        human_tracker发送FLU格式的速度
        这里缓存，主循环中转换为NED发布到MAVROS
        """
        # 直接缓存FLU格式的命令（不检查flight_mode，避免模式切换时丢失命令）
        cmd_stamped = TwistStamped()
        cmd_stamped.header.stamp = rospy.Time.now()
        cmd_stamped.twist = msg  # 保持FLU格式
        
        self.current_cmd = cmd_stamped
        
        # 调试日志
        rospy.loginfo_throttle(3.0, 
            f"[追踪速度FLU] 无人机{self.drone_id} 收到并缓存: "
            f"vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, vz={msg.linear.z:.2f}, yaw={msg.angular.z:.2f}")
            
    def _tracking_request_callback(self, msg):
        """追踪模式请求回调"""
        rospy.loginfo(f"[DEBUG] 无人机{self.drone_id} 收到追踪请求: data={msg.data}, is_flying={self.is_flying}")
        if msg.data and self.is_flying:
            rospy.loginfo(f"🎯 无人机{self.drone_id}: 切换到追踪模式")
            self.flight_mode = "TRACKING"
            self.flight_mode_pub.publish("TRACKING")
        elif not self.is_flying:
            rospy.logwarn(f"⚠️ 无人机{self.drone_id} 未起飞，无法切换到追踪模式")
            
    def _waypoint_request_callback(self, msg):
        """恢复巡检请求回调"""
        if msg.data and self.is_flying:
            rospy.loginfo(f"🔄 无人机{self.drone_id}: 恢复航点巡检")
            self.flight_mode = "WAYPOINT"
            self.flight_mode_pub.publish("WAYPOINT")
            
    def arm_and_takeoff(self):
        """解锁并起飞（使用位置控制避免反向问题）"""
        rospy.loginfo(f"🚁 无人机{self.drone_id}: 准备起飞到{self.takeoff_altitude}米")
        
        # 等待MAVROS连接
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and (self.mavros_state is None or not self.mavros_state.connected):
            rospy.loginfo_once(f"等待无人机{self.drone_id} MAVROS连接...")
            rate.sleep()
            
        rospy.loginfo(f"✅ 无人机{self.drone_id} MAVROS已连接")
        
        # 发送足够的setpoint（恢复v10.1.2稳定做法）
        rospy.loginfo(f"发送初始setpoint，准备OFFBOARD模式...")
        for i in range(150):  # 7.5秒
            if rospy.is_shutdown():
                break
                
            # 使用TwistStamped（起飞阶段，之前稳定的做法）
            cmd = TwistStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "base_link"
            cmd.twist.linear.z = 0.5  # 向上
            self.cmd_vel_pub.publish(cmd)
            
            rate.sleep()
        
        # 现在尝试切换到OFFBOARD模式
        rospy.loginfo(f"切换无人机{self.drone_id}到OFFBOARD模式...")
        offboard_switched = False
        for attempt in range(10):  # 最多尝试10次
            if self.current_mode == "OFFBOARD":
                rospy.loginfo(f"✅ 无人机{self.drone_id}已切换到OFFBOARD模式")
                offboard_switched = True
                break
            
            try:
                mode_resp = self.set_mode_client(custom_mode="OFFBOARD")
                if mode_resp.mode_sent:
                    rospy.sleep(0.5)  # 等待模式切换完成
                    if self.current_mode == "OFFBOARD":
                        rospy.loginfo(f"✅ 无人机{self.drone_id}已切换到OFFBOARD模式")
                        offboard_switched = True
                        break
            except Exception as e:
                rospy.logwarn(f"设置OFFBOARD模式失败(尝试{attempt+1}/10): {e}")
                rospy.sleep(0.5)
        
        if not offboard_switched:
            rospy.logerr(f"❌ 无人机{self.drone_id} OFFBOARD模式设置失败")
            return False
            
        # 解锁前再发送一些setpoint，确保稳定
        rospy.loginfo(f"准备解锁无人机{self.drone_id}...")
        for i in range(30):  # 1.5秒
            cmd = TwistStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "base_link"
            cmd.twist.linear.z = 0.5
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()
        
        # 解锁
        rospy.loginfo(f"解锁无人机{self.drone_id}...")
        arm_success = False
        for retry in range(3):  # 重试3次
            try:
                arm_resp = self.arming_client(True)
                if arm_resp.success:
                    rospy.loginfo(f"✅ 无人机{self.drone_id}解锁服务调用成功")
                    arm_success = True
                    break
                else:
                    rospy.logwarn(f"解锁尝试 {retry+1}/3 失败")
                    rospy.sleep(0.5)
            except Exception as e:
                rospy.logwarn(f"解锁服务调用失败: {e}")
                rospy.sleep(0.5)
        
        if not arm_success:
            rospy.logerr(f"❌ 无人机{self.drone_id}解锁服务调用失败")
            return False
        
        # 等待MAVROS状态更新
        rospy.loginfo(f"等待MAVROS状态更新...")
        for i in range(20):  # 最多等待2秒
            if self.armed:
                rospy.loginfo(f"✅ 无人机{self.drone_id}解锁状态确认")
                break
            rospy.sleep(0.1)
        
        if not self.armed:
            rospy.logerr(f"❌ 无人机{self.drone_id}解锁状态未确认，但继续起飞尝试")
            # 不return False，继续尝试起飞
                
        # 起飞到目标高度（恢复v10.1.2稳定做法 - TwistStamped速度控制）
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
        
        # 停止发布速度命令，让位置控制接管
        rospy.loginfo(f"⏸️ 无人机{self.drone_id}: 停止速度控制，释放控制权...")
        rospy.sleep(0.5)
        
        # 自动切换到航点巡检模式
        rospy.loginfo(f"🎯 无人机{self.drone_id}: 切换到航点巡检模式")
        self.flight_mode = "WAYPOINT"
        self.flight_mode_pub.publish("WAYPOINT")
        
        # 发布航点文件参数供waypoint_navigator使用
        rospy.set_param(f'/drone_{self.drone_id}/drone_controller/waypoint_file', self.waypoint_file)
        
        # 给waypoint_navigator时间准备
        rospy.sleep(0.5)
        rospy.loginfo(f"✅ 无人机{self.drone_id}: 控制权已交接给waypoint_navigator")
        
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
        
        # 主控制循环（提高到100Hz，响应更快）
        rate = rospy.Rate(100)  # 100Hz（原20Hz）
        status_counter = 0
        
        while not rospy.is_shutdown():
            # 确保在OFFBOARD模式
            if self.current_mode != "OFFBOARD" and self.armed:
                try:
                    self.set_mode_client(custom_mode="OFFBOARD")
                except:
                    pass
                    
            # 发布当前速度命令到MAVROS（仅在TRACKING模式）
            if self.is_flying and self.flight_mode == "TRACKING":
                # ✅ 参考v9.2稳定版本：使用FRAME_BODY_NED（机体坐标系）
                # human_tracker发送FLU格式 → 简单转换为BODY_NED
                # 这样避免了复杂的旋转变换，更稳定可靠
                
                cmd = self.current_cmd.twist
                target = PositionTarget()
                target.header.stamp = rospy.Time.now()
                target.header.frame_id = "base_link"
                target.coordinate_frame = PositionTarget.FRAME_BODY_NED  # 机体坐标系NED
                
                # 只使用速度，忽略位置
                target.type_mask = (
                    PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
                    PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                    PositionTarget.IGNORE_YAW
                )
                
                # FLU → BODY_NED 简单转换（参考v9.2 waypoint_flight.py）
                target.velocity.x = cmd.linear.x      # X: 前（不变）
                target.velocity.y = -cmd.linear.y     # Y: 左→右（取反）
                target.velocity.z = -cmd.linear.z     # Z: 上→下（取反）
                target.yaw_rate = -cmd.angular.z      # Yaw: 逆时针→顺时针（取反）
                
                self.setpoint_raw_pub.publish(target)
                
                rospy.loginfo_throttle(3.0, 
                    f"[控制权-TRACKING] 无人机{self.drone_id} "
                    f"FLU:(vx={cmd.linear.x:.2f},vy={cmd.linear.y:.2f},yaw={cmd.angular.z:.3f}) "
                    f"→ BODY_NED:(vx={target.velocity.x:.2f},vy={target.velocity.y:.2f},yaw={target.yaw_rate:.3f})")
            elif self.is_flying and self.flight_mode == "WAYPOINT":
                # WAYPOINT模式 - 不发布任何命令，由waypoint_navigator控制
                rospy.loginfo_once(f"[控制权-WAYPOINT] 无人机{self.drone_id} 控制权在waypoint_navigator")
                
            # 定期发布飞行模式（确保各模块同步）
            if status_counter % 100 == 0:  # 每秒发布一次（100Hz下）
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

