#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
航点飞行节点
功能：专门负责航点飞行控制
与人体跟踪节点通过ROS话题通信，实现优先级控制
改进：添加后台setpoint发送线程，确保OFFBOARD模式稳定
"""

import rospy
import math
import numpy as np
import json
import threading
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from mavros_msgs.msg import PositionTarget, State, MountControl, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, MountConfigure, ParamSet
from geometry_msgs.msg import Twist
from pyquaternion import Quaternion

class WaypointFlight:
    def __init__(self):
        rospy.init_node('waypoint_flight', anonymous=True)
        
        # 多机参数
        self.drone_id = rospy.get_param("~drone_id", 0)
        self.num_drones = rospy.get_param("~num_drones", 1)
        self.vehicle_ns = rospy.get_param('~vehicle_ns', f'typhoon_h480_{self.drone_id}')
        
        # 飞行状态
        self.mission_active = False
        self.current_position = None
        self.current_yaw = 0.0
        self.mavros_state = None
        
        # 航点相关
        self.waypoints = []
        self.current_waypoint_index = 0
        self.waypoint_reached = False
        self.waypoint_start_time = 0
        
        # 人体跟踪状态
        self.tracking_active = False
        self.tracking_paused_waypoint = None
        self.last_tracking_control_time = 0
        self.latest_tracking_cmd = None
        
        # 后台setpoint发送线程（保持OFFBOARD模式）
        self.background_setpoint_active = False
        self.background_thread = None
        self.setpoint_lock = threading.Lock()
        
        # 参数配置
        self.waypoint_reached_threshold = rospy.get_param("~waypoint_reached_threshold", 2.0)
        self.waypoint_velocity = rospy.get_param("~waypoint_velocity", 5.0)
        self.waypoint_hover_time = rospy.get_param("~waypoint_hover_time", 1.0)
        self.yaw_rate_limit = rospy.get_param("~yaw_rate_limit", 1.5)
        self.takeoff_altitude = rospy.get_param("~takeoff_altitude", 3.0)  # 固定3米
        self.max_altitude = rospy.get_param("~max_altitude", 3.5)
        self.min_altitude = rospy.get_param("~min_altitude", 2.5)
        
        # 控制坐标系
        self.velocity_frame = PositionTarget.FRAME_BODY_NED
        self.position_frame = PositionTarget.FRAME_LOCAL_NED
        
        # ROS发布者（多机话题命名）
        self.setpoint_pub = rospy.Publisher(f'/{self.vehicle_ns}/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.flight_status_pub = rospy.Publisher(f'/drone_{self.drone_id}/waypoint_flight/status', String, queue_size=1)
        self.mount_pub = rospy.Publisher(f'/{self.vehicle_ns}/mavros/mount_control/command', MountControl, queue_size=1)
        
        # ROS订阅者（多机话题命名）
        self.pose_sub = rospy.Subscriber(f'/{self.vehicle_ns}/mavros/local_position/pose', PoseStamped, self._pose_callback)
        self.state_sub = rospy.Subscriber(f'/{self.vehicle_ns}/mavros/state', State, self._state_callback)
        self.tracking_status_sub = rospy.Subscriber(f'/drone_{self.drone_id}/human_tracker/status', Bool, self._tracking_status_callback)
        self.tracking_control_sub = rospy.Subscriber(f'/drone_{self.drone_id}/human_tracker/cmd_vel', Twist, self._tracking_control_callback)
        
        # 手动控制订阅者（多机话题命名）
        self.manual_cmd_sub = rospy.Subscriber(f'/drone_{self.drone_id}/waypoint_flight/manual_cmd', String, self._manual_cmd_callback)
        
        # ROS服务
        self.arming_client = rospy.ServiceProxy(f'/{self.vehicle_ns}/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy(f'/{self.vehicle_ns}/mavros/set_mode', SetMode)
        
        # 加载航点
        self._load_waypoints()
        
        rospy.loginfo("="*60)
        rospy.loginfo(f"✈️ 航点飞行节点已启动 - 无人机 {self.drone_id}")
        rospy.loginfo("="*60)
        rospy.loginfo(f"   无人机ID: {self.drone_id}")
        rospy.loginfo(f"   无人机总数: {self.num_drones}")
        rospy.loginfo(f"   命名空间: {self.vehicle_ns}")
        rospy.loginfo(f"   航点数量: {len(self.waypoints)}")
        rospy.loginfo("📡 手动控制指令:")
        rospy.loginfo(f"  启动: rostopic pub /drone_{self.drone_id}/waypoint_flight/manual_cmd std_msgs/String 'start'")
        rospy.loginfo(f"  停止: rostopic pub /drone_{self.drone_id}/waypoint_flight/manual_cmd std_msgs/String 'stop'")
        rospy.loginfo(f"  降落: rostopic pub /drone_{self.drone_id}/waypoint_flight/manual_cmd std_msgs/String 'land'")
        rospy.loginfo("="*60)
    
    def _load_waypoints(self):
        """加载航点数据"""
        waypoint_file = rospy.get_param("~waypoint_file", "")
        
        if waypoint_file:
            try:
                with open(waypoint_file, 'r') as f:
                    waypoint_data = json.load(f)
                    self.waypoints = waypoint_data.get('waypoints', [])
                
                rospy.loginfo(f"成功从 {waypoint_file} 加载 {len(self.waypoints)} 个航点")
                for i, wp in enumerate(self.waypoints):
                    rospy.loginfo(f"  航点 {i}: ({wp['x']:.1f}, {wp['y']:.1f}, {wp['z']:.1f})")
                
                    
            except Exception as e:
                rospy.logerr(f"加载航点文件失败: {e}")
                self.waypoints = []
        
        # 如果没有成功加载航点，使用默认航点（固定3米）
        if not self.waypoints:
            self.waypoints = [
                {'x': 0, 'y': 0, 'z': 3.0},
                {'x': 105, 'y': 0, 'z': 3.0},
                {'x': 105, 'y': 45, 'z': 3.0},
                {'x': 45, 'y': 45, 'z': 3.0},
                {'x': -45, 'y': 45, 'z': 3.0},
                {'x': -45, 'y': 0, 'z': 3.0},
                {'x': -15, 'y': 0, 'z': 3.0},
                {'x': -15, 'y': -45, 'z': 3.0},
                {'x': 45, 'y': -45, 'z': 3.0},
                {'x': 45, 'y': 45, 'z': 3.0},
                {'x': -15, 'y': 45, 'z': 3.0},
                {'x': -15, 'y': 0, 'z': 3.0},
                {'x': -45, 'y': 0, 'z': 3.0},
                {'x': -45, 'y': -45, 'z': 3.0},
                {'x': 105, 'y': -45, 'z': 3.0},
                {'x': 105, 'y': 45, 'z': 3.0},
                {'x': 45, 'y': 45, 'z': 3.0},
                {'x': 45, 'y': -45, 'z': 3.0},
                {'x': -15, 'y': -45, 'z': 3.0},
                {'x': -15, 'y': 0, 'z': 3.0},
                {'x': 0, 'y': 0, 'z': 3.0}
            ]
            rospy.loginfo(f"使用默认航点数据，共 {len(self.waypoints)} 个航点（固定3米）")
        
        # 确保所有航点高度都在5米以下（无论是从文件加载还是默认航点）
        self._enforce_altitude_limits()
    
    def _enforce_altitude_limits(self):
        """强制所有航点高度固定在3米"""
        modified_count = 0
        for i, waypoint in enumerate(self.waypoints):
            original_z = waypoint['z']
            # 强制所有航点高度为3米
            waypoint['z'] = 3.0
            if abs(waypoint['z'] - original_z) > 0.1:
                modified_count += 1
                rospy.logwarn(f"航点 {i} 高度修正: {original_z:.1f}m -> 3.0m")
        
        if modified_count > 0:
            rospy.logwarn(f"共修改了 {modified_count} 个航点高度为固定3米")
        else:
            rospy.loginfo(f"所有 {len(self.waypoints)} 个航点高度已固定在3米")
    
    def _pose_callback(self, msg):
        """位置回调函数"""
        self.current_position = msg.pose.position
        q = msg.pose.orientation
        q_ = Quaternion(q.w, q.x, q.y, q.z)
        self.current_yaw = q_.yaw_pitch_roll[0]
    
    def _state_callback(self, msg):
        """MAVROS状态回调"""
        self.mavros_state = msg
    
    def _tracking_status_callback(self, msg):
        """人体跟踪状态回调 - 简化版本"""
        was_tracking = self.tracking_active
        self.tracking_active = msg.data
        
        if not was_tracking and self.tracking_active:
            # 开始跟踪，暂停航点飞行
            self.tracking_paused_waypoint = self.current_waypoint_index
            rospy.loginfo_throttle(2.0, f"🎯 开始人体跟踪，暂停航点 {self.current_waypoint_index}")
            
        elif was_tracking and not self.tracking_active:
            # 停止跟踪，恢复航点飞行 - 但不要重置航点索引
            rospy.loginfo_throttle(2.0, f"⏸️ 停止跟踪，恢复航点飞行")
            # 不重置waypoint_reached，让航点自然继续
    
    def _tracking_control_callback(self, msg):
        """人体跟踪控制指令回调 - 仅更新时间戳，不直接控制"""
        # 仅更新跟踪控制时间戳，不直接发布控制指令
        # 控制逻辑统一在主飞行循环中处理，避免控制冲突
        self.last_tracking_control_time = rospy.get_time()
        
        # 存储跟踪指令供主循环使用
        self.latest_tracking_cmd = msg
    
    def _manual_cmd_callback(self, msg):
        """手动控制指令回调"""
        cmd = msg.data.lower().strip()
        if cmd == "start":
            rospy.loginfo("📡 收到手动启动指令")
            if not self.mission_active:
                # 在新线程中启动任务，避免阻塞回调
                import threading
                threading.Thread(target=self.start_mission, daemon=True).start()
        elif cmd == "stop":
            rospy.loginfo("⏹️ 收到停止指令")
            self.mission_active = False
        elif cmd == "land":
            rospy.loginfo("🛬 收到降落指令")
            self._set_mode("AUTO.LAND")
    
    def _ensure_connection(self):
        """确保MAVROS和PX4完全连接并就绪"""
        rospy.loginfo(f"⏳ 等待MAVROS和PX4连接 - {self.vehicle_ns}...")
        
        max_wait_time = 60.0  # 最多等待60秒
        check_interval = 0.5  # 每0.5秒检查一次
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            # 检查超时
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > max_wait_time:
                rospy.logerr(f"❌ 连接超时({elapsed:.1f}s) - {self.vehicle_ns}")
                rospy.logerr("请确保Gazebo和PX4已完全启动（等待60秒）")
                return False
            
            # 等待状态消息
            try:
                state_msg = rospy.wait_for_message(f'/{self.vehicle_ns}/mavros/state', State, timeout=5.0)
                
                # 关键：检查PX4是否真正连接且有有效模式
                if state_msg.connected and state_msg.mode != "":
                    rospy.loginfo(f"✅ MAVROS和PX4已完全连接 - {self.vehicle_ns}")
                    rospy.loginfo(f"   当前模式: {state_msg.mode}, 连接状态: {state_msg.connected}")
                    
                    # 额外检查：确保系统稳定
                    rospy.sleep(0.5)
                    return True
                else:
                    # PX4未完全就绪，继续等待
                    if elapsed % 2.0 < check_interval:  # 每2秒打印一次
                        rospy.loginfo(f"⏳ 等待PX4就绪... ({elapsed:.1f}s) - {self.vehicle_ns}")
                        rospy.loginfo(f"   当前状态: mode='{state_msg.mode}', connected={state_msg.connected}")
                    rospy.sleep(check_interval)
                    
            except rospy.ROSException:
                if elapsed % 2.0 < check_interval:
                    rospy.loginfo(f"⏳ 等待MAVROS话题... ({elapsed:.1f}s) - {self.vehicle_ns}")
                rospy.sleep(check_interval)
        
        return False
    
    def _configure_gimbal(self):
        """配置云台初始设置"""
        try:
            from std_msgs.msg import Header
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "map"
            
            self.mount_cfg_srv(header=header, mode=2, stabilize_roll=0, stabilize_yaw=0, stabilize_pitch=0)
            rospy.loginfo(f"✅ 云台已配置 - {self.vehicle_ns}")
        except Exception as e:
            rospy.logwarn(f"⚠️ 云台配置失败 ({self.vehicle_ns}): {e}")
    
    def _publish_gimbal(self, pitch_deg=-45.0, yaw_deg=0.0, roll_deg=0.0):
        """发布云台控制指令，设置俯仰角为-45°"""
        msg = MountControl()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.mode = 2  # MAV_MOUNT_MODE_MAVLINK_TARGETING
        msg.pitch = pitch_deg
        msg.yaw = yaw_deg
        msg.roll = roll_deg
        self.mount_pub.publish(msg)
        rospy.logdebug_throttle(10.0, f"发布云台指令: pitch={pitch_deg}° - {self.vehicle_ns}")
    
    def _send_initial_setpoints(self, count=100):
        """发送初始setpoint，PX4要求进入OFFBOARD前发送"""
        target = PositionTarget()
        target.coordinate_frame = self.velocity_frame
        target.type_mask = (PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +
                            PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                            PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE)
        target.velocity.x = 0.0
        target.velocity.y = 0.0
        target.velocity.z = 0.0
        
        rate = rospy.Rate(20)  # 20Hz
        for i in range(count):
            self.setpoint_pub.publish(target)
            rate.sleep()
            if i % 20 == 0:  # 每秒打印一次进度
                rospy.loginfo(f"发送初始setpoint... {i+1}/{count}")
    
    def _background_setpoint_sender(self):
        """后台线程：30Hz持续发送setpoint保持OFFBOARD模式
        
        完全参考XTDrone官方communication.py：持续30Hz发送
        """
        rate = rospy.Rate(30)  # 30Hz，与XTDrone官方完全一致
        
        rospy.loginfo(f"🔄 后台setpoint保持线程已启动(30Hz) - 无人机 {self.drone_id}")
        
        # 初始setpoint
        initial_target = PositionTarget()
        initial_target.coordinate_frame = self.position_frame
        initial_target.type_mask = (PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
                                  PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                                  PositionTarget.IGNORE_YAW_RATE)
        initial_target.position.x = 0.0
        initial_target.position.y = 0.0
        initial_target.position.z = 0.0
        initial_target.yaw = 0.0
        
        while not rospy.is_shutdown() and self.background_setpoint_active:
            try:
                # 无条件持续发送setpoint（参考XTDrone官方communication.py）
                # 只在主循环未活动时发送，避免冲突
                if not self.mission_active:
                    if self.current_position:
                        # 有位置信息，发送当前位置
                        target = PositionTarget()
                        target.coordinate_frame = self.position_frame
                        target.type_mask = (PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
                                          PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                                          PositionTarget.IGNORE_YAW_RATE)
                        
                        target.position.x = self.current_position.x
                        target.position.y = self.current_position.y
                        target.position.z = max(self.current_position.z, 0.0)
                        target.yaw = self.current_yaw
                        
                        target.header.stamp = rospy.Time.now()
                        
                        self.setpoint_pub.publish(target)
                        rospy.logdebug_throttle(5.0, f"后台发送setpoint: ({self.current_position.x:.2f}, {self.current_position.y:.2f}, {self.current_position.z:.2f})")
                    else:
                        # 无位置信息，发送初始setpoint（原点）
                        self.setpoint_pub.publish(initial_target)
                        rospy.logdebug_throttle(5.0, "后台发送初始setpoint (0,0,0)")
                
            except Exception as e:
                rospy.logwarn_throttle(10.0, f"后台setpoint错误: {e}")
            
            rate.sleep()
        
        rospy.loginfo(f"🔄 后台setpoint保持线程已停止 - 无人机 {self.drone_id}")
    
    def _start_background_setpoint_sender(self):
        """启动后台setpoint发送线程"""
        if not self.background_setpoint_active:
            self.background_setpoint_active = True
            self.background_thread = threading.Thread(
                target=self._background_setpoint_sender,
                daemon=True
            )
            self.background_thread.start()
            rospy.loginfo(f"✅ 后台setpoint发送器已启动 - 无人机 {self.drone_id}")
    
    def _stop_background_setpoint_sender(self):
        """停止后台setpoint发送线程"""
        self.background_setpoint_active = False
        if self.background_thread:
            self.background_thread.join(timeout=2.0)
            rospy.loginfo(f"⏹️ 后台setpoint发送器已停止 - 无人机 {self.drone_id}")
    
    def _set_mode(self, mode):
        """设置飞行模式"""
        try:
            response = self.set_mode_client(custom_mode=mode)
            if response.mode_sent:
                rospy.loginfo(f"✅ 飞行模式设置为: {mode}")
                return True
            else:
                rospy.logwarn(f"❌ 飞行模式设置失败: {mode}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"设置飞行模式服务调用失败: {e}")
            return False
    
    def _arm_vehicle(self):
        """解锁无人机（带重试机制）"""
        max_attempts = 5
        for attempt in range(max_attempts):
            try:
                # 检查MAVROS状态
                if self.mavros_state is None:
                    rospy.logwarn(f"⚠️ MAVROS状态未就绪，等待... (尝试 {attempt+1}/{max_attempts})")
                    rospy.sleep(1.0)
                    continue
                
                # 检查是否已经在OFFBOARD模式
                if self.mavros_state.mode != "OFFBOARD":
                    rospy.logwarn(f"⚠️ 模式不是OFFBOARD (当前: {self.mavros_state.mode})，重新设置...")
                    self._set_mode("OFFBOARD")
                    rospy.sleep(0.5)
                
                # 尝试解锁
                response = self.arming_client(True)
                if response.success:
                    rospy.loginfo(f"✅ 无人机已解锁 - 无人机 {self.drone_id}")
                    return True
                else:
                    rospy.logwarn(f"❌ 解锁失败 (尝试 {attempt+1}/{max_attempts})")
                    if attempt < max_attempts - 1:
                        rospy.loginfo("   等待1秒后重试...")
                        rospy.sleep(1.0)
                    
            except rospy.ServiceException as e:
                rospy.logerr(f"解锁服务调用失败 (尝试 {attempt+1}/{max_attempts}): {e}")
                if attempt < max_attempts - 1:
                    rospy.sleep(1.0)
        
        rospy.logerr(f"❌ 无人机解锁失败（已尝试{max_attempts}次） - 无人机 {self.drone_id}")
        return False
    
    def _takeoff(self):
        """起飞"""
        rospy.loginfo(f"🚁 开始起飞到 {self.takeoff_altitude}m - {self.vehicle_ns}")
        
        # 发送起飞位置指令
        takeoff_target = PositionTarget()
        takeoff_target.coordinate_frame = self.position_frame
        takeoff_target.type_mask = (PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
                                   PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                                   PositionTarget.IGNORE_YAW_RATE)
        
        if self.current_position:
            takeoff_target.position.x = self.current_position.x
            takeoff_target.position.y = self.current_position.y
        else:
            takeoff_target.position.x = 0.0
            takeoff_target.position.y = 0.0
        
        takeoff_target.position.z = self.takeoff_altitude
        takeoff_target.yaw = 0.0
        
        # 持续发送起飞指令
        rate = rospy.Rate(20)
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            takeoff_target.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(takeoff_target)
            
            # 检查是否到达起飞高度
            if (self.current_position and 
                abs(self.current_position.z - self.takeoff_altitude) < 0.3):
                rospy.loginfo(f"✅ 起飞完成，当前高度: {self.current_position.z:.2f}m - {self.vehicle_ns}")
                
                # 起飞完成后，继续稳定悬停1秒
                hover_start = rospy.Time.now()
                while (rospy.Time.now() - hover_start).to_sec() < 1.0:
                    takeoff_target.header.stamp = rospy.Time.now()
                    self.setpoint_pub.publish(takeoff_target)
                    rate.sleep()
                
                rospy.loginfo(f"✅ 起飞稳定完成 - {self.vehicle_ns}")
                return True
            
            # 实时显示起飞进度
            if self.current_position and rospy.get_time() % 1.0 < 0.05:
                rospy.loginfo(f"🚁 起飞中: {self.current_position.z:.2f}m / {self.takeoff_altitude:.2f}m")
            
            # 检查超时
            if (rospy.Time.now() - start_time).to_sec() > 30.0:
                rospy.logwarn(f"⚠️ 起飞超时 - {self.vehicle_ns}")
                return False
            
            rate.sleep()
        
        return False
    
    def _check_waypoint_reached(self):
        """检查是否到达航点"""
        if not self.current_position or self.current_waypoint_index >= len(self.waypoints):
            return False
        
        current_wp = self.waypoints[self.current_waypoint_index]
        dx = current_wp['x'] - self.current_position.x
        dy = current_wp['y'] - self.current_position.y
        dz = current_wp['z'] - self.current_position.z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        return distance < self.waypoint_reached_threshold
    
    def _waypoint_control(self):
        """航点飞行控制"""
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            return None  # 任务完成
        
        current_wp = self.waypoints[self.current_waypoint_index]
        
        # 检查是否到达航点
        if self._check_waypoint_reached():
            if not self.waypoint_reached:
                self.waypoint_reached = True
                self.waypoint_start_time = rospy.Time.now().to_sec()
                rospy.loginfo(f"📍 到达航点 {self.current_waypoint_index}: ({current_wp['x']:.1f}, {current_wp['y']:.1f}, {current_wp['z']:.1f})")
            
            # 在航点停留1秒
            hover_time_elapsed = rospy.Time.now().to_sec() - self.waypoint_start_time
            if hover_time_elapsed < self.waypoint_hover_time:
                # 悬停指令
                hover_cmd = PositionTarget()
                hover_cmd.coordinate_frame = self.velocity_frame
                hover_cmd.type_mask = (PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +
                                     PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                                     PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE)
                hover_cmd.velocity.x = 0.0
                hover_cmd.velocity.y = 0.0
                hover_cmd.velocity.z = 0.0
                return hover_cmd
            else:
                # 停留时间结束，前往下一个航点
                self.current_waypoint_index += 1
                self.waypoint_reached = False
                if self.current_waypoint_index < len(self.waypoints):
                    next_wp = self.waypoints[self.current_waypoint_index]
                    rospy.loginfo(f"➡️ 前往航点 {self.current_waypoint_index}: ({next_wp['x']:.1f}, {next_wp['y']:.1f}, {next_wp['z']:.1f})")
                else:
                    rospy.loginfo("🏁 所有航点完成")
                    return None
        
        # 计算到航点的控制指令
        if not self.current_position:
            return None
        
        dx = current_wp['x'] - self.current_position.x
        dy = current_wp['y'] - self.current_position.y
        dz = current_wp['z'] - self.current_position.z
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 计算期望偏航角
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        
        # 计算速度控制指令（提高响应速度）
        if distance > 0.5:
            # 速度控制 - 快速前进
            vx = self.waypoint_velocity * math.cos(yaw_error)
            vy = self.waypoint_velocity * math.sin(yaw_error)
            vz = np.clip(dz * 1.0, -1.0, 1.0)  # 提高垂直速度
            yaw_rate = np.clip(yaw_error * 1.5, -self.yaw_rate_limit, self.yaw_rate_limit)
        else:
            vx = 0.0
            vy = 0.0
            vz = np.clip(dz * 0.5, -0.5, 0.5)
            yaw_rate = np.clip(yaw_error * 1.0, -self.yaw_rate_limit, self.yaw_rate_limit)
        
        # 高度安全检查
        if self.current_position:
            current_height = self.current_position.z
            if current_height >= self.max_altitude - 0.2:
                vz = min(vz, -0.1)  # 强制下降
            elif current_height <= self.min_altitude + 0.2:
                vz = max(vz, 0.1)   # 强制上升
        
        # 创建速度控制指令
        cmd = PositionTarget()
        cmd.coordinate_frame = self.velocity_frame
        cmd.type_mask = (PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +
                        PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                        PositionTarget.IGNORE_YAW)
        cmd.velocity.x = vx
        cmd.velocity.y = vy
        cmd.velocity.z = vz
        cmd.yaw_rate = yaw_rate
        
        return cmd
    
    def _flight_loop(self):
        """主飞行循环"""
        rate = rospy.Rate(20)  # 20Hz
        
        while not rospy.is_shutdown() and self.mission_active:
            try:
                current_time = rospy.get_time()
                
                # 检查是否最近收到了跟踪控制指令（1.5秒内）
                time_since_tracking = current_time - self.last_tracking_control_time
                actively_tracking = time_since_tracking < 1.5 and self.tracking_active
                
                if actively_tracking:
                    # 正在执行跟踪，发布跟踪控制指令
                    if self.latest_tracking_cmd:
                        # 将Twist(FLU: X前, Y左, Z上) 转换为 PositionTarget(BODY_NED: X前, Y右, Z下)
                        cmd = self.latest_tracking_cmd
                        target = PositionTarget()
                        target.coordinate_frame = PositionTarget.FRAME_BODY_NED
                        target.type_mask = (PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +
                                          PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                                          PositionTarget.IGNORE_YAW)

                        target.velocity.x = cmd.linear.x
                        target.velocity.y = -cmd.linear.y
                        target.velocity.z = -cmd.linear.z
                        # Yaw 由 ENU(绕+Z逆时针为正) 转 NED(绕+Z顺时针为正)，取反
                        target.yaw_rate = -cmd.angular.z
                        
                        target.header.stamp = rospy.Time.now()
                        target.header.frame_id = "base_link"
                        
                        self.setpoint_pub.publish(target)
                        
                    self.flight_status_pub.publish(String(data="TRACKING_ACTIVE"))
                    rospy.logdebug_throttle(2.0, f"🎯 正在执行人体跟踪 (距离上次跟踪指令: {time_since_tracking:.2f}s)")
                else:
                    # 执行航点飞行控制
                    if self.tracking_active and time_since_tracking >= 1.5:
                        # 跟踪超时，恢复航点飞行
                        rospy.loginfo_throttle(5.0, "⏰ 跟踪超时，恢复航点飞行")
                        self.tracking_active = False
                        # 不重置航点索引，继续当前航点
                    
                    # 航点飞行控制
                    control_cmd = self._waypoint_control()
                    
                    if control_cmd is None:
                        # 任务完成
                        rospy.loginfo("🏁 航点飞行任务完成")
                        self.mission_active = False
                        self.flight_status_pub.publish(String(data="MISSION_COMPLETE"))
                        break
                    else:
                        # 发布航点控制指令
                        self.setpoint_pub.publish(control_cmd)
                        self.flight_status_pub.publish(String(data="WAYPOINT_FLIGHT"))
                
                # 云台由独立gimbal_control节点控制，此处不再发送
                
            except Exception as e:
                rospy.logerr(f"飞行循环错误: {e}")
            
            rate.sleep()
    
    def start_mission(self):
        """开始任务 - 参考XTDrone官方communication.py简化流程"""
        rospy.loginfo(f"🚀 开始航点飞行任务 - 无人机 {self.drone_id}")
        
        # 1. 确保连接
        if not self._ensure_connection():
            return False
        
        # 2. 等待后台线程发送足够的setpoint（后台线程已在run()中启动）
        rospy.loginfo("⏳ 等待后台setpoint生效...")
        rospy.sleep(2.0)
        
        # 3. 设置OFFBOARD模式
        rospy.loginfo("🔧 设置OFFBOARD模式...")
        if not self._set_mode("OFFBOARD"):
            rospy.logerr("❌ OFFBOARD模式设置失败")
            return False
        
        rospy.sleep(0.5)
        
        # 4. 解锁（带重试）
        rospy.loginfo("🔓 解锁无人机...")
        if not self._arm_vehicle():
            rospy.logerr("❌ 无人机解锁失败")
            return False
        
        rospy.sleep(0.5)
        
        # 5. 起飞
        if not self._takeoff():
            return False
        
        # 6. 云台由独立gimbal_control节点控制（参考XTDrone官方）
        rospy.loginfo(f"📷 云台由gimbal_control节点控制 - {self.vehicle_ns}")
        rospy.loginfo(f"   请确保gimbal_control节点已在launch文件中启动")
        
        # 7. 开始任务（停止后台setpoint，主循环接管）
        self.mission_active = True
        self._stop_background_setpoint_sender()
        rospy.loginfo("✅ 航点飞行任务已启动")
        
        # 8. 启用人体跟踪
        try:
            enable_pub = rospy.Publisher(f'/drone_{self.drone_id}/human_tracker/enable', Bool, queue_size=1)
            rospy.sleep(1.0)
            
            for i in range(2):
                enable_pub.publish(Bool(data=True))
                rospy.sleep(0.3)
                
            rospy.loginfo("✅ 人体跟踪已启用")
        except Exception as e:
            rospy.logwarn(f"启用人体跟踪失败: {e}")
        
        return True
    
    def run(self):
        """运行航点飞行器 - 参考XTDrone官方communication.py"""
        try:
            # 立即启动后台setpoint线程（无延迟，参考XTDrone官方）
            rospy.loginfo(f"🔄 启动后台setpoint保持线程 - 无人机 {self.drone_id}")
            self._start_background_setpoint_sender()
            
            # 等待MAVROS连接
            rospy.loginfo(f"⏳ 等待系统准备 - 无人机 {self.drone_id}")
            rospy.sleep(3.0)  # 只等待MAVROS连接，不分先后
            
            # 自动开始任务（无延迟，所有无人机同时启动）
            rospy.loginfo(f"🚀 开始航点飞行任务 - 无人机 {self.drone_id}")
            if self.start_mission():
                # 运行飞行循环
                self._flight_loop()
            else:
                rospy.logerr(f"❌ 任务启动失败 - 无人机 {self.drone_id}")
                rospy.logerr("请检查:")
                rospy.logerr("1. Gazebo是否已启动")
                rospy.logerr("2. MAVROS是否已连接")
                rospy.logerr("3. 无人机模型是否已加载")
            
        except rospy.ROSInterruptException:
            rospy.loginfo(f"航点飞行节点关闭 - 无人机 {self.drone_id}")
        except Exception as e:
            rospy.logerr(f"航点飞行运行错误 - 无人机 {self.drone_id}: {e}")
        finally:
            # 确保后台线程停止
            self._stop_background_setpoint_sender()

if __name__ == '__main__':
    try:
        flight = WaypointFlight()
        flight.run()
    except Exception as e:
        rospy.logerr(f"航点飞行节点启动失败: {e}")
