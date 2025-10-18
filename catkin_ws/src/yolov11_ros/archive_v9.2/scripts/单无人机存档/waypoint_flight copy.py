#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
航点飞行节点
功能：专门负责航点飞行控制
与人体跟踪节点通过ROS话题通信，实现优先级控制
"""

import rospy
import math
import numpy as np
import json
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from mavros_msgs.msg import PositionTarget, State, MountControl
from mavros_msgs.srv import CommandBool, SetMode, MountConfigure
from geometry_msgs.msg import Twist
from pyquaternion import Quaternion

class WaypointFlight:
    def __init__(self):
        rospy.init_node('waypoint_flight', anonymous=True)
        
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
        
        # 使用可配置命名空间的MAVROS话题
        vehicle_ns = rospy.get_param('~vehicle_ns', 'typhoon_h480_0')
        
        # ROS发布者
        self.setpoint_pub = rospy.Publisher(f'/{vehicle_ns}/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.flight_status_pub = rospy.Publisher('/waypoint_flight/status', String, queue_size=1)
        self.mount_pub = rospy.Publisher(f'/{vehicle_ns}/mavros/mount_control/command', MountControl, queue_size=1)
        
        # ROS订阅者
        self.pose_sub = rospy.Subscriber(f'/{vehicle_ns}/mavros/local_position/pose', PoseStamped, self._pose_callback)
        self.state_sub = rospy.Subscriber(f'/{vehicle_ns}/mavros/state', State, self._state_callback)
        self.tracking_status_sub = rospy.Subscriber('/human_tracker/status', Bool, self._tracking_status_callback)
        self.tracking_control_sub = rospy.Subscriber('/human_tracker/cmd_vel', Twist, self._tracking_control_callback)
        
        # 手动控制订阅者
        self.manual_cmd_sub = rospy.Subscriber('/waypoint_flight/manual_cmd', String, self._manual_cmd_callback)
        
        # ROS服务
        self.arming_client = rospy.ServiceProxy(f'/{vehicle_ns}/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy(f'/{vehicle_ns}/mavros/set_mode', SetMode)
        self.mount_cfg_srv = rospy.ServiceProxy(f'/{vehicle_ns}/mavros/mount_control/configure', MountConfigure)
        
        # 加载航点
        self._load_waypoints()
        
        rospy.loginfo("✈️ 航点飞行节点已启动")
        rospy.loginfo("📡 手动控制指令:")
        rospy.loginfo("  启动任务: rostopic pub /waypoint_flight/manual_cmd std_msgs/String 'start'")
        rospy.loginfo("  停止任务: rostopic pub /waypoint_flight/manual_cmd std_msgs/String 'stop'")
        rospy.loginfo("  降落: rostopic pub /waypoint_flight/manual_cmd std_msgs/String 'land'")
    
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
        """确保MAVROS连接"""
        rospy.loginfo("等待MAVROS连接...")
        
        # 等待MAVROS状态话题
        try:
            rospy.wait_for_message(f'/typhoon_h480_0/mavros/state', State, timeout=15.0)
            rospy.loginfo("✅ MAVROS已连接")
            return True
        except rospy.ROSException:
            rospy.logerr("❌ MAVROS连接超时")
            rospy.logerr("请确保Gazebo和MAVROS已启动")
            return False
    
    def _configure_gimbal(self):
        """配置云台初始设置"""
        try:
            from std_msgs.msg import Header
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "map"
            
            self.mount_cfg_srv(header=header, mode=2, stabilize_roll=0, stabilize_yaw=0, stabilize_pitch=0)
            rospy.loginfo("✅ 云台已配置")
        except Exception as e:
            rospy.logwarn(f"⚠️ 云台配置失败: {e}")
    
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
        """解锁无人机"""
        try:
            response = self.arming_client(True)
            if response.success:
                rospy.loginfo("✅ 无人机已解锁")
                return True
            else:
                rospy.logwarn("❌ 无人机解锁失败")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"解锁服务调用失败: {e}")
            return False
    
    def _takeoff(self):
        """起飞"""
        rospy.loginfo(f"🚁 开始起飞到 {self.takeoff_altitude}m")
        
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
            self.setpoint_pub.publish(takeoff_target)
            
            # 检查是否到达起飞高度（放宽阈值，加快起飞）
            if (self.current_position and 
                abs(self.current_position.z - self.takeoff_altitude) < 0.5):
                rospy.loginfo(f"✅ 起飞完成，当前高度: {self.current_position.z:.2f}m")
                return True
            
            # 实时显示起飞进度
            if self.current_position and rospy.get_time() % 1.0 < 0.05:
                rospy.loginfo(f"🚁 起飞中: {self.current_position.z:.2f}m / {self.takeoff_altitude:.2f}m")
            
            # 检查超时（增加到45秒，确保有足够时间）
            if (rospy.Time.now() - start_time).to_sec() > 45.0:
                rospy.logwarn("⚠️ 起飞超时")
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
                
                # 定期发送云台控制指令，保持-45°俯仰角
                self._publish_gimbal(pitch_deg=-45.0)
                
            except Exception as e:
                rospy.logerr(f"飞行循环错误: {e}")
            
            rate.sleep()
    
    def start_mission(self):
        """开始任务"""
        rospy.loginfo("🚀 开始航点飞行任务")
        
        # 1. 确保连接
        if not self._ensure_connection():
            return False
        
        # 2. 发送初始setpoint（PX4要求）
        rospy.loginfo("📡 发送初始setpoint...")
        self._send_initial_setpoints()
        
        # 3. 设置OFFBOARD模式
        if not self._set_mode("OFFBOARD"):
            return False
        
        # 4. 解锁
        if not self._arm_vehicle():
            return False
        
        # 5. 起飞
        if not self._takeoff():
            return False
        
        # 6. 配置云台
        rospy.loginfo("📷 配置云台...")
        self._configure_gimbal()
        rospy.sleep(1.0)  # 等待配置生效
        self._publish_gimbal(pitch_deg=-45.0)  # 设置俯仰角-45°
        rospy.loginfo("✅ 云台已设置为-45°俯仰角")
        
        # 7. 开始任务
        self.mission_active = True
        rospy.loginfo("✅ 航点飞行任务已启动")
        
        # 启用人体跟踪
        try:
            enable_pub = rospy.Publisher('/human_tracker/enable', Bool, queue_size=1)
            rospy.sleep(2.0)  # 增加等待时间确保连接建立
            
            # 多次发送启用信号确保接收
            for i in range(3):
                enable_pub.publish(Bool(data=True))
                rospy.sleep(0.5)
                rospy.loginfo(f"🎯 人体跟踪启用信号已发送 ({i+1}/3)")
                
            rospy.loginfo("✅ 人体跟踪已启用")
        except Exception as e:
            rospy.logwarn(f"启用人体跟踪失败: {e}")
        
        return True
    
    def run(self):
        """运行航点飞行器"""
        try:
            # 等待系统准备
            rospy.loginfo("⏳ 等待系统初始化...")
            rospy.sleep(5.0)  # 增加等待时间确保所有系统准备就绪
            
            # 自动开始任务
            rospy.loginfo("🚀 自动启动航点飞行任务...")
            if self.start_mission():
                # 运行飞行循环
                self._flight_loop()
            else:
                rospy.logerr("❌ 任务启动失败")
                rospy.logerr("请检查:")
                rospy.logerr("1. Gazebo是否已启动")
                rospy.logerr("2. MAVROS是否已连接")
                rospy.logerr("3. 无人机模型是否已加载")
            
        except rospy.ROSInterruptException:
            rospy.loginfo("航点飞行节点关闭")
        except Exception as e:
            rospy.logerr(f"航点飞行运行错误: {e}")

if __name__ == '__main__':
    try:
        flight = WaypointFlight()
        flight.run()
    except Exception as e:
        rospy.logerr(f"航点飞行节点启动失败: {e}")
