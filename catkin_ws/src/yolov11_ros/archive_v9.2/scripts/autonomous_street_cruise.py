#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import threading
import time
import math
import platform

import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge

from mavros_msgs.msg import PositionTarget, State, MountControl
from mavros_msgs.srv import CommandBool, SetMode, MountConfigure
from pyquaternion import Quaternion



class StreetCruiseYolo:
    def __init__(self, vehicle_type="typhoon_h480", vehicle_id="0"):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        self.node_name = f"{vehicle_type}_{vehicle_id}_street_cruise_yolo"

        rospy.init_node(self.node_name, anonymous=True)
        self.rate_hz = rospy.get_param("~rate", 30)
        self.rate = rospy.Rate(self.rate_hz)

        # 状态
        self.current_position = None
        self.current_yaw = 0.0
        self.armed = False
        self.offboard = False
        self.should_land = False
        self.mission_active = False
        
        # 航点飞行相关
        self.waypoints = []
        self.current_waypoint_index = 0
        self.waypoint_reached_threshold = 5.0  # 航点到达阈值（米）
        self.waypoint_hover_time = 2.0  # 每个航点停留时间（秒）
        self.waypoint_start_time = 0.0  # 到达航点的时间
        self.waypoint_reached = False
        self.waypoint_velocity = 3.0  # 航点间飞行速度 3m/s
        
        # 云台控制参数
        self.gimbal_yaw = 0.0  # 云台当前偏航角
        self.target_gimbal_yaw = 0.0  # 云台目标偏航角

        # 传感器
        self.bridge = CvBridge()
        self.rgb_image = None
        

        # 话题 - 参考autonomous_street_cruise.py的配置
        base = f"{self.vehicle_type}_{self.vehicle_id}"
        self.pose_topic = rospy.get_param("~pose_topic", f"{base}/mavros/local_position/pose")
        # 图像话题：使用与yolov11一致的cgo3_camera话题作为默认
        # 相机话题：允许通过 ~image_topic/~rgb_topic 指定；否则使用cgo3_camera
        self.rgb_topic = rospy.get_param("~image_topic", rospy.get_param("~rgb_topic", f"/{base}/cgo3_camera/image_raw"))
        self.gps_topic = rospy.get_param("~gps_topic", f"{base}/mavros/global_position/global")
        
        
        # 控制
        self.setpoint_pub = rospy.Publisher(f"{base}/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        self.mount_pub = rospy.Publisher(f"{base}/mavros/mount_control/command", MountControl, queue_size=1)
        self.arm_srv = rospy.ServiceProxy(f"{base}/mavros/cmd/arming", CommandBool)
        self.mode_srv = rospy.ServiceProxy(f"{base}/mavros/set_mode", SetMode)
        self.mount_cfg_srv = rospy.ServiceProxy(f"{base}/mavros/mount_control/configure", MountConfigure)

        # 订阅
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_cb, queue_size=1)
        self.rgb_sub = rospy.Subscriber(self.rgb_topic, Image, self._rgb_cb, queue_size=1, buff_size=52428800)
        self.gps_fix = None
        self.gps_sub = rospy.Subscriber(self.gps_topic, NavSatFix, self._gps_cb, queue_size=1)

        # 键盘监听
        self.key_sub = rospy.Subscriber("/street_cruise_yolo/keyboard", String, self._keyboard_cb, queue_size=1)
        self._start_keyboard_thread()

        # 云台初始化
        self._configure_gimbal()

        # 飞行控制参数
        self.yaw_rate_limit = rospy.get_param("~yaw_rate_limit", 1.0)
        self.max_forward_velocity = rospy.get_param("~max_forward_velocity", 6.0)
        self.lateral_velocity = rospy.get_param("~lateral_velocity", 2.0)
        
        # 起飞参数
        self.takeoff_altitude = rospy.get_param("~takeoff_altitude", 4.5)
        self.takeoff_timeout_s = rospy.get_param("~takeoff_timeout_s", 20.0)
        self.hover_yaw = rospy.get_param("~hover_yaw", 0.0)

        # 控制坐标系
        self.velocity_frame = 8  # BODY_NED
        self.position_frame = 1  # LOCAL_NED

        # 状态机
        self.flight_mode = "WAYPOINT_FLIGHT"  # 航点飞行模式
        
        # 加载航点数据
        self._load_waypoints()
        
        # 状态打印
        rospy.loginfo(f"{self.node_name}: 初始化完成，等待位姿数据...")
        rospy.loginfo(f"已加载 {len(self.waypoints)} 个航点")

    def _load_waypoints(self):
        """加载航点数据"""
        waypoint_file = rospy.get_param("~waypoint_file", "way.txt")
        # 尝试多个可能的路径
        possible_paths = [
            os.path.join(os.path.dirname(__file__), "..", waypoint_file),
            os.path.join(os.path.dirname(__file__), waypoint_file),
            waypoint_file
        ]
        
        waypoint_path = None
        for path in possible_paths:
            if os.path.exists(path):
                waypoint_path = path
                break
        
        if waypoint_path:
            try:
                with open(waypoint_path, 'r') as f:
                    lines = f.readlines()
                
                for line in lines:
                    line = line.strip()
                    if line and not line.startswith('x'):  # 跳过标题行
                        parts = line.split()
                        if len(parts) >= 3:
                            x = float(parts[0])
                            y = float(parts[1])
                            z = float(parts[2])
                            self.waypoints.append({'x': x, 'y': y, 'z': z})
                
                rospy.loginfo(f"成功从 {waypoint_path} 加载 {len(self.waypoints)} 个航点")
                for i, wp in enumerate(self.waypoints):
                    rospy.loginfo(f"  航点 {i}: ({wp['x']:.1f}, {wp['y']:.1f}, {wp['z']:.1f})")
                    
            except Exception as e:
                rospy.logerr(f"加载航点文件失败: {e}")
                self.waypoints = []
        else:
            rospy.logerr(f"未找到航点文件: {waypoint_file}")
            self.waypoints = []
        
        # 如果没有成功加载航点，使用默认航点
        if not self.waypoints:
            self.waypoints = [
                {'x': 0, 'y': 0, 'z': 5},
                {'x': 105, 'y': 0, 'z': 5},
                {'x': 105, 'y': 45, 'z': 5},
                {'x': 45, 'y': 45, 'z': 5},
                {'x': -45, 'y': 45, 'z': 5},
                {'x': -45, 'y': 0, 'z': 5},
                {'x': -15, 'y': 0, 'z': 5},
                {'x': -15, 'y': -45, 'z': 5},
                {'x': 45, 'y': -45, 'z': 5},
                {'x': 45, 'y': 45, 'z': 5},
                {'x': -15, 'y': 45, 'z': 5},
                {'x': -15, 'y': 0, 'z': 5},
                {'x': -45, 'y': 0, 'z': 5},
                {'x': -45, 'y': -45, 'z': 5},
                {'x': 105, 'y': -45, 'z': 5},
                {'x': 105, 'y': 45, 'z': 5},
                {'x': 45, 'y': 45, 'z': 5},
                {'x': 45, 'y': -45, 'z': 5},
                {'x': -15, 'y': -45, 'z': 5},
                {'x': -15, 'y': 0, 'z': 5},
                {'x': 0, 'y': 0, 'z': 5}
            ]
            rospy.logwarn(f"使用默认航点数据，共 {len(self.waypoints)} 个航点")

    def _check_waypoint_reached(self):
        """检查是否到达当前航点"""
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            return False
        
        if self.current_position is None:
            return False
        
        current_wp = self.waypoints[self.current_waypoint_index]
        dx = self.current_position.x - current_wp['x']
        dy = self.current_position.y - current_wp['y']
        dz = self.current_position.z - current_wp['z']
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        return distance <= self.waypoint_reached_threshold

    def _calculate_waypoint_yaw(self, target_wp):
        """计算朝向目标航点的偏航角"""
        if self.current_position is None:
            return self.current_yaw
        
        dx = target_wp['x'] - self.current_position.x
        dy = target_wp['y'] - self.current_position.y
        return math.atan2(dy, dx)
    

    def _waypoint_flight_control(self):
        """航点飞行控制"""
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            return 0.0, 0.0, 0.0, 0.0, True  # 任务完成
        
        current_wp = self.waypoints[self.current_waypoint_index]
        
        # 检查是否到达航点
        if self._check_waypoint_reached():
            if not self.waypoint_reached:
                self.waypoint_reached = True
                self.waypoint_start_time = time.time()
                rospy.loginfo(f"到达航点 {self.current_waypoint_index}: ({current_wp['x']:.1f}, {current_wp['y']:.1f}, {current_wp['z']:.1f})")
            
            # 在航点停留
            if time.time() - self.waypoint_start_time < self.waypoint_hover_time:
                return 0.0, 0.0, 0.0, 0.0, False  # 悬停
            
            # 停留时间结束，前往下一个航点
            self.current_waypoint_index += 1
            self.waypoint_reached = False
            if self.current_waypoint_index < len(self.waypoints):
                next_wp = self.waypoints[self.current_waypoint_index]
                rospy.loginfo(f"前往航点 {self.current_waypoint_index}: ({next_wp['x']:.1f}, {next_wp['y']:.1f}, {next_wp['z']:.1f})")
        
        # 计算到目标航点的距离和方向
        if self.current_position is None:
            return 0.0, 0.0, 0.0, 0.0, False
        
        dx = current_wp['x'] - self.current_position.x
        dy = current_wp['y'] - self.current_position.y
        dz = current_wp['z'] - self.current_position.z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # 计算期望的偏航角
        target_yaw = self._calculate_waypoint_yaw(current_wp)
        yaw_error = target_yaw - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        
        # 计算速度分量
        if distance > self.waypoint_reached_threshold:
            # 计算在机体坐标系下的速度分量
            vx = self.waypoint_velocity * math.cos(yaw_error)
            vy = self.waypoint_velocity * math.sin(yaw_error)
            vz = np.clip(dz * 0.5, -1.0, 1.0)  # 高度控制
            yaw_rate = np.clip(yaw_error * 1.0, -self.yaw_rate_limit, self.yaw_rate_limit)
        else:
            vx = 0.0
            vy = 0.0
            vz = np.clip(dz * 0.5, -1.0, 1.0)
            yaw_rate = np.clip(yaw_error * 1.0, -self.yaw_rate_limit, self.yaw_rate_limit)
        
        return vx, vy, vz, yaw_rate, False

    # 回调函数
    def _pose_cb(self, msg: PoseStamped):
        self.current_position = msg.pose.position
        q = msg.pose.orientation
        q_ = Quaternion(q.w, q.x, q.y, q.z)
        self.current_yaw = q_.yaw_pitch_roll[0]

    def _rgb_cb(self, msg: Image):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            try:
                self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            except Exception:
                self.rgb_image = None

    def _gps_cb(self, msg: NavSatFix):
        self.gps_fix = msg


    def _keyboard_cb(self, msg: String):
        key = msg.data.lower().strip()
        if key == 's':
            self.should_land = True
        elif key == 'r':
            # 重新开始航点飞行
            self.current_waypoint_index = 0
            self.waypoint_reached = False
            self.flight_mode = "WAYPOINT_FLIGHT"
            rospy.loginfo("重新开始航点飞行")

    # 键盘线程
    def _start_keyboard_thread(self):
        def kb_loop():
            if platform.system().lower().startswith('win'):
                try:
                    import msvcrt
                except Exception:
                    return
                while not rospy.is_shutdown():
                    if msvcrt.kbhit():
                        ch = msvcrt.getwch()
                        if ch and ch.lower() == 's':
                            self.should_land = True
                    time.sleep(0.05)
            else:
                import sys
                import select
                while not rospy.is_shutdown():
                    dr, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if dr:
                        ch = sys.stdin.read(1)
                        if ch and ch.lower() == 's':
                            self.should_land = True
        t = threading.Thread(target=kb_loop, daemon=True)
        t.start()

    # 云台控制
    def _configure_gimbal(self):
        try:
            header = rospy.Header()
        except Exception:
            import std_msgs.msg
            header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        try:
            self.mount_cfg_srv(header=header, mode=2, stabilize_roll=0, stabilize_yaw=0, stabilize_pitch=0)
            rospy.loginfo("云台已配置")
        except Exception as e:
            rospy.logwarn(f"云台配置失败: {e}")

    def _update_gimbal_target(self):
        """更新云台目标角度，基于航点间方向变化计算角度"""
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            self.target_gimbal_yaw = 0.0
            return
        
        # 如果有上一个航点，计算从上个航点到当前航点的方向角（当前航段）
        if self.current_waypoint_index > 0:
            prev_wp = self.waypoints[self.current_waypoint_index - 1]
            current_wp = self.waypoints[self.current_waypoint_index]
            
            # 计算航点间的方向向量
            dx = current_wp['x'] - prev_wp['x']
            dy = current_wp['y'] - prev_wp['y']
            
            # 计算方向角度（弧度）
            direction_angle = math.atan2(dy, dx)
            
            # 转换为角度并归一化到[-180, 180]范围
            angle_deg = math.degrees(direction_angle)
            angle_deg = (angle_deg + 180) % 360 - 180  # 归一化到[-180, 180]
            
            # 根据飞行方向设置云台角度
            # 正东方向(0度) -> 云台0度(正前方)
            # 正北方向(90度) -> 云台-90度(左侧)  
            # 正西方向(180度) -> 云台-180度(正后方)
            # 正南方向(-90度) -> 云台90度(右侧)
            self.target_gimbal_yaw = angle_deg
            
            rospy.loginfo_throttle(2.0, f"航段 {self.current_waypoint_index-1}->{self.current_waypoint_index}: "
                                  f"方向角 {angle_deg:.1f}°, 云台角度 {self.target_gimbal_yaw:.1f}°")
        else:
            # 第一个航点（起飞点），设为0度
            self.target_gimbal_yaw = 0.0

    def _publish_gimbal(self, pitch_deg=-45.0, yaw_deg=None, roll_deg=0.0):
        """发布云台控制指令，基于航点间方向变化计算角度"""
        msg = MountControl()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.mode = 2
        msg.pitch = pitch_deg
        
        # 如果未指定偏航角，使用基于航点间方向变化计算的角度
        if yaw_deg is None:
            self._update_gimbal_target()  # 更新目标角度（基于航点间方向）
            yaw_deg = self.target_gimbal_yaw
        
        msg.yaw = yaw_deg
        msg.roll = roll_deg
        self.mount_pub.publish(msg)


    # 飞行控制
    def _publish_velocity(self, vx, vy, vz, yaw_rate):
        target = PositionTarget()
        target.coordinate_frame = self.velocity_frame
        target.type_mask = (PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +
                            PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                            PositionTarget.IGNORE_YAW)
        target.velocity.x = float(np.clip(vx, -self.max_forward_velocity, self.max_forward_velocity))
        target.velocity.y = float(np.clip(vy, -self.lateral_velocity, self.lateral_velocity))
        target.velocity.z = 0.0
        target.yaw_rate = float(np.clip(yaw_rate, -self.yaw_rate_limit, self.yaw_rate_limit))
        self.setpoint_pub.publish(target)

    def _publish_position(self, x, y, z, yaw):
        t = PositionTarget()
        t.coordinate_frame = self.position_frame
        t.type_mask = (PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
                       PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                       PositionTarget.IGNORE_YAW_RATE)
        t.position.x = float(x)
        t.position.y = float(y)
        t.position.z = float(z)
        t.yaw = float(yaw)
        self.setpoint_pub.publish(t)

    # MAVROS操作
    def _ensure_connection(self, timeout=15):
        try:
            state = rospy.wait_for_message(f"{self.vehicle_type}_{self.vehicle_id}/mavros/state", State, timeout=timeout)
            return state.connected
        except Exception as e:
            rospy.logerr(f"等待mavros连接失败: {e}")
            return False

    def _send_initial_setpoints(self, count=100):
        target = PositionTarget()
        target.coordinate_frame = self.velocity_frame
        target.type_mask = (PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +
                            PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                            PositionTarget.IGNORE_YAW + 0)
        target.velocity.x = 0.0
        target.velocity.y = 0.0
        target.velocity.z = 0.0
        target.yaw_rate = 0.0
        for _ in range(count):
            self.setpoint_pub.publish(target)
            self.rate.sleep()

    def arm(self):
        try:
            if self.arm_srv(True):
                self.armed = True
                return True
        except Exception as e:
            rospy.logerr(f"解锁失败: {e}")
        return False

    def set_offboard(self):
        try:
            if self.mode_srv(custom_mode="OFFBOARD"):
                self.offboard = True
                return True
        except Exception as e:
            rospy.logerr(f"设置OFFBOARD失败: {e}")
        return False

    def set_auto_land(self):
        try:
            return self.mode_srv(custom_mode="AUTO.LAND")
        except Exception as e:
            rospy.logerr(f"设置降落失败: {e}")
            return False

    # 主流程
    def start(self):
        if not self._ensure_connection():
            rospy.logerr("MAVROS未连接")
            return False

        # 等待姿态；图像稍后在悬停阶段等待 - 复制自autonomous_street_cruise.py
        t0 = time.time()
        retried_img = False
        retried_pose = False
        while not rospy.is_shutdown():
            has_pose = self.current_position is not None
            if has_pose:
                break
            # 图像话题重试
            if time.time() - t0 > 10 and not retried_img:
                try:
                    topics = rospy.get_published_topics()
                except Exception:
                    topics = []
                image_topics = [t for t, ttype in topics if ttype == 'sensor_msgs/Image']
                # 候选优先级列表（包含yolo风格与带vehicle前缀的常见命名）
                base = f"{self.vehicle_type}_{self.vehicle_id}"
                candidates = [
                    self.rgb_topic,
                    f"/{base}/camera/color/image_raw",
                    f"/{base}/camera/rgb/image_raw",
                    f"/{base}/camera/image_raw",
                    f"/{base}/cgo3_camera/image_raw",
                    "/camera/color/image_raw",
                    "/camera/rgb/image_raw",
                    "/camera/image_raw",
                ]
                pick = None
                for c in candidates:
                    if c in image_topics:
                        pick = c
                        break
                if pick is None and len(image_topics) > 0:
                    pick = image_topics[0]
                if pick and pick != self.rgb_topic:
                    try:
                        self.rgb_sub.unregister()
                    except Exception:
                        pass
                    self.rgb_topic = pick
                    self.rgb_sub = rospy.Subscriber(self.rgb_topic, Image, self._rgb_cb, queue_size=1, buff_size=52428800)
                    rospy.loginfo(f"重订阅图像话题: {self.rgb_topic}")
                retried_img = True
            # 位姿话题重试
            if time.time() - t0 > 10 and not retried_pose and not has_pose:
                try:
                    topics = rospy.get_published_topics()
                except Exception:
                    topics = []
                pose_topics = [t for t, ttype in topics if ttype == 'geometry_msgs/PoseStamped']
                base = f"{self.vehicle_type}_{self.vehicle_id}"
                pose_candidates = [
                    self.pose_topic,
                    f"{base}/mavros/local_position/pose",
                    f"/{base}/mavros/local_position/pose",
                ]
                pose_pick = None
                for c in pose_candidates:
                    if c in pose_topics:
                        pose_pick = c
                        break
                if pose_pick is None and len(pose_topics) > 0:
                    pose_pick = pose_topics[0]
                if pose_pick and pose_pick != self.pose_topic:
                    try:
                        self.pose_sub.unregister()
                    except Exception:
                        pass
                    self.pose_topic = pose_pick
                    # 先阻塞拉取一帧，确保位姿初始化成功
                    try:
                        msg = rospy.wait_for_message(self.pose_topic, PoseStamped, timeout=5.0)
                        self._pose_cb(msg)
                        rospy.loginfo(f"收到首帧位姿: {self.pose_topic}")
                    except Exception as e:
                        rospy.logwarn(f"首帧位姿等待失败: {e}")
                    # 再订阅持续接收
                    self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_cb, queue_size=1)
                    rospy.loginfo(f"重订阅位姿话题: {self.pose_topic}")
                retried_pose = True
            if time.time() - t0 > 30:
                try:
                    topics = rospy.get_published_topics()
                    image_topics = [t for t, ttype in topics if ttype == 'sensor_msgs/Image']
                    pose_topics = [t for t, ttype in topics if ttype == 'geometry_msgs/PoseStamped']
                    rospy.logerr(f"等待位姿/图像超时。可用图像话题: {image_topics}, 可用位姿话题: {pose_topics}")
                except Exception:
                    rospy.logerr("等待位姿/图像超时")
                return False
            time.sleep(0.2)

        # 发送初始空速度setpoint确保OFFBOARD（机体系0速度）- 复制自autonomous_street_cruise.py
        self._send_initial_setpoints(100)
        if not self.set_offboard():
            return False
        time.sleep(1.0)
        if not self.arm():
            return False
        time.sleep(1.0)

        # 起飞到设定高度并保持悬停，期间等待图像 - 复制自autonomous_street_cruise.py
        hover_yaw = float(self.hover_yaw)
        takeoff_deadline = time.time() + float(self.takeoff_timeout_s)
        rospy.loginfo(f"起飞到 {self.takeoff_altitude:.1f} m 并悬停，等待Gazebo模型状态...")
        if self.current_position is None:
            rospy.logwarn("起飞前未获得当前位置，放弃")
            return False
        hold_x = self.current_position.x
        hold_y = self.current_position.y
        while not rospy.is_shutdown() and time.time() < takeoff_deadline:
            target_z = self.takeoff_altitude
            self._publish_position(hold_x, hold_y, target_z, hover_yaw)
            # 在起飞阶段设置云台角度为0度（正前方）
            self._publish_gimbal(yaw_deg=0.0)
            if self.current_position is not None and abs(self.current_position.z - target_z) < 0.3:
                break
            self.rate.sleep()

        # 等待起飞完成
        rospy.loginfo("起飞完成，准备开始航点飞行任务")

        self.mission_active = True
        rospy.loginfo("开始航点飞行任务：")
        rospy.loginfo("  按 s 降落")
        rospy.loginfo("  按 r 重新开始航点飞行")
        
        # 初始化控制变量
        vx_cmd = 0.0
        vy_cmd = 0.0
        vz_cmd = 0.0
        yaw_rate_cmd = 0.0

        # 主控制循环
        while not rospy.is_shutdown() and self.mission_active:
            if self.should_land:
                rospy.loginfo("收到降落指令")
                self.mission_active = False
                break

            # 航点飞行控制
            if self.flight_mode == "WAYPOINT_FLIGHT":
                vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd, mission_complete = self._waypoint_flight_control()
                
                if mission_complete:
                    rospy.loginfo("航点飞行任务完成！")
                    self.mission_active = False
                    break
                
                # 发布控制指令
                self._publish_velocity(vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd)
                self._publish_gimbal()  # 云台跟随无人机航向
                
                # 状态信息
                if self.current_waypoint_index < len(self.waypoints):
                    current_wp = self.waypoints[self.current_waypoint_index]
                    if self.current_position is not None:
                        dx = current_wp['x'] - self.current_position.x
                        dy = current_wp['y'] - self.current_position.y
                        dz = current_wp['z'] - self.current_position.z
                        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                        
                        # 显示云台角度信息
                        rospy.loginfo_throttle(2.0, f"航点飞行: {self.current_waypoint_index+1}/{len(self.waypoints)}, 距离: {distance:.2f}m, 速度: {vx_cmd:.2f}m/s, 航段角度: {self.target_gimbal_yaw:.1f}°")
                    else:
                        rospy.loginfo_throttle(2.0, f"航点飞行: {self.current_waypoint_index+1}/{len(self.waypoints)}, 等待位置数据")
                else:
                    rospy.loginfo_throttle(2.0, "航点飞行: 任务完成")
            
            self.rate.sleep()

        # 降落
        self.set_auto_land()
        time.sleep(8.0)
        return True


def main():
    try:
        drone = StreetCruiseYolo("typhoon_h480", "0")
        ok = drone.start()
        if ok:
            print("航点飞行任务完成或已进入降落")
        else:
            print("航点飞行任务启动失败")
    except KeyboardInterrupt:
        print("用户中断，尝试降落...")
        try:
            drone.set_auto_land()
        except Exception:
            pass
    except Exception as e:
        print(f"程序异常: {e}")


if __name__ == "__main__":
    main()
