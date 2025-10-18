#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
人体跟踪节点 - 完整版本（严格按照human_tracker copy.py）
"""

import rospy
import numpy as np
import time
import threading
import math
from geometry_msgs.msg import PoseStamped, Twist, Quaternion as GeometryQuaternion
from std_msgs.msg import String, Bool
from yolov11_ros_msgs.msg import BoundingBoxes
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
from sensor_msgs.msg import LaserScan, Range
from pyquaternion import Quaternion

class HumanTracker:
    def __init__(self):
        rospy.init_node('human_tracker', anonymous=True)
        
        # 获取vehicle_ns参数（关键！从launch文件传入）
        self.vehicle_ns = rospy.get_param('~vehicle_ns', 'typhoon_h480_0')
        self.drone_id = rospy.get_param('~drone_id', 0)
        
        # ========== 关键控制变量（来自human_tracker copy.py）==========
        self.tracking_active = True
        self.find_cnt = 0
        self.find_cnt_last = 0
        self.not_find_time = 0
        self.get_time = False
        
        # 高度和位置
        self.height = 0.0
        self.target_height = 3.0
        self.theta = math.radians(-45.0)  # 默认云台角度
        self.cam_pose_received = False
        
        # 激光测距
        self.laser_range = None
        self.laser_valid = False
        self.fused_distance = None
        
        # 检测相关
        self.detection_confidence = rospy.get_param("~detection_confidence", 0.3)
        self.human_position = None
        self.drone_position = None
        
        # 相机参数
        self.image_width = rospy.get_param("~image_width", 640)
        self.image_height = rospy.get_param("~image_height", 360)
        self.u_center = self.image_width / 2.0
        self.v_center = self.image_height / 2.0
        self.fx = rospy.get_param("~fx", 205.46963709898583)
        self.fy = rospy.get_param("~fy", 205.46963709898583)
        
        # 控制增益（XTDrone原始算法）
        self.Kp_xy = rospy.get_param("~Kp_xy", 0.5)
        self.Kp_z = rospy.get_param("~Kp_z", 1.0)
        
        # 速度限制
        self.max_vel = rospy.get_param("~max_vel", 2.0)
        self.max_vel_z = rospy.get_param("~max_vel_z", 1.0)
        
        # 控制指令（关键！）
        self.twist = Twist()
        self.cmd_string = ""
        
        # 检测结果
        self.latest_detections = None
        # 颜色类别作为目标（根据比赛要求）
        self.color_classes = ['blue', 'green', 'white', 'brown', 'red', 'red1', 'red2']
        # 也支持人类检测（兼容）
        self.human_classes = ['person', 'human']
        self.person_id = rospy.get_param('~person_id', 0)
        
        # 距离融合参数
        self.dist_alpha_laser = rospy.get_param("~dist_alpha_laser", 0.7)
        self.dist_alpha_vision = rospy.get_param("~dist_alpha_vision", 0.35)
        self.vision_distance_k = 0.5
        
        # 固定高度
        self.fixed_height = 3.0
        
        # 跟踪计时
        self.tracking_start_time = None
        
        # 视觉距离估计
        self.target_size_ratio = rospy.get_param("~target_size_ratio", 0.15)
        self.target_tracking_distance = 3.0
        
        # ========== ROS发布者（完整版，不精简）==========
        
        # 跟踪状态发布
        self.tracking_status_pub = rospy.Publisher('/human_tracker/status', Bool, queue_size=1)
        self.tracking_info_pub = rospy.Publisher('/human_tracker/info', String, queue_size=1)
        
        # XTDrone控制话题（主要）- 这是关键！
        self.cmd_vel_pub_xt = rospy.Publisher(f'/xtdrone/{self.vehicle_ns}/cmd_vel_flu', Twist, queue_size=1)
        self.cmd_pub_xt = rospy.Publisher(f'/xtdrone/{self.vehicle_ns}/cmd', String, queue_size=1)
        
        # 多机控制话题（重要！用于控制XTDrone多机系统）
        self.multi_cmd_pub = rospy.Publisher('/multi/cmd_accel', String, queue_size=1)
        
        # 兼容话题
        self.cmd_vel_pub = rospy.Publisher('/human_tracker/cmd_vel', Twist, queue_size=1)
        self.cmd_pub = rospy.Publisher('/human_tracker/cmd', String, queue_size=1)
        
        # 航点控制（关键！必须有这个才能暂停和恢复航点飞行）
        self.waypoint_pause_pub = rospy.Publisher(f'/drone_{self.drone_id}/waypoint_flight/pause', String, queue_size=1)
        self.waypoint_resume_pub = rospy.Publisher(f'/drone_{self.drone_id}/waypoint_flight/resume', String, queue_size=1)
        
        # 追踪声明（使用String类型）
        self.tracking_claim_pub = rospy.Publisher(f'/drone_{self.drone_id}/tracking_claim', String, queue_size=1)
        
        # 云台控制（保持-45度俯仰角）
        from mavros_msgs.msg import MountControl
        self.mount_pub = rospy.Publisher(f'/{self.vehicle_ns}/mavros/mount_control/command', MountControl, queue_size=1)
        
        # 记分系统发布者
        self.actor_blue_pub = rospy.Publisher('/actor_blue_info', ActorInfo, queue_size=1)
        self.actor_green_pub = rospy.Publisher('/actor_green_info', ActorInfo, queue_size=1)
        self.actor_white_pub = rospy.Publisher('/actor_white_info', ActorInfo, queue_size=1)
        self.actor_brown_pub = rospy.Publisher('/actor_brown_info', ActorInfo, queue_size=1)
        self.actor_red1_pub = rospy.Publisher('/actor_red1_info', ActorInfo, queue_size=1)
        self.actor_red2_pub = rospy.Publisher('/actor_red2_info', ActorInfo, queue_size=1)
        
        # ========== ROS订阅者 ==========
        # 从launch文件获取detection_topic参数
        detection_topic = rospy.get_param("~detection_topic", "/yolov11/bounding_boxes")
        pose_topic = rospy.get_param("~pose_topic", f"/{self.vehicle_ns}/mavros/local_position/pose")
        cam_pose_topic = rospy.get_param("~cam_pose_topic", f"/xtdrone/{self.vehicle_ns}/cam_pose")
        
        self.detection_sub = rospy.Subscriber(detection_topic, BoundingBoxes, self._detection_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self._pose_callback, queue_size=1)
        self.cam_pose_sub = rospy.Subscriber(cam_pose_topic, PoseStamped, self._cam_pose_callback, queue_size=1)
        self.enable_sub = rospy.Subscriber('/human_tracker/enable', Bool, self._enable_callback)
        
        # 激光订阅
        laser_scan_topic = rospy.get_param("~laser_scan_topic", f"/{self.vehicle_ns}/scan")
        laser_range_topic = rospy.get_param("~laser_range_topic", f"/{self.vehicle_ns}/laser_rangefinder/distance")
        
        try:
            self.laser_scan_sub = rospy.Subscriber(laser_scan_topic, LaserScan, self._laser_scan_callback, queue_size=1)
            rospy.loginfo(f"📡 订阅2D激光雷达: {laser_scan_topic}")
        except Exception as e:
            rospy.logwarn(f"无法订阅2D激光雷达: {e}")
        
        try:
            self.laser_range_sub = rospy.Subscriber(laser_range_topic, Range, self._laser_range_callback, queue_size=1)
            rospy.loginfo(f"📡 订阅激光测距仪: {laser_range_topic}")
        except Exception as e:
            rospy.logwarn(f"无法订阅激光测距仪: {e}")
        
        # 线程锁
        self.tracking_lock = threading.Lock()
        
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"🤖 人体跟踪节点已启动 - 无人机 {self.drone_id}")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"📡 Vehicle: {self.vehicle_ns}")
        rospy.loginfo(f"📡 检测话题: {detection_topic}")
        rospy.loginfo(f"📡 控制话题: /xtdrone/{self.vehicle_ns}/cmd_vel_flu")
        rospy.loginfo(f"📡 位置话题: {pose_topic}")
        rospy.loginfo("=" * 60)
    
    def _detection_callback(self, data):
        """YOLO检测结果回调 - 完整版本（参考human_tracker copy.py）"""
        rospy.logdebug_throttle(2.0, f"收到检测结果，包含 {len(data.bounding_boxes)} 个检测框")
        
        if not self.tracking_active:
            rospy.logwarn_throttle(5.0, "跟踪未启用，忽略检测结果")
            return
        
        human_detected = False
        all_detections = []
        best_human_target = None
        detected_cls_name = None
        
        for target in data.bounding_boxes:
            cls_name = getattr(target, 'Class', '')
            cls_id = getattr(target, 'id', None)
            prob = getattr(target, 'probability', 0.0)
            
            if cls_name:
                all_detections.append(f"{cls_name}({prob:.2f})")
            elif cls_id is not None:
                all_detections.append(f"id:{cls_id}({prob:.2f})")
            
            # 优先检测颜色类别（比赛目标）
            is_color_target = (cls_name.lower() in self.color_classes) if cls_name else False
            is_human_by_name = (cls_name in self.human_classes)
            is_human_by_id = (cls_id == self.person_id)
            
            if (is_color_target or is_human_by_name or is_human_by_id) and prob >= self.detection_confidence:
                human_detected = True
                best_human_target = target
                detected_cls_name = cls_name if cls_name else None
                tag = cls_name if cls_name else f'id:{cls_id}'
                rospy.loginfo_throttle(1.0, f'🎯 发现人体 ({tag}, 置信度: {prob:.3f})')
                break
            elif (is_human_by_name or is_human_by_id):
                tag = cls_name if cls_name else f'id:{cls_id}'
                rospy.logwarn_throttle(2.0, f'❌ 人体置信度过低: {tag} ({prob:.3f} < {self.detection_confidence})')
        
        if all_detections:
            rospy.logdebug_throttle(2.0, f"所有检测结果: {', '.join(all_detections)}")
        
        if human_detected and best_human_target:
            # 计算图像坐标
            u = (best_human_target.xmax + best_human_target.xmin) / 2.0
            v = (best_human_target.ymax + best_human_target.ymin) / 2.0
            u_ = u - self.u_center
            v_ = v - self.v_center
            
            # 记录跟踪开始时间并暂停航点飞行
            if self.tracking_start_time is None:
                self.tracking_start_time = rospy.get_time()
                rospy.loginfo("=" * 60)
                rospy.loginfo("🎯 开始人体追踪，暂停航点飞行")
                rospy.loginfo(f"📍 无人机ID: {self.drone_id}")
                rospy.loginfo(f"📍 vehicle_ns: {self.vehicle_ns}")
                rospy.loginfo("=" * 60)
                # 暂停航点飞行（关键！）
                self.waypoint_pause_pub.publish(String(data="pause"))
                rospy.loginfo(f"✅ 发布暂停航点指令 -> /drone_{self.drone_id}/waypoint_flight/pause")
                # 发送TRACKING命令到XTDrone
                self.cmd_string = 'TRACKING'
                self.cmd_pub_xt.publish(String(data=self.cmd_string))
                self.cmd_pub.publish(String(data=self.cmd_string))
                self.multi_cmd_pub.publish(String(data=self.cmd_string))
                rospy.loginfo(f"✅ 发布TRACKING命令 -> /xtdrone/{self.vehicle_ns}/cmd")
                # 声明追踪目标
                self.tracking_claim_pub.publish(String(data="tracking"))
                rospy.loginfo(f"✅ 发布追踪声明 -> /drone_{self.drone_id}/tracking_claim")
            
            # 更新融合距离
            try:
                self._update_fused_distance(best_human_target, u, v)
            except Exception as e:
                rospy.logwarn_throttle(5.0, f"距离融合更新失败: {e}")
            
            # 计算速度控制指令（XTDrone算法）
            velocity_cmd = self._compute_velocity_control(u_, v_)
            
            # 设置Twist消息（关键！）
            self.twist.linear.x = velocity_cmd['x']
            self.twist.linear.y = velocity_cmd['y']
            self.twist.linear.z = velocity_cmd['z']
            self.twist.angular.z = velocity_cmd['yaw_rate']
            
            # 立即发布速度控制（关键！）
            self.cmd_vel_pub_xt.publish(self.twist)
            self.cmd_vel_pub.publish(self.twist)
            rospy.logdebug_throttle(3.0, f"✈️ 发布速度指令: vx={self.twist.linear.x:.3f}, vy={self.twist.linear.y:.3f}, vz={self.twist.linear.z:.3f}, yaw_rate={self.twist.angular.z:.3f}")
            
            self.cmd_string = ''
            self.find_cnt += 1  # 关键！增加检测计数
            self.get_time = False  # 关键！重置时间标志
            
            # 提取值用于日志
            x_velocity = velocity_cmd['x']
            y_velocity = velocity_cmd['y']
            z_velocity = velocity_cmd['z']
            yaw_rate = velocity_cmd['yaw_rate']
            
            # 发布记分系统信息
            self._publish_human_detection(best_human_target, u, v, detected_cls_name)
            
            # ✅ 详细的追踪状态信息
            center_offset = math.sqrt(u_*u_ + v_*v_)
            distance_info = f"激光{self.laser_range:.2f}m" if (self.laser_valid and self.laser_range) else "无激光"
            
            # 跟踪状态（更细致的判断）
            if center_offset < 15:
                status = "✅居中"
            elif center_offset < 50:
                status = "🔄跟踪"
            else:
                status = "⚡调整中"
            
            # 检测框位置信息
            bbox_width = best_human_target.xmax - best_human_target.xmin
            bbox_height = best_human_target.ymax - best_human_target.ymin
            bbox_info = f"框({bbox_width:.0f}x{bbox_height:.0f})"
            
            # 偏移量信息
            offset_info = f"偏移(u:{u_:.0f}px, v:{v_:.0f}px)"
            
            # 偏航状态
            yaw_status = ""
            if abs(yaw_rate) > 0.05:
                yaw_dir = "右转" if yaw_rate > 0 else "左转"
                yaw_status = f" | {yaw_dir}({abs(yaw_rate):.2f}rad/s)"
            
            info_msg = (f"🎯 追踪中 | {status}({center_offset:.0f}px) | {bbox_info} | {offset_info} | {distance_info} | "
                       f"高度:{self.height:.2f}m→{self.target_height:.2f}m{yaw_status} | "
                       f"速度(x:{x_velocity:.2f}, y:{y_velocity:.2f}, z:{z_velocity:.2f})")
            
            self.tracking_info_pub.publish(String(data=info_msg))
            rospy.loginfo_throttle(1.0, info_msg)
        
        # 如果没有检测到人体，重置跟踪计时
        if not human_detected:
            if self.tracking_start_time is not None:
                rospy.loginfo("⚠️ 丢失目标，恢复航点飞行")
                self.tracking_start_time = None
                # 恢复航点飞行（关键！）
                self.waypoint_resume_pub.publish(String(data="resume"))
                # 释放追踪声明
                self.tracking_claim_pub.publish(String(data="released"))
            rospy.logdebug_throttle(3.0, f"未检测到人体，当前检测框数: {len(data.bounding_boxes)}, 置信度阈值: {self.detection_confidence}")
    
    def _compute_velocity_control(self, u_, v_):
        """计算速度控制指令 - XTDrone算法（优化版，确保框保持在中心）"""
        # 像素速度（XTDrone原始算法）
        u_velocity = -self.Kp_xy * u_
        v_velocity = -self.Kp_xy * v_
        
        # 相机姿态（俯仰角）
        theta = self.theta if self.cam_pose_received else math.radians(-45.0)
        
        # 深度估计（XTDrone公式）
        sin_theta = math.sin(theta)
        if abs(sin_theta) < 1e-3:
            sin_theta = 1e-3 if sin_theta >= 0.0 else -1e-3
        z_depth = self.height / sin_theta
        
        # 世界坐标速度（XTDrone几何变换）
        fx, fy = self.fx, self.fy
        denom = (v_ * math.cos(theta) + fy * math.sin(theta))
        if abs(denom) > 1e-6:
            x_velocity = v_velocity * z_depth / denom
        else:
            x_velocity = 0.0
        
        if fx != 0:
            y_velocity = (z_depth * u_velocity - u_ * math.cos(theta) * x_velocity) / fx
        else:
            y_velocity = 0.0
        
        # ✅ 高度控制：参考XTDrone，使用Kp_z控制高度（保持在target_height）
        # 这样可以跟踪人的上下移动
        z_velocity = self.Kp_z * (self.target_height - self.height)
        
        # 限幅速度
        x_velocity = np.clip(x_velocity, -self.max_vel, self.max_vel)
        y_velocity = np.clip(y_velocity, -self.max_vel, self.max_vel)
        z_velocity = np.clip(z_velocity, -self.max_vel_z, self.max_vel_z)
        
        # 偏航控制（增强响应性）
        yaw_rate = self._compute_yaw_control(u_)
        
        return {
            'x': x_velocity,
            'y': y_velocity,
            'z': z_velocity,
            'yaw_rate': yaw_rate
        }
    
    def _compute_yaw_control(self, u_offset):
        """计算偏航控制 - 优化版（保持目标在图像中心）"""
        # ✅ 减小死区，提高灵敏度（从30px降到15px）
        yaw_deadzone = 15
        if abs(u_offset) < yaw_deadzone:
            return 0.0
        
        # ✅ 增加增益，提高响应速度（从0.003到0.005）
        Kp_yaw = 0.005
        max_yaw_rate = 1.0  # 增加最大角速度（从0.8到1.0）
        
        # 比例控制
        yaw_rate = Kp_yaw * u_offset
        yaw_rate = np.clip(yaw_rate, -max_yaw_rate, max_yaw_rate)
        
        # 平滑处理（减小平滑系数，提高响应）
        if hasattr(self, '_last_yaw_rate'):
            max_yaw_change = 0.5  # 从0.3增加到0.5，允许更快的转向
            yaw_rate = np.clip(yaw_rate,
                              self._last_yaw_rate - max_yaw_change,
                              self._last_yaw_rate + max_yaw_change)
        self._last_yaw_rate = yaw_rate
        
        return yaw_rate
    
    def _update_fused_distance(self, target, center_u, center_v):
        """融合激光/视觉距离估计"""
        # 激光优先
        if self.laser_valid and self.laser_range and self.laser_range > 0.05:
            if self.fused_distance is None:
                self.fused_distance = float(self.laser_range)
            else:
                a = self.dist_alpha_laser
                self.fused_distance = a * float(self.laser_range) + (1.0 - a) * self.fused_distance
            return
        
        # 视觉估距
        try:
            bbox_h_ratio = max(1e-3, (target.ymax - target.ymin) / float(self.image_height))
            vision_d = self.vision_distance_k / bbox_h_ratio
            if self.fused_distance is None:
                self.fused_distance = float(vision_d)
            else:
                a = self.dist_alpha_vision
                self.fused_distance = a * float(vision_d) + (1.0 - a) * self.fused_distance
        except Exception:
            pass
    
    def _pose_callback(self, data):
        """位置回调 - 保存位置和姿态"""
        self.height = data.pose.position.z
        self.drone_position = data.pose.position
        
        # 获取当前偏航角（关键！用于坐标转换）
        q = Quaternion(data.pose.orientation.w, data.pose.orientation.x,
                      data.pose.orientation.y, data.pose.orientation.z)
        self._current_yaw = q.yaw_pitch_roll[0]  # yaw角
        
        # 强制设置目标高度
        self.target_height = self.fixed_height
        self.target_set = True
    
    def _cam_pose_callback(self, data):
        """相机姿态回调 - 基于XTDrone算法"""
        q = Quaternion(data.pose.orientation.w, data.pose.orientation.x, 
                      data.pose.orientation.y, data.pose.orientation.z)
        self.theta = q.yaw_pitch_roll[1]  # 俯仰角
        self.cam_pose_received = True
    
    def _enable_callback(self, msg):
        """跟踪使能回调"""
        if msg.data and not self.tracking_active:
            self.tracking_active = True
            rospy.loginfo("🎯 人体跟踪已启用")
        elif not msg.data and self.tracking_active:
            self.tracking_active = False
            rospy.loginfo("⏸️ 人体跟踪已禁用")
    
    def _laser_scan_callback(self, msg):
        """激光雷达回调"""
        try:
            ranges = np.array(msg.ranges)
            valid_ranges = ranges[np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)]
            
            if len(valid_ranges) > 0:
                front_angle_range = int(len(ranges) * 0.25)
                front_start = len(ranges) // 2 - front_angle_range // 2
                front_end = len(ranges) // 2 + front_angle_range // 2
                
                front_ranges = ranges[front_start:front_end]
                front_valid = front_ranges[np.isfinite(front_ranges) & (front_ranges > msg.range_min)]
                
                if len(front_valid) > 0:
                    self.laser_range = float(np.min(front_valid))
                    self.laser_valid = True
                else:
                    self.laser_range = None
                    self.laser_valid = False
            else:
                self.laser_range = None
                self.laser_valid = False
        except Exception as e:
            rospy.logwarn_throttle(10, f"激光雷达数据处理错误: {e}")
            self.laser_range = None
            self.laser_valid = False
    
    def _laser_range_callback(self, msg):
        """激光测距仪回调"""
        try:
            if msg.range > msg.min_range and msg.range < msg.max_range:
                self.laser_range = msg.range
                self.laser_valid = True
            else:
                self.laser_range = None
                self.laser_valid = False
        except Exception as e:
            rospy.logwarn_throttle(10, f"激光测距仪数据处理错误: {e}")
            self.laser_range = None
            self.laser_valid = False
    
    def _publish_human_detection(self, target, center_u, center_v, detected_cls_name=None):
        """发布人体检测信息到记分系统"""
        if not self.drone_position:
            return
        
        try:
            horizontal_offset = (center_u - self.u_center) * 0.01
            
            if self.fused_distance is not None:
                distance = float(np.clip(self.fused_distance, 0.5, 30.0))
            elif self.laser_valid and self.laser_range:
                distance = float(self.laser_range)
            else:
                bbox_h_ratio = max(0.05, (target.ymax - target.ymin) / float(self.image_height))
                distance = max(2.0, min(12.0, self.target_size_ratio / bbox_h_ratio * 3.0))
            
            human_x = self.drone_position.x + distance * 0.8
            human_y = self.drone_position.y + horizontal_offset
            
            actor_msg = ActorInfo()
            if detected_cls_name and detected_cls_name.lower() in self.color_classes:
                actor_msg.cls = detected_cls_name.lower()
            else:
                actor_msg.cls = 'blue'
            actor_msg.x = human_x
            actor_msg.y = human_y
            
            # ✅ 兼容性处理：有些版本的ActorInfo没有z属性
            try:
                actor_msg.z = 0.0  # 地面高度
            except AttributeError:
                pass  # 如果没有z属性就跳过
            
            try:
                actor_msg.distance = float(self.laser_range) if self.laser_valid and self.laser_range else distance
            except AttributeError:
                pass  # 如果没有distance属性就跳过
            
            # 发布到对应颜色通道
            if actor_msg.cls == 'blue':
                self.actor_blue_pub.publish(actor_msg)
            elif actor_msg.cls == 'green':
                self.actor_green_pub.publish(actor_msg)
            elif actor_msg.cls == 'white':
                self.actor_white_pub.publish(actor_msg)
            elif actor_msg.cls == 'brown':
                self.actor_brown_pub.publish(actor_msg)
            elif actor_msg.cls in ['red', 'red1']:
                self.actor_red1_pub.publish(actor_msg)
            elif actor_msg.cls == 'red2':
                self.actor_red2_pub.publish(actor_msg)
            else:
                self.actor_blue_pub.publish(actor_msg)
            
            laser_status = f"激光{self.laser_range:.2f}m" if (self.laser_valid and self.laser_range) else "视觉估计"
            rospy.loginfo_throttle(5.0, f"📡 发布人体检测: 位置({human_x:.2f}, {human_y:.2f}), 距离{distance:.2f}m ({laser_status})")
            
        except Exception as e:
            rospy.logwarn(f"发布人体检测信息失败: {e}")
    
    def _publish_gimbal(self, pitch_deg=-45.0, yaw_deg=0.0, roll_deg=0.0):
        """发布云台控制指令，保持-45°俯仰角"""
        from mavros_msgs.msg import MountControl
        msg = MountControl()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.mode = 2  # MAV_MOUNT_MODE_MAVLINK_TARGETING
        msg.pitch = pitch_deg
        msg.yaw = yaw_deg
        msg.roll = roll_deg
        self.mount_pub.publish(msg)
    
    def tracking_loop(self):
        """主跟踪循环 - 优化版（参考XTDrone yolo_human_tracking.py）"""
        rate_hz = rospy.get_param('~control_rate_hz', 60)  # ✅ 提高到60Hz（参考XTDrone）
        rate = rospy.Rate(rate_hz)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("🚀 人体跟踪循环已启动（XTDrone算法）")
        rospy.loginfo(f"📡 发布频率: {rate_hz}Hz")
        rospy.loginfo(f"📍 控制话题: /xtdrone/{self.vehicle_ns}/cmd_vel_flu")
        rospy.loginfo(f"📍 命令话题: /xtdrone/{self.vehicle_ns}/cmd")
        rospy.loginfo(f"⚙️  控制增益: Kp_xy={self.Kp_xy}, Kp_z={self.Kp_z}")
        rospy.loginfo("=" * 60)
        
        while not rospy.is_shutdown():
            try:
                # 云台控制始终保持-45°（XTDrone标准配置）
                self._publish_gimbal(pitch_deg=-45.0)
                
                # 检查跟踪状态
                if self.tracking_active:
                    # ✅ 持续发布速度指令到XTDrone（高频率）
                    self.cmd_vel_pub_xt.publish(self.twist)
                    self.cmd_vel_pub.publish(self.twist)
                    
                    # ✅ 持续发布命令（如果有）
                    if self.cmd_string:
                        self.cmd_pub_xt.publish(String(data=self.cmd_string))
                        self.cmd_pub.publish(String(data=self.cmd_string))
                    
                    # 检查是否丢失目标（XTDrone逻辑）
                    if self.find_cnt == self.find_cnt_last:
                        if not self.get_time:
                            self.not_find_time = rospy.get_time()
                            self.get_time = True
                        
                        # ✅ 2秒后悬停并恢复航点
                        if rospy.get_time() - self.not_find_time > 2.0:
                            rospy.loginfo("⏰ 目标丢失超过2秒，悬停并恢复航点飞行")
                            # 停止所有运动
                            zero_twist = Twist()
                            zero_twist.linear.x = 0.0
                            zero_twist.linear.y = 0.0
                            zero_twist.linear.z = 0.0
                            zero_twist.angular.z = 0.0
                            self.cmd_vel_pub_xt.publish(zero_twist)
                            self.cmd_vel_pub.publish(zero_twist)
                            self.twist = zero_twist  # 更新self.twist
                            
                            # 发送HOVER命令
                            self.cmd_string = 'HOVER'
                            self.cmd_pub_xt.publish(String(data=self.cmd_string))
                            self.cmd_pub.publish(String(data=self.cmd_string))
                            self.multi_cmd_pub.publish(String(data=self.cmd_string))
                            
                            # 恢复航点飞行
                            self.waypoint_resume_pub.publish(String(data="resume"))
                            self.tracking_claim_pub.publish(String(data="released"))
                            self.tracking_status_pub.publish(Bool(data=False))
                            
                            # 重置跟踪状态
                            self.tracking_start_time = None
                            self.get_time = False
                    else:
                        # 有新检测，继续跟踪
                        self.get_time = False
                        self.tracking_status_pub.publish(Bool(data=True))
                        rospy.logdebug_throttle(2.0, f"✈️ 追踪中: vx={self.twist.linear.x:.3f}, vy={self.twist.linear.y:.3f}, vz={self.twist.linear.z:.3f}, yaw={self.twist.angular.z:.3f}")
                else:
                    # 跟踪未启用，发布停止状态
                    self.tracking_status_pub.publish(Bool(data=False))
                
                # ✅ 更新上一次计数（XTDrone关键逻辑）
                self.find_cnt_last = self.find_cnt
                
            except Exception as e:
                rospy.logerr(f"❌ 跟踪循环错误: {e}")
                import traceback
                rospy.logerr(traceback.format_exc())
            
            rate.sleep()
    
    def run(self):
        """运行跟踪器"""
        try:
            rospy.loginfo("🚀 人体跟踪器运行中...")
            self.tracking_loop()  # 直接运行主循环
        except rospy.ROSInterruptException:
            rospy.loginfo("人体跟踪节点关闭")

if __name__ == '__main__':
    try:
        tracker = HumanTracker()
        tracker.run()
    except Exception as e:
        rospy.logerr(f"人体跟踪节点启动失败: {e}")
