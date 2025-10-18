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
import torch
from ultralytics import YOLO

import rospy
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Header
from cv_bridge import CvBridge

from mavros_msgs.msg import PositionTarget, State, MountControl
from mavros_msgs.srv import CommandBool, SetMode, MountConfigure
from pyquaternion import Quaternion
from yolov11_ros_msgs.msg import BoundingBox, BoundingBoxes

# ByteTrack追踪器
from collections import deque


class ByteTracker:
    """简化的ByteTrack追踪器"""
    def __init__(self, track_thresh=0.5, track_buffer=30, match_thresh=0.8):
        self.track_thresh = track_thresh
        self.track_buffer = track_buffer
        self.match_thresh = match_thresh
        
        self.tracked_tracks = []  # 正在追踪的目标
        self.lost_tracks = []     # 丢失的目标
        self.removed_tracks = []  # 移除的目标
        self.frame_id = 0
        self.next_id = 1
        
    def update(self, detections):
        """更新追踪器"""
        self.frame_id += 1
        
        # 为检测结果分配ID
        detections_with_id = []
        for det in detections:
            det['track_id'] = self._assign_id(det)
            detections_with_id.append(det)
        
        # 更新追踪状态
        self._update_tracks(detections_with_id)
        
        # 返回当前活跃的追踪目标
        active_tracks = [track for track in self.tracked_tracks if track['active']]
        return active_tracks
    
    def _assign_id(self, detection):
        """为检测结果分配ID"""
        # 简化的ID分配：基于IoU匹配
        best_match_id = None
        best_iou = 0
        
        for track in self.tracked_tracks:
            if track['active']:
                iou = self._calculate_iou(detection['bbox'], track['bbox'])
                if iou > self.match_thresh and iou > best_iou:
                    best_iou = iou
                    best_match_id = track['id']
        
        if best_match_id is not None:
            return best_match_id
        else:
            # 分配新ID
            new_id = self.next_id
            self.next_id += 1
            return new_id
    
    def _update_tracks(self, detections):
        """更新追踪状态"""
        # 为每个检测结果找到最佳匹配的追踪目标
        matched_tracks = set()
        
        for det in detections:
            best_track = None
            best_iou = 0
            
            for track in self.tracked_tracks:
                if track['active'] and track['id'] not in matched_tracks:
                    iou = self._calculate_iou(det['bbox'], track['bbox'])
                    if iou > self.match_thresh and iou > best_iou:
                        best_iou = iou
                        best_track = track
            
            if best_track:
                # 更新追踪目标
                best_track['bbox'] = det['bbox']
                best_track['center'] = det['center']
                best_track['confidence'] = det['confidence']
                best_track['class'] = det['class']
                best_track['last_seen'] = self.frame_id
                matched_tracks.add(best_track['id'])
            else:
                # 创建新的追踪目标
                new_track = {
                    'id': det['track_id'],
                    'bbox': det['bbox'],
                    'center': det['center'],
                    'confidence': det['confidence'],
                    'class': det['class'],
                    'first_seen': self.frame_id,
                    'last_seen': self.frame_id,
                    'active': True
                }
                self.tracked_tracks.append(new_track)
        
        # 标记未匹配的追踪目标为丢失
        for track in self.tracked_tracks:
            if track['active'] and track['id'] not in matched_tracks:
                track['active'] = False
                track['lost_frames'] = track.get('lost_frames', 0) + 1
        
        # 移除长时间丢失的追踪目标
        self.tracked_tracks = [track for track in self.tracked_tracks 
                              if track.get('lost_frames', 0) < self.track_buffer]
    
    def _calculate_iou(self, bbox1, bbox2):
        """计算两个边界框的IoU"""
        x1_1, y1_1, x2_1, y2_1 = bbox1
        x1_2, y1_2, x2_2, y2_2 = bbox2
        
        # 计算交集
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)
        
        if x2_i <= x1_i or y2_i <= y1_i:
            return 0.0
        
        intersection = (x2_i - x1_i) * (y2_i - y1_i)
        
        # 计算并集
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0.0


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
        
        # 删除云台控制参数
        # self.gimbal_yaw = 0.0  # 云台当前偏航角
        # self.target_gimbal_yaw = 0.0  # 云台目标偏航角

        # 传感器
        self.bridge = CvBridge()
        self.rgb_image = None
        self.cv_bridge_working = True  # 标记cv_bridge是否工作正常
        
        # YOLO检测相关
        self.yolo_model = None
        self.detection_enabled = True
        self.det_conf_threshold = rospy.get_param("~det_conf_threshold", 0.35)
        self.street_classes = rospy.get_param("~street_classes", ['road', 'street', 'highway'])
        self.intersection_classes = rospy.get_param("~intersection_classes", ['intersection', 'crossroad', 'junction'])
        self.human_classes = rospy.get_param("~human_classes", ['human', 'person', 'pedestrian'])
        
        # 行人跟踪相关
        self.human_tracking_enabled = rospy.get_param("~human_tracking_enabled", True)
        self.human_tracking_speed = rospy.get_param("~human_tracking_speed", 2.0)
        self.human_center_threshold = rospy.get_param("~human_center_threshold", 0.15)
        self.current_human_target = None  # 当前跟踪的行人目标
        self.human_tracking_timeout = 10.0  # 行人跟踪超时时间（秒）- 增加超时时间
        self.last_human_detection_time = 0.0
        self.force_detection_mode = True  # 强制检测模式，确保每次都能检测
        
        # 街道跟踪相关
        self.street_center_threshold = rospy.get_param("~street_center_threshold", 0.1)
        self.street_tracking_gain = rospy.get_param("~street_tracking_gain", 1.5)
        self.forward_velocity = rospy.get_param("~forward_velocity", 5.0)
        
        # 路口处理相关
        self.intersection_stop_distance = rospy.get_param("~intersection_stop_distance", 10.0)
        self.turn_angle = rospy.get_param("~turn_angle", 90.0)
        
        # 调试图像发布
        self.show_debug_image = rospy.get_param("~show_debug_image", True)
        self.debug_image_pub = rospy.Publisher("/street_cruise_yolo/debug_image", Image, queue_size=1)
        self.detection_image_pub = rospy.Publisher("/yolov11/detection_image", Image, queue_size=1)
        
        # 图像质量优化参数
        self.image_quality_mode = rospy.get_param("~image_quality_mode", "high")  # high, medium, low
        self.enable_image_upscaling = rospy.get_param("~enable_image_upscaling", True)
        self.target_image_width = rospy.get_param("~target_image_width", 1280)
        self.target_image_height = rospy.get_param("~target_image_height", 720)
        
        # 初始化YOLO模型
        self._init_yolo_model()
        
        # 初始化ByteTracker追踪器
        self.tracker = ByteTracker(track_thresh=0.3, track_buffer=30, match_thresh=0.6)
        self.current_tracked_human = None  # 当前追踪的行人目标
        self.tracking_start_time = 0.0    # 开始追踪的时间
        
        # 跟随飞行参数
        self.follow_mode = "intelligent"  # 跟随模式：simple, intelligent
        self.target_distance = 3.0        # 目标跟随距离（米）
        self.follow_speed_factor = 1.0    # 跟随速度系数
        self.prediction_enabled = True    # 启用运动预测
        
        # 运动预测和历史记录
        self.human_position_history = {}  # 存储每个ID的位置历史
        self.max_history_length = 10      # 最大历史记录长度
        
        # 测试cv_bridge兼容性
        self._test_cv_bridge()
        

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
        # 优化图像订阅：增加缓冲区大小以提高画质
        self.rgb_sub = rospy.Subscriber(self.rgb_topic, Image, self._rgb_cb, queue_size=1, buff_size=104857600)  # 100MB缓冲区
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
        
        # 高度限制参数
        self.max_altitude = 5.0  # 最大飞行高度限制为5米
        self.min_altitude = 1.0   # 最小飞行高度限制为1米
        self.emergency_landing = False  # 紧急降落标志
        self.height_control_active = False  # 高度控制激活标志
        
        # 起飞参数
        self.takeoff_altitude = rospy.get_param("~takeoff_altitude", 3.0)  # 起飞高度3米
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
        rospy.loginfo(f"YOLO检测: {'启用' if self.detection_enabled else '禁用'}")
        rospy.loginfo(f"cv_bridge状态: {'正常' if self.cv_bridge_working else '异常（LIBFFI问题）'}")
        if self.human_tracking_enabled and self.cv_bridge_working:
            rospy.loginfo("🚨 行人跟踪: 启用（最高优先级）")
            rospy.loginfo(f"行人跟踪速度: {self.human_tracking_speed} m/s")
            rospy.loginfo(f"行人跟踪超时: {self.human_tracking_timeout} 秒")
        elif self.human_tracking_enabled and not self.cv_bridge_working:
            rospy.loginfo("⚠️ 行人跟踪: 禁用（cv_bridge问题）")
        else:
            rospy.loginfo("行人跟踪: 禁用")
        

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
        
        # 如果没有成功加载航点，使用默认航点（统一高度3米）
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
            rospy.logwarn(f"使用默认航点数据，共 {len(self.waypoints)} 个航点（统一高度3米）")

    def _init_yolo_model(self):
        """初始化YOLO模型"""
        if not self.detection_enabled:
            rospy.loginfo("YOLO检测已禁用")
            return
            
        try:
            # 获取权重文件路径
            weight_path = rospy.get_param("~yolo_weight_path", "/home/cty/catkin_ws/src/yolov11_ros/weights/best.pt")
            
            # 检查权重文件是否存在
            if not os.path.exists(weight_path):
                rospy.logwarn(f"YOLO权重文件不存在: {weight_path}")
                rospy.logwarn("将禁用YOLO检测功能")
                self.detection_enabled = False
                return
            
            rospy.loginfo(f"加载YOLO模型: {weight_path}")
            
            # 初始化YOLO模型
            self.yolo_model = YOLO(weight_path)
            self.yolo_model.fuse()
            
            # 设置设备
            if rospy.get_param('/use_gpu', False):
                self.device = 'cuda'
            else:
                self.device = 'cpu'
            
            rospy.loginfo(f"YOLO模型初始化成功，使用设备: {self.device}")
            
        except ImportError as e:
            rospy.logerr(f"YOLO依赖包未安装: {e}")
            rospy.logerr("请安装ultralytics: pip install ultralytics")
            self.detection_enabled = False
        except Exception as e:
            rospy.logerr(f"YOLO模型初始化失败: {e}")
            self.detection_enabled = False

    def _test_cv_bridge(self):
        """测试cv_bridge兼容性"""
        try:
            # 创建一个测试图像
            test_image = np.zeros((100, 100, 3), dtype=np.uint8)
            
            # 测试cv2_to_imgmsg
            test_msg = self.bridge.cv2_to_imgmsg(test_image, "bgr8")
            
            # 测试imgmsg_to_cv2
            test_result = self.bridge.imgmsg_to_cv2(test_msg, desired_encoding="bgr8")
            
            rospy.loginfo("cv_bridge兼容性测试通过")
            self.cv_bridge_working = True
            
        except Exception as e:
            if "LIBFFI_BASE" in str(e) or "ffi_type_pointer" in str(e):
                rospy.logwarn("检测到cv_bridge LIBFFI兼容性问题，将禁用图像处理功能")
                rospy.logwarn("这不会影响航点飞行功能，但行人跟踪功能将被禁用")
                self.cv_bridge_working = False
            else:
                rospy.logwarn(f"cv_bridge测试失败: {e}")
                self.cv_bridge_working = False

    def _detect_objects(self, image):
        """使用YOLO检测图像中的对象"""
        if not self.detection_enabled or self.yolo_model is None:
            return []
        
        try:
            # 运行YOLO检测
            results = self.yolo_model(image, conf=self.det_conf_threshold, verbose=False)
            
            detections = []
            if results and len(results) > 0:
                for result in results[0].boxes:
                    if result.conf.item() >= self.det_conf_threshold:
                        detection = {
                            'class': results[0].names[result.cls.item()],
                            'confidence': result.conf.item(),
                            'bbox': result.xyxy[0].cpu().numpy(),  # [x1, y1, x2, y2]
                            'center': [(result.xyxy[0][0] + result.xyxy[0][2]) / 2, 
                                      (result.xyxy[0][1] + result.xyxy[0][3]) / 2]
                        }
                        detections.append(detection)
            
            return detections
            
        except Exception as e:
            rospy.logwarn(f"YOLO检测失败: {e}")
            return []

    def _detect_and_track_human(self):
        """使用YOLO检测 + ByteTracker追踪进行行人检测和跟踪"""
        if not self.human_tracking_enabled:
            return None
        if self.yolo_model is None:
            return None
        if self.rgb_image is None:
            return None
        
        try:
            # YOLO检测：专注于检测
            rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB)
            results = self.yolo_model(rgb_image, show=False, conf=0.1)
            
            if not results or len(results) == 0:
                return None
            
            # 提取所有行人检测结果
            human_detections = []
            for result in results[0].boxes:
                class_name = results[0].names[result.cls.item()]
                confidence = result.conf.item()
                
                if class_name in self.human_classes and confidence >= 0.1:
                    detection = {
                        'class': class_name,
                        'confidence': confidence,
                        'bbox': result.xyxy[0].cpu().numpy(),  # [x1, y1, x2, y2]
                        'center': [(result.xyxy[0][0] + result.xyxy[0][2]) / 2, 
                                  (result.xyxy[0][1] + result.xyxy[0][3]) / 2]
                    }
                    human_detections.append(detection)
            
            # ByteTracker追踪：专注于关联和ID维护
            tracked_targets = self.tracker.update(human_detections)
            
            if not tracked_targets:
                return None
            
            # 选择最佳追踪目标（优先选择置信度高的）
            best_target = max(tracked_targets, key=lambda x: x['confidence'])
            
            # 记录位置历史用于运动预测
            track_id = best_target['id']
            current_time = time.time()
            
            if track_id not in self.human_position_history:
                self.human_position_history[track_id] = []
            
            # 添加当前位置到历史记录
            position_record = {
                'time': current_time,
                'center': best_target['center'],
                'bbox': best_target['bbox']
            }
            self.human_position_history[track_id].append(position_record)
            
            # 保持历史记录长度
            if len(self.human_position_history[track_id]) > self.max_history_length:
                self.human_position_history[track_id].pop(0)
            
            # 检查是否开始新的追踪
            if (self.current_tracked_human is None or 
                self.current_tracked_human['id'] != best_target['id']):
                self.current_tracked_human = best_target
                self.tracking_start_time = time.time()
                rospy.loginfo(f"🚨 开始追踪行人 ID:{best_target['id']}, 置信度: {best_target['confidence']:.3f}")
            
            # 调试信息：显示检测状态
            rospy.loginfo_throttle(1.0, f"检测状态: 找到{len(tracked_targets)}个追踪目标，选择ID:{best_target['id']}")
            
            return best_target
            
        except Exception as e:
            rospy.logwarn(f"行人检测和追踪失败: {e}")
            return None

    def _find_human_target(self, detections):
        """从检测结果中找到最佳的行人跟踪目标（保留兼容性）"""
        if not self.human_tracking_enabled:
            return None
        
        human_detections = [d for d in detections if d['class'] in self.human_classes]
        
        if not human_detections:
            return None
        
        # 选择置信度最高的行人
        best_human = max(human_detections, key=lambda x: x['confidence'])
        
        # 检查是否在图像中心附近
        image_center_x = self.rgb_image.shape[1] / 2
        image_center_y = self.rgb_image.shape[0] / 2
        
        center_x, center_y = best_human['center']
        distance_from_center = math.sqrt((center_x - image_center_x)**2 + (center_y - image_center_y)**2)
        max_distance = min(self.rgb_image.shape[1], self.rgb_image.shape[0]) * self.human_center_threshold
        
        if distance_from_center <= max_distance:
            return best_human
        
        return None

    def _human_tracking_control(self, human_target):
        """最简单的行人追踪：让人保持在图像中心"""
        if not human_target or not self.rgb_image:
            return 0.0, 0.0, 0.0, 0.0, False
        
        # 获取图像尺寸和人体位置
        image_height, image_width = self.rgb_image.shape[:2]
        center_x, center_y = human_target['center']
        bbox = human_target['bbox']
        
        # 计算图像中心
        image_center_x = image_width / 2
        image_center_y = image_height / 2
        
        # 计算偏移量（像素）
        offset_x = center_x - image_center_x
        offset_y = center_y - image_center_y
        
        # 计算人体大小
        person_width = bbox[2] - bbox[0]
        person_height = bbox[3] - bbox[1]
        person_area = person_width * person_height
        image_area = image_width * image_height
        size_ratio = person_area / image_area
        
        # 简单的控制逻辑：
        # 1. 水平控制：让人水平居中
        lateral_velocity = -offset_x * 0.004  # 左右移动
        
        # 2. 前后控制：根据人体大小调整距离
        target_size_ratio = 0.08  # 目标：人体占图像8%
        if size_ratio > target_size_ratio * 1.3:  # 太近
            forward_velocity = -0.6
        elif size_ratio < target_size_ratio * 0.7:  # 太远
            forward_velocity = 0.6
        else:  # 距离合适
            forward_velocity = 0.0
        
        # 3. 垂直控制：保持高度稳定（禁用垂直跟随，避免下降）
        vertical_velocity = 0.0  # 不根据人的垂直位置调整高度
        
        # 4. 偏航控制：旋转让人保持在中心
        yaw_rate = -offset_x * 0.002
        
        # 严格的高度限制
        if self.current_position:
            current_height = self.current_position.z
            if current_height >= self.max_altitude - 0.2:  # 接近5米时
                vertical_velocity = min(vertical_velocity, -0.2)  # 轻微下降
                rospy.logwarn_throttle(3.0, f"追踪时高度限制: 当前{current_height:.2f}m，轻微下降")
            elif current_height <= self.min_altitude + 0.2:  # 接近1米时
                vertical_velocity = max(vertical_velocity, 0.2)   # 轻微上升
                rospy.logwarn_throttle(3.0, f"追踪时高度限制: 当前{current_height:.2f}m，轻微上升")
            else:
                # 在安全范围内，保持高度稳定
                vertical_velocity = 0.0
        
        # 速度限制
        lateral_velocity = np.clip(lateral_velocity, -1.0, 1.0)
        forward_velocity = np.clip(forward_velocity, -1.0, 1.0)
        yaw_rate = np.clip(yaw_rate, -0.3, 0.3)
        
        # 简化的调试信息
        rospy.loginfo_throttle(3.0, 
            f"🎯 中心化追踪: 人在({center_x:.0f},{center_y:.0f}), 中心({image_center_x:.0f},{image_center_y:.0f}), "
            f"偏移({offset_x:.0f},{offset_y:.0f}), 大小{size_ratio:.3f}")
        
        return forward_velocity, lateral_velocity, vertical_velocity, yaw_rate, True

    def _predict_human_movement(self, track_id):
        """预测行人运动方向"""
        if track_id not in self.human_position_history:
            return None, None
        
        history = self.human_position_history[track_id]
        if len(history) < 3:  # 需要至少3个历史点
            return None, None
        
        # 计算运动速度
        recent_positions = history[-3:]  # 最近3个位置
        velocities = []
        
        for i in range(1, len(recent_positions)):
            dt = recent_positions[i]['time'] - recent_positions[i-1]['time']
            if dt > 0:
                dx = recent_positions[i]['center'][0] - recent_positions[i-1]['center'][0]
                dy = recent_positions[i]['center'][1] - recent_positions[i-1]['center'][1]
                vx = dx / dt
                vy = dy / dt
                velocities.append((vx, vy))
        
        if not velocities:
            return None, None
        
        # 计算平均速度
        avg_vx = sum(v[0] for v in velocities) / len(velocities)
        avg_vy = sum(v[1] for v in velocities) / len(velocities)
        
        # 预测下一个位置（预测0.5秒后）
        current_pos = history[-1]['center']
        predicted_x = current_pos[0] + avg_vx * 0.5
        predicted_y = current_pos[1] + avg_vy * 0.5
        
        return (predicted_x, predicted_y), (avg_vx, avg_vy)

    def _monitor_height(self):
        """监控高度限制"""
        if self.current_position:
            current_height = self.current_position.z
            if current_height > self.max_altitude + 2.0:  # 超过7米，紧急降落
                rospy.logerr(f"🚨 高度严重超限！当前{current_height:.2f}m > 紧急限制{self.max_altitude + 2.0}m")
                self.emergency_landing = True
                return False
            elif current_height > self.max_altitude + 0.2:  # 超过5.2米，开始强制控制
                if not self.height_control_active:
                    rospy.logerr(f"🚨 高度超限！当前{current_height:.2f}m > 限制{self.max_altitude}m，启动强制高度控制")
                    self.height_control_active = True
                # 持续强制下降
                self._force_height_control()
                return False
            elif current_height <= self.max_altitude and self.height_control_active:
                # 高度恢复正常，停止强制控制
                rospy.loginfo(f"✅ 高度恢复正常: {current_height:.2f}m，停止强制控制")
                self.height_control_active = False
                return True
            elif current_height < self.min_altitude:
                rospy.logerr(f"🚨 高度过低！当前{current_height:.2f}m < 限制{self.min_altitude}m")
                return False
            else:
                # 正常范围内，每10秒报告一次
                rospy.loginfo_throttle(10.0, f"高度正常: {current_height:.2f}m (范围: {self.min_altitude}-{self.max_altitude}m)")
                return True
        return False

    def _force_height_control(self):
        """强制高度控制：立即下降到安全高度"""
        rospy.logwarn("执行强制高度控制：立即下降到5米")
        
        if self.current_position:
            # 使用位置控制强制下降到3米
            target = PositionTarget()
            target.coordinate_frame = self.position_frame
            target.type_mask = (PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
                               PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                               PositionTarget.IGNORE_YAW_RATE)
            target.position.x = self.current_position.x
            target.position.y = self.current_position.y
            target.position.z = self.max_altitude  # 强制设置为3米
            target.yaw = self.current_yaw
            self.setpoint_pub.publish(target)
            
            rospy.logwarn(f"强制位置控制: 当前高度{self.current_position.z:.2f}m -> 目标高度{self.max_altitude}m")

    def _check_detection_status(self):
        """检查检测状态（不依赖cv_bridge）"""
        if self.rgb_image is None:
            rospy.logwarn_throttle(10.0, "未收到图像数据")
            return False
        
        if self.yolo_model is None:
            rospy.logwarn_throttle(10.0, "YOLO模型未初始化")
            return False
        
        if not self.human_tracking_enabled:
            rospy.logwarn_throttle(10.0, "行人追踪已禁用")
            return False
        
        return True

    def _enhance_image_quality(self, image):
        """增强图像质量"""
        if image is None:
            return None
        
        try:
            enhanced_image = image.copy()
            
            # 根据质量模式进行不同的处理
            if self.image_quality_mode == "high":
                # 高质量模式：图像增强
                # 1. 去噪
                enhanced_image = cv2.bilateralFilter(enhanced_image, 9, 75, 75)
                
                # 2. 锐化
                kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
                enhanced_image = cv2.filter2D(enhanced_image, -1, kernel)
                
                # 3. 对比度增强
                enhanced_image = cv2.convertScaleAbs(enhanced_image, alpha=1.2, beta=10)
                
            elif self.image_quality_mode == "medium":
                # 中等质量模式：轻微增强
                enhanced_image = cv2.bilateralFilter(enhanced_image, 5, 50, 50)
                enhanced_image = cv2.convertScaleAbs(enhanced_image, alpha=1.1, beta=5)
            
            # 图像上采样（如果需要）
            if self.enable_image_upscaling:
                current_height, current_width = enhanced_image.shape[:2]
                if current_width < self.target_image_width or current_height < self.target_image_height:
                    # 使用双三次插值进行上采样
                    enhanced_image = cv2.resize(enhanced_image, 
                                              (self.target_image_width, self.target_image_height), 
                                              interpolation=cv2.INTER_CUBIC)
            
            return enhanced_image
            
        except Exception as e:
            rospy.logwarn(f"图像质量增强失败: {e}")
            return image

    def _publish_debug_image(self):
        """发布调试图像，显示检测结果和跟踪状态（参考yolo_v11.py的实现，增强画质）"""
        if self.rgb_image is None:
            return
        
        try:
            # 使用增强后的图像
            debug_image = self._enhance_image_quality(self.rgb_image)
            if debug_image is None:
                debug_image = self.rgb_image.copy()
            
            # 如果启用了检测，绘制检测结果（完全按照yolo_v11.py的方式）
            if self.detection_enabled and self.yolo_model is not None:
                try:
                    # 完全按照yolo_v11.py的方式
                    rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB)
                    results = self.yolo_model(rgb_image, show=False, conf=0.3)
                    
                    if results and len(results) > 0:
                        # 使用YOLO自带的绘制功能（完全按照yolo_v11.py）
                        annotated_frame = results[0].plot()
                        
                        # 将注释后的图像转换回BGR格式
                        debug_image = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)
                        
                        # 添加FPS信息（完全按照yolo_v11.py）
                        fps = 1000.0 / results[0].speed['inference']
                        cv2.putText(debug_image, f'FPS: {int(fps)}', (20, 50), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
                        
                except Exception as e:
                    rospy.logwarn(f"YOLO检测绘制失败: {e}")
                
                # 高亮显示当前追踪的行人
                if self.current_tracked_human:
                    bbox = self.current_tracked_human['bbox']
                    track_id = self.current_tracked_human['id']
                    confidence = self.current_tracked_human['confidence']
                    
                    cv2.rectangle(debug_image, 
                                (int(bbox[0]), int(bbox[1])), 
                                (int(bbox[2]), int(bbox[3])), 
                                (0, 0, 255), 3)  # 红色高亮
                    
                    cv2.putText(debug_image, f"TRACKING ID:{track_id}", 
                              (int(bbox[0]), int(bbox[1]) - 30),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    cv2.putText(debug_image, f"Conf: {confidence:.3f}", 
                              (int(bbox[0]), int(bbox[1]) - 5),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # 添加状态信息（突出行人追踪优先级）
            if self.current_tracked_human:
                status_text = f"PRIORITY: Tracking ID:{self.current_tracked_human['id']}"
                color = (0, 0, 255)  # 红色突出显示
            else:
                status_text = "Mode: Waypoint Flight"
                color = (255, 255, 255)  # 白色
            
            cv2.putText(debug_image, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            if self.current_waypoint_index < len(self.waypoints):
                wp_text = f"Waypoint: {self.current_waypoint_index + 1}/{len(self.waypoints)}"
                cv2.putText(debug_image, wp_text, (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 发布调试图像到ROS话题（修复cv_bridge兼容性问题）
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.header.frame_id = "camera_color_frame"
                self.debug_image_pub.publish(debug_msg)
            except Exception as e:
                rospy.logwarn(f"发布调试图像失败: {e}")
            
            # 发布检测图像到yolov11话题（与yolo_v11.py兼容）
            try:
                detection_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                detection_msg.header.stamp = rospy.Time.now()
                detection_msg.header.frame_id = "camera_color_frame"
                self.detection_image_pub.publish(detection_msg)
            except Exception as e:
                rospy.logwarn(f"发布检测图像失败: {e}")
            
            # 使用与yolo_v11.py相同的OpenCV显示方式
            if self.show_debug_image:
                try:
                    height, width = debug_image.shape[:2]
                    new_width = width // 2
                    new_height = height // 2
                    resized_frame = cv2.resize(debug_image, (new_width, new_height))
                    # 直接显示，OpenCV会在需要时自动创建窗口
                    cv2.imshow('Street Cruise YOLO', resized_frame)
                    cv2.waitKey(3)  # 与yolo_v11.py保持一致
                except cv2.error as e:
                    rospy.logerr(f"OpenCV window error: {e}")
            
        except Exception as e:
            rospy.logwarn(f"发布调试图像失败: {e}")

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
        """航点飞行控制（行人跟踪优先级最高）"""
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            return 0.0, 0.0, 0.0, 0.0, True  # 任务完成
        
        current_wp = self.waypoints[self.current_waypoint_index]
        
        # 优先级1: 行人跟踪逻辑（最高优先级）
        if self._check_detection_status():
            # 使用YOLO + ByteTracker进行检测和追踪
            tracked_target = self._detect_and_track_human()
            
            if tracked_target:
                # 执行行人跟踪控制
                vx, vy, vz, yaw_rate, tracking_active = self._human_tracking_control(tracked_target)
                
                # 立即进入跟踪模式（无条件）
                rospy.loginfo(f"🚨 立即追踪行人 ID:{tracked_target['id']}, 置信度:{tracked_target['confidence']:.3f}")
                rospy.loginfo(f"追踪控制指令: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, yaw_rate={yaw_rate:.2f}")
                return vx, vy, vz, yaw_rate, False
            else:
                # 检查追踪超时
                if (self.current_tracked_human and 
                    time.time() - self.tracking_start_time > self.human_tracking_timeout):
                    rospy.loginfo("行人追踪超时，返回航点飞行")
                    self.current_tracked_human = None
                else:
                    rospy.logdebug_throttle(5.0, "未检测到行人，继续航点飞行")
        
        # 优先级2: 航点飞行控制（行人跟踪未激活时）
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
        
        # 正常航点飞行控制
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
            # 高度控制：严格限制垂直速度
            vz = np.clip(dz * 0.3, -0.3, 0.3)  # 进一步降低垂直速度
            yaw_rate = np.clip(yaw_error * 1.0, -self.yaw_rate_limit, self.yaw_rate_limit)
        else:
            vx = 0.0
            vy = 0.0
            vz = np.clip(dz * 0.3, -0.3, 0.3)  # 进一步降低垂直速度
            yaw_rate = np.clip(yaw_error * 1.0, -self.yaw_rate_limit, self.yaw_rate_limit)
        
        # 严格的高度限制检查
        if self.current_position:
            current_height = self.current_position.z
            if current_height >= self.max_altitude - 0.2:  # 接近5米时
                vz = min(vz, -0.1)  # 强制下降
                rospy.logwarn_throttle(2.0, f"航点飞行高度限制: 当前{current_height:.2f}m，强制下降")
            elif current_height <= self.min_altitude + 0.1:  # 接近1米时
                vz = max(vz, 0.1)  # 强制上升
                rospy.logwarn_throttle(2.0, f"航点飞行高度限制: 当前{current_height:.2f}m，强制上升")
        
        return vx, vy, vz, yaw_rate, False

    # 回调函数
    def _pose_cb(self, msg: PoseStamped):
        self.current_position = msg.pose.position
        q = msg.pose.orientation
        q_ = Quaternion(q.w, q.x, q.y, q.z)
        self.current_yaw = q_.yaw_pitch_roll[0]

    def _rgb_cb(self, msg: Image):
        """优化的图像回调函数，提高画质（修复cv_bridge兼容性问题）"""
        # 总是尝试手动解码，不依赖cv_bridge
        try:
            # 直接使用NumPy手动解码
            import numpy as np
            
            if msg.encoding == "bgr8":
                data = np.frombuffer(msg.data, dtype=np.uint8)
                expected_size = msg.height * msg.width * 3
                if len(data) == expected_size:
                    self.rgb_image = data.reshape((msg.height, msg.width, 3)).copy()
                    self.image_width = msg.width
                    self.image_height = msg.height
                    return
            elif msg.encoding == "rgb8":
                data = np.frombuffer(msg.data, dtype=np.uint8)
                expected_size = msg.height * msg.width * 3
                if len(data) == expected_size:
                    rgb_image = data.reshape((msg.height, msg.width, 3))
                    self.rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR).copy()
                    self.image_width = msg.width
                    self.image_height = msg.height
                    return
            
            rospy.logwarn_throttle(5.0, f"不支持的图像编码: {msg.encoding}")
            self.rgb_image = None
            
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"手动图像解码失败: {str(e)[:100]}")
            self.rgb_image = None
            return
            
        # 如果手动解码失败，才尝试cv_bridge（作为备用）
        if not self.cv_bridge_working:
            return
            
        try:
            # 方法1：尝试使用bgr8编码
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # 记录图像信息用于调试
            if hasattr(self, '_image_count'):
                self._image_count += 1
            else:
                self._image_count = 1
                rospy.loginfo(f"图像订阅成功: {msg.width}x{msg.height}, 编码: {msg.encoding}")
            
        except Exception as e1:
            # 检测是否是LIBFFI错误
            if "LIBFFI_BASE" in str(e1) or "ffi_type_pointer" in str(e1):
                rospy.logwarn("检测到cv_bridge LIBFFI兼容性问题，禁用图像处理")
                self.cv_bridge_working = False
                self.rgb_image = None
                return
            
            try:
                # 方法2：尝试使用rgb8编码
                self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                # 转换RGB到BGR
                self.rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR)
                rospy.logwarn(f"使用rgb8编码转换: {msg.encoding}")
            except Exception as e2:
                try:
                    # 方法3：使用原始编码
                    self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    rospy.logwarn(f"使用原始编码: {msg.encoding}")
                except Exception as e3:
                    try:
                        # 方法4：手动解码（最后的备用方案）
                        import numpy as np
                        
                        if msg.encoding == "bgr8":
                            # 手动解码bgr8
                            data = np.frombuffer(msg.data, dtype=np.uint8)
                            self.rgb_image = data.reshape((msg.height, msg.width, 3))
                        elif msg.encoding == "rgb8":
                            # 手动解码rgb8并转换为bgr8
                            data = np.frombuffer(msg.data, dtype=np.uint8)
                            rgb_image = data.reshape((msg.height, msg.width, 3))
                            self.rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
                        else:
                            rospy.logerr(f"不支持的图像编码: {msg.encoding}")
                            self.rgb_image = None
                            return
                        
                        rospy.logwarn(f"手动解码图像成功: {msg.encoding}")
                        
                    except Exception as e4:
                        rospy.logerr(f"所有图像转换方法都失败: {e1}, {e2}, {e3}, {e4}")
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
        """配置云台初始设置"""
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

    def _publish_gimbal(self, pitch_deg=-45.0, yaw_deg=0.0, roll_deg=0.0):
        """发布云台控制指令，只设置初始俯仰角"""
        msg = MountControl()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.mode = 2
        msg.pitch = pitch_deg
        msg.yaw = yaw_deg
        msg.roll = roll_deg
        self.mount_pub.publish(msg)



    # 飞行控制
    def _publish_velocity(self, vx, vy, vz, yaw_rate):
        # 如果高度控制激活，不发布速度控制指令
        if self.height_control_active:
            rospy.logwarn_throttle(2.0, "高度控制激活中，跳过速度控制")
            return
            
        target = PositionTarget()
        target.coordinate_frame = self.velocity_frame
        target.type_mask = (PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +
                            PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                            PositionTarget.IGNORE_YAW)
        target.velocity.x = float(np.clip(vx, -self.max_forward_velocity, self.max_forward_velocity))
        target.velocity.y = float(np.clip(vy, -self.lateral_velocity, self.lateral_velocity))
        target.velocity.z = float(vz)  # 不再额外限制垂直速度，避免下降问题
        target.yaw_rate = float(np.clip(yaw_rate, -self.yaw_rate_limit, self.yaw_rate_limit))
        
        # 额外的高度检查
        if self.current_position and self.current_position.z > self.max_altitude:
            rospy.logwarn("高度超限，不发布速度控制")
            return
            
        self.setpoint_pub.publish(target)

    def _publish_position(self, x, y, z, yaw):
        # 严格限制高度在安全范围内
        z_clamped = np.clip(z, self.min_altitude, self.max_altitude)
        
        t = PositionTarget()
        t.coordinate_frame = self.position_frame
        t.type_mask = (PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
                       PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                       PositionTarget.IGNORE_YAW_RATE)
        t.position.x = float(x)
        t.position.y = float(y)
        t.position.z = float(z_clamped)  # 使用限制后的高度
        t.yaw = float(yaw)
        self.setpoint_pub.publish(t)
        
        # 如果高度被限制，记录警告
        if z != z_clamped:
            rospy.logwarn(f"位置控制高度限制: 请求{z:.2f}m -> 限制为{z_clamped:.2f}m")
        
        # 额外检查：如果当前高度超过限制，强制调整
        if self.current_position:
            current_height = self.current_position.z
            if current_height > self.max_altitude:
                rospy.logerr(f"⚠️ 当前高度{current_height:.2f}m超过限制{self.max_altitude}m！")
            elif current_height < self.min_altitude:
                rospy.logerr(f"⚠️ 当前高度{current_height:.2f}m低于限制{self.min_altitude}m！")

    # MAVROS操作
    def _ensure_connection(self, timeout=15):
        try:
            rospy.loginfo("等待MAVROS连接...")
            state = rospy.wait_for_message(f"{self.vehicle_type}_{self.vehicle_id}/mavros/state", State, timeout=timeout)
            if state.connected:
                rospy.loginfo("MAVROS连接成功")
                return True
            else:
                rospy.logwarn("MAVROS未连接")
                return False
        except rospy.ROSException as e:
            rospy.logerr(f"等待MAVROS连接超时: {e}")
            rospy.logerr("请确保Gazebo和MAVROS已启动")
            return False
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
        rospy.loginfo("开始启动街道巡航任务...")
        
        if not self._ensure_connection():
            rospy.logerr("MAVROS未连接，无法启动任务")
            rospy.logerr("请先启动Gazebo和MAVROS:")
            rospy.logerr("  roslaunch px4 mavros_posix_sitl.launch")
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
                # 候选优先级列表（包含高分辨率相机话题）
                base = f"{self.vehicle_type}_{self.vehicle_id}"
                candidates = [
                    self.rgb_topic,
                    # 高分辨率相机话题（优先）
                    f"/{base}/camera/color/image_raw",
                    f"/{base}/camera/rgb/image_raw",
                    f"/{base}/camera/image_raw",
                    f"/{base}/cgo3_camera/image_raw",
                    f"/{base}/camera/image_rect_color",
                    f"/{base}/camera/image_rect",
                    # 标准分辨率话题
                    "/camera/color/image_raw",
                    "/camera/rgb/image_raw",
                    "/camera/image_raw",
                    "/camera/image_rect_color",
                    "/camera/image_rect",
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
            # 设置云台初始俯仰角-45°
            self._publish_gimbal(pitch_deg=-45.0, yaw_deg=0.0, roll_deg=0.0)
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

            # 监控高度限制
            height_ok = self._monitor_height()
            if not height_ok:
                if self.emergency_landing:
                    rospy.logerr("🚨 紧急降落模式启动")
                    self.set_auto_land()
                    self.mission_active = False
                    break
                elif self.height_control_active:
                    # 高度控制激活时，暂停其他控制，只执行强制高度控制
                    rospy.logwarn_throttle(2.0, "高度控制激活中，暂停航点飞行和追踪控制")
                    self.rate.sleep()
                    continue

            # 航点飞行控制（仅在高度正常时执行）
            if self.flight_mode == "WAYPOINT_FLIGHT" and height_ok:
                vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd, mission_complete = self._waypoint_flight_control()
                
                if mission_complete:
                    rospy.loginfo("航点飞行任务完成！")
                    self.mission_active = False
                    break
                
                # 发布控制指令
                self._publish_velocity(vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd)
                # 删除云台控制调用
                # self._publish_gimbal()  # 云台跟随无人机航向
                
                # 发布调试图像
                if self.show_debug_image and self.rgb_image is not None and self.cv_bridge_working:
                    self._publish_debug_image()
                
                # 状态信息
                if self.current_waypoint_index < len(self.waypoints):
                    current_wp = self.waypoints[self.current_waypoint_index]
                    if self.current_position is not None:
                        dx = current_wp['x'] - self.current_position.x
                        dy = current_wp['y'] - self.current_position.y
                        dz = current_wp['z'] - self.current_position.z
                        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                        
                        # 显示航点飞行状态
                        if self.current_tracked_human:
                            rospy.loginfo_throttle(5.0, f"航点飞行（追踪行人ID:{self.current_tracked_human['id']}）: {self.current_waypoint_index+1}/{len(self.waypoints)}")
                    else:
                            rospy.loginfo_throttle(5.0, f"航点飞行: {self.current_waypoint_index+1}/{len(self.waypoints)}")
            
            self.rate.sleep()

        # 降落
        self.set_auto_land()
        time.sleep(8.0)
        return True


def main():
    try:
        rospy.loginfo("=== 街道巡航YOLO行人跟踪系统 ===")
        rospy.loginfo("初始化无人机控制节点...")
        
        drone = StreetCruiseYolo("typhoon_h480", "0")
        
        rospy.loginfo("启动街道巡航任务...")
        ok = drone.start()
        
        if ok:
            rospy.loginfo("街道巡航任务完成或已进入降落")
        else:
            rospy.logerr("街道巡航任务启动失败")
            rospy.logerr("请检查:")
            rospy.logerr("1. Gazebo是否已启动")
            rospy.logerr("2. MAVROS是否已连接")
            rospy.logerr("3. YOLO权重文件是否存在")
            
    except KeyboardInterrupt:
        rospy.loginfo("用户中断，尝试降落...")
        try:
            drone.set_auto_land()
        except Exception:
            pass
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
