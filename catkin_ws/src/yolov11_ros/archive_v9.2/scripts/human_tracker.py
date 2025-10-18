#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
人体跟踪节点 - v9.0 完全重写版（基于plan3_optimized.py）
核心思想：
1. 稳定追踪：3米距离，视觉伺服，不抖动
2. 加速匹配：快速匹配人体速度
3. 保持框在中心：多级转向控制
4. 智能后退：人太近时后退增大视野
"""

import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
from yolov11_ros_msgs.msg import BoundingBoxes
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
from mavros_msgs.msg import MountControl
from collections import deque

class HumanTracker:
    def __init__(self):
        rospy.init_node('human_tracker', anonymous=True)
        
        # ========== 无人机标识 ==========
        self.drone_id = rospy.get_param('~drone_id', 0)
        self.vehicle_ns = rospy.get_param('~vehicle_ns', 'typhoon_h480_0')
        
        # ========== 图像参数 ==========
        self.image_width = rospy.get_param("~image_width", 640)
        self.image_height = rospy.get_param("~image_height", 360)
        self.image_center_x = self.image_width // 2
        self.image_center_y = self.image_height // 2
        
        # ========== YOLO类别配置 ==========
        self.color_classes = ['blue', 'green', 'white', 'brown', 'red', 'red1', 'red2']
        self.human_classes = ['person', 'human']
        self.person_id = rospy.get_param('~person_id', 0)
        
        # ========== 📏 距离与box_size关系（用于距离估算） ==========
        # 通过实测标定：2米→2400px², 3米→1067px², 4米→600px², 5米→384px²
        self.distance_calibration = {
            2.0: 2400,
            3.0: 1067,
            4.0: 600,
            5.0: 384
        }
        
        # ========== 追踪参数（统一设计） ==========
        # 1. 稳定追踪阶段
        self.ideal_distance = 3.0  # 理想追踪距离3米
        self.ideal_box_size = 1067  # 3米对应的box_size
        
        # 2. 距离分级
        self.very_close_size = 1600   # 非常近：<2.5米
        self.close_size = 1200         # 近：2.5-2.8米
        self.ideal_size_min = 950      # 理想范围：2.8-3.2米
        self.ideal_size_max = 1200
        self.far_size = 600            # 远：4米
        self.very_far_size = 384       # 非常远：5米
        
        # 3. 速度参数
        self.stable_tracking_speed_range = (0.3, 2.5)  # 稳定追踪速度范围
        self.acceleration_max_speed = 8.0  # 加速追踪最大速度
        self.deceleration_distance = 3.0   # 3米开始减速
        
        # 4. 视觉伺服控制增益
        self.kp_distance = 2.0   # 距离误差 → 速度
        self.kd_distance = 20.0  # 距离变化率 → 速度增量
        
        # 5. 转向参数（让人居中）
        self.yaw_gains = {
            'very_close': 0.0012,   # |u| < 80
            'close': 0.0025,        # 80 < |u| < 150
            'medium': 0.0040,       # 150 < |u| < 220
            'far': 0.0060           # |u| > 220
        }
        
        # ========== 历史与平滑 ==========
        self.target_history = deque(maxlen=8)
        self.size_history = deque(maxlen=20)
        self.velocity_history = deque(maxlen=15)
        
        # ========== 人体速度估算 ==========
        self.person_speed = 0.0  # 估算的人体速度（m/s）
        
        # ========== 状态标志 ==========
        self.tracking_active = True
        self.tracking_mode = "IDLE"  # 追踪模式：STABLE/ACCELERATING/DECELERATING/RETREAT
        self.lost_target_count = 0
        self.lost_threshold = 5  # 丢失5帧才触发
        self.last_u = 0
        self.last_v = 0
        self.last_box_size = 0
        
        # ========== 目标锁定 ==========
        self.locked_target_color = None
        self.locked_target_position = None
        self.target_lock_start_time = None
        self.target_lock_duration = 15.0
        self.lock_position_tolerance = 100
        
        # ========== 速度控制 ==========
        self.x_velocity = 0.0
        self.y_velocity = 0.0
        self.z_velocity = 0.0
        self.yaw_rate = 0.0
        self.tracking_count = 0
        
        # ========== 高度保持 ==========
        self.target_height = 3.0
        self.height = 0.0
        self.height_tolerance = 0.8
        self.height_kp = 0.6
        self.max_z_velocity = 0.8
        
        # ========== 检测结果 ==========
        self.detections = None
        self.drone_position = None
        
        # ========== 跟踪计时 ==========
        self.tracking_start_time = None
        
        # ========== ROS发布者 ==========
        # ✅ 关键！waypoint_flight订阅这个话题（waypoint_flight负责转发到MAVROS）
        self.human_tracker_vel_pub = rospy.Publisher(f'/drone_{self.drone_id}/human_tracker/cmd_vel', Twist, queue_size=1)
        
        # 航点控制
        self.waypoint_pause_pub = rospy.Publisher(f'/drone_{self.drone_id}/waypoint_flight/pause', String, queue_size=1)
        self.waypoint_resume_pub = rospy.Publisher(f'/drone_{self.drone_id}/waypoint_flight/resume', String, queue_size=1)
        
        # 追踪声明
        self.tracking_claim_pub = rospy.Publisher(f'/drone_{self.drone_id}/tracking_claim', String, queue_size=1)
        
        # 追踪状态
        self.tracking_status_pub = rospy.Publisher('/human_tracker/status', Bool, queue_size=1)
        self.tracking_info_pub = rospy.Publisher('/human_tracker/info', String, queue_size=1)
        
        # 云台控制
        self.mount_pub = rospy.Publisher(f'/{self.vehicle_ns}/mavros/mount_control/command', MountControl, queue_size=1)
        
        # 记分系统发布者
        self.actor_blue_pub = rospy.Publisher('/actor_blue_info', ActorInfo, queue_size=1)
        self.actor_green_pub = rospy.Publisher('/actor_green_info', ActorInfo, queue_size=1)
        self.actor_white_pub = rospy.Publisher('/actor_white_info', ActorInfo, queue_size=1)
        self.actor_brown_pub = rospy.Publisher('/actor_brown_info', ActorInfo, queue_size=1)
        self.actor_red1_pub = rospy.Publisher('/actor_red1_info', ActorInfo, queue_size=1)
        self.actor_red2_pub = rospy.Publisher('/actor_red2_info', ActorInfo, queue_size=1)
        
        # ========== ROS订阅者 ==========
        detection_topic = rospy.get_param("~detection_topic", "/yolov11/bounding_boxes")
        pose_topic = rospy.get_param("~pose_topic", f"/{self.vehicle_ns}/mavros/local_position/pose")
        
        self.detection_sub = rospy.Subscriber(detection_topic, BoundingBoxes, self._detection_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self._pose_callback, queue_size=1)
        self.enable_sub = rospy.Subscriber('/human_tracker/enable', Bool, self._enable_callback)
        
        rospy.loginfo("=" * 80)
        rospy.loginfo(f"🚁 人体跟踪节点已启动 - v9.1 (plan3_optimized)")
        rospy.loginfo("=" * 80)
        rospy.loginfo(f"📍 无人机ID: {self.drone_id}")
        rospy.loginfo(f"📍 Vehicle: {self.vehicle_ns}")
        rospy.loginfo(f"📡 订阅检测: {detection_topic}")
        rospy.loginfo(f"📡 订阅位置: {pose_topic}")
        rospy.loginfo("📤 发布话题:")
        rospy.loginfo(f"   - /drone_{self.drone_id}/human_tracker/cmd_vel (→ waypoint_flight → MAVROS)")
        rospy.loginfo(f"   - /drone_{self.drone_id}/human_tracker/status (→ waypoint_flight暂停航点)")
        rospy.loginfo(f"⚙️  理想距离: {self.ideal_distance}m")
        rospy.loginfo(f"⚙️  控制架构: human_tracker → waypoint_flight → MAVROS")
        rospy.loginfo("=" * 80)
    
    def _detection_callback(self, data):
        """YOLO检测结果回调"""
        self.detections = data.bounding_boxes
        rospy.logdebug_throttle(2.0, f"📡 收到检测结果: {len(data.bounding_boxes)}个框")
    
    def _pose_callback(self, data):
        """位置回调"""
        self.height = data.pose.position.z
        self.drone_position = data.pose.position
    
    def _enable_callback(self, msg):
        """跟踪使能回调"""
        if msg.data and not self.tracking_active:
            self.tracking_active = True
            rospy.loginfo("🎯 人体跟踪已启用")
        elif not msg.data and self.tracking_active:
            self.tracking_active = False
            rospy.loginfo("⏸️ 人体跟踪已禁用")
    
    def estimate_distance_from_box_size(self, box_size):
        """
        根据box_size估算距离（米）
        使用反比关系：distance ≈ sqrt(ref_size / box_size) × ref_distance
        """
        if box_size < 50:
            return 10.0  # 太小，估算为很远
        
        # 使用3米作为参考点
        ref_distance = 3.0
        ref_size = 1067
        
        estimated_distance = math.sqrt(ref_size / box_size) * ref_distance
        return estimated_distance
    
    def estimate_person_speed(self, current_distance):
        """
        估算人体速度（m/s）
        基于距离变化率
        """
        if len(self.velocity_history) < 5:
            return 0.0
        
        # 计算最近5帧的距离变化
        recent_distances = list(self.velocity_history)[-5:]
        distance_changes = []
        
        for i in range(1, len(recent_distances)):
            distance_changes.append(recent_distances[i] - recent_distances[i-1])
        
        # 平均距离变化率（米/帧）
        avg_change_per_frame = sum(distance_changes) / len(distance_changes)
        
        # 转换为米/秒（60Hz → ×60）
        speed = -avg_change_per_frame * 60.0  # 负号：距离变小=人加速远离
        
        # 限制合理范围（0-10m/s）
        speed = max(0, min(10.0, speed))
        
        return speed
    
    def get_best_color_target(self):
        """选择最佳颜色目标"""
        if self.detections is None:
            rospy.logdebug_throttle(5.0, "❌ 未收到检测结果（self.detections is None）")
            return None
        
        if len(self.detections) == 0:
            rospy.logdebug_throttle(5.0, "❌ 检测结果为空")
            return None
        
        candidates = []
        
        for box in self.detections:
            detected_class = getattr(box, 'Class', '').lower()
            cls_id = getattr(box, 'id', None)
            prob = getattr(box, 'probability', 0.0)
            
            rospy.logdebug_throttle(3.0, f"🔍 检测到: Class={detected_class}, id={cls_id}, prob={prob:.2f}")
            
            # 检查是否是目标类别
            is_color = detected_class in self.color_classes
            is_human_by_name = detected_class in self.human_classes
            is_human_by_id = (cls_id == self.person_id)
            
            if not (is_color or is_human_by_name or is_human_by_id):
                rospy.logdebug_throttle(5.0, f"⏭️  跳过非目标类别: {detected_class}")
                continue
            
            # 添加置信度检查
            confidence_threshold = rospy.get_param('~detection_confidence', 0.3)
            if prob < confidence_threshold:
                rospy.logdebug_throttle(5.0, f"⏭️  跳过低置信度: {detected_class} (prob={prob:.2f} < {confidence_threshold})")
                continue
            
            center_x = (box.xmin + box.xmax) / 2
            center_y = (box.ymin + box.ymax) / 2
            box_size = (box.xmax - box.xmin) * (box.ymax - box.ymin)
            
            if box_size < 200:
                rospy.logdebug_throttle(5.0, f"⏭️  跳过过小框: {detected_class} (size={box_size} < 200)")
                continue
            
            distance_to_center = abs(center_x - self.image_center_x)
            
            rospy.loginfo_throttle(2.0, f"✅ 添加候选: {detected_class.upper()} prob={prob:.2f} size={int(box_size)}px²")
            
            candidates.append({
                'center_x': center_x,
                'center_y': center_y,
                'size': box_size,
                'distance_to_center': distance_to_center,
                'color': detected_class if is_color else 'person',
                'probability': prob
            })
        
        if len(candidates) == 0:
            rospy.logdebug_throttle(3.0, f"❌ 无有效候选目标（检测到{len(self.detections)}个框，但都不符合条件）")
            return None
        
        rospy.logdebug_throttle(3.0, f"✅ 找到{len(candidates)}个候选目标")
        
        # 目标锁定机制
        if self.locked_target_color is not None:
            current_time = rospy.Time.now().to_sec()
            lock_elapsed = current_time - self.target_lock_start_time if self.target_lock_start_time else 0
            
            if lock_elapsed < self.target_lock_duration:
                # 筛选相同颜色的候选
                locked_candidates = [c for c in candidates if c['color'] == self.locked_target_color]
                
                if len(locked_candidates) > 0:
                    if self.locked_target_position is not None:
                        # 选择最接近锁定位置的目标
                        min_distance = float('inf')
                        closest_candidate = None
                        
                        for c in locked_candidates:
                            pos_dist = math.sqrt(
                                (c['center_x'] - self.locked_target_position[0])**2 +
                                (c['center_y'] - self.locked_target_position[1])**2
                            )
                            if pos_dist < min_distance:
                                min_distance = pos_dist
                                closest_candidate = c
                        
                        if min_distance < self.lock_position_tolerance:
                            candidates = [closest_candidate]
                        else:
                            candidates = locked_candidates
                    else:
                        candidates = locked_candidates
                else:
                    return None
                else:
                # 锁定时间到，解锁
                self.locked_target_color = None
                self.locked_target_position = None
                self.target_lock_start_time = None
        
        # 优先级：越靠近中间且越近（box_size越大）的先追踪
        if len(candidates) > 0:
            max_size = max([c['size'] for c in candidates])
            min_size = min([c['size'] for c in candidates])
            max_dist = max([c['distance_to_center'] for c in candidates])
            
            for c in candidates:
                # 归一化 size
                if max_size > min_size:
                    size_score = (c['size'] - min_size) / (max_size - min_size)
            else:
                    size_score = 1.0
                
                # 归一化 distance_to_center
                if max_dist > 0:
                    center_score = 1.0 - (c['distance_to_center'] / max_dist)
            else:
                    center_score = 1.0
                
                # 综合评分：距离权重60%，居中权重40%
                c['score'] = size_score * 0.6 + center_score * 0.4
        
        best = max(candidates, key=lambda x: x['score'])
        self.target_history.append(best)
        
        rospy.loginfo_throttle(3.0, f"🎯 选择最佳目标: {best['color'].upper()} 评分={best['score']:.2f} size={int(best['size'])}px²")
        
        # 历史平滑
        if len(self.target_history) >= 5:
            avg_x = sum([t['center_x'] for t in self.target_history]) / len(self.target_history)
            avg_y = sum([t['center_y'] for t in self.target_history]) / len(self.target_history)
            avg_size = sum([t['size'] for t in self.target_history]) / len(self.target_history)
            best['center_x'] = avg_x
            best['center_y'] = avg_y
            best['size'] = avg_size
        
        return best
    
    def compute_height_control(self):
        """高度保持"""
        if self.drone_position is None:
            return 0.0
        
        current_height = self.height
        height_error = self.target_height - current_height
        
        if abs(height_error) < self.height_tolerance:
            return 0.0
        
        z_velocity = self.height_kp * height_error
        z_velocity = max(-self.max_z_velocity, min(self.max_z_velocity, z_velocity))
        
        return z_velocity
    
    def publish_cmd_vel(self, vx, vy, vz, yaw_rate):
        """发布速度命令到waypoint_flight（由waypoint_flight转发到MAVROS）"""
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = vz if vz != 0 else self.compute_height_control()
        twist.angular.z = yaw_rate
        
        # ✅ 关键！发布到waypoint_flight订阅的话题
        # waypoint_flight会转换为PositionTarget并发送到MAVROS
        self.human_tracker_vel_pub.publish(twist)
    
    def _publish_gimbal(self, pitch_deg=-40.0, yaw_deg=0.0, roll_deg=0.0):
        """发布云台控制指令"""
        msg = MountControl()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.mode = 2
        msg.pitch = pitch_deg
        msg.yaw = yaw_deg
        msg.roll = roll_deg
        self.mount_pub.publish(msg)
    
    def _publish_human_detection(self, target, center_u, center_v):
        """发布人体检测信息到记分系统"""
        if not self.drone_position:
            return
        
        try:
            horizontal_offset = (center_u - self.image_center_x) * 0.01
            
            # 估算距离
            box_size = target['size']
            distance = self.estimate_distance_from_box_size(box_size)
            distance = float(np.clip(distance, 0.5, 30.0))
            
            # 计算人体世界坐标
            human_x = self.drone_position.x + distance * 0.8
            human_y = self.drone_position.y + horizontal_offset
            
            # 创建ActorInfo消息
            actor_msg = ActorInfo()
            detected_cls_name = target['color']
            if detected_cls_name in self.color_classes:
                actor_msg.cls = detected_cls_name
            else:
                actor_msg.cls = 'blue'
            
            actor_msg.x = human_x
            actor_msg.y = human_y
            
            # 兼容性处理
            try:
                actor_msg.z = 0.0
            except AttributeError:
                pass
            
            try:
                actor_msg.distance = distance
            except AttributeError:
                pass
            
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
            
            rospy.logdebug_throttle(5.0, f"📡 发布人体检测: 位置({human_x:.2f}, {human_y:.2f}), 距离{distance:.2f}m")
            
        except Exception as e:
            rospy.logwarn(f"发布人体检测信息失败: {e}")
    
    def track_target(self):
        """
        统一的追踪逻辑（基于plan3_optimized.py）
        """
        target = self.get_best_color_target()
        
        # ==================== 目标丢失处理 ====================
        if target is None:
            self.lost_target_count += 1
            
            # 短暂丢失（1-4帧）：保持当前速度
            if self.lost_target_count <= 4:
                self.publish_cmd_vel(self.x_velocity * 0.6, 0, 0, self.yaw_rate * 0.3)
                if self.lost_target_count == 1:
                    rospy.loginfo(f"   ⏸️  目标暂时丢失，保持当前状态...")
                return
            
            # 确认丢失（第5帧）
            if self.lost_target_count == 5:
                rospy.logwarn("⚠️  目标丢失，恢复航点飞行")
                
                # ✅ 关键！发布tracking_status = False（告知waypoint_flight恢复）
                self.tracking_status_pub.publish(Bool(data=False))
                rospy.loginfo("✅ 发布tracking_status=False -> waypoint_flight恢复航点")
                
                # 释放追踪声明
                self.tracking_claim_pub.publish(String(data="released"))
                
                # 重置状态
                self.tracking_start_time = None
                self.locked_target_color = None
                self.locked_target_position = None
                self.target_lock_start_time = None
            
            # 悬停等待
            if self.lost_target_count <= 120:
                self.publish_cmd_vel(0, 0, 0, 0)
            else:
                # 丢失过久，保持悬停
                self.publish_cmd_vel(0, 0, 0, 0)
            return
        
        # ==================== 重新捕获目标 ====================
        if self.lost_target_count > 0:
            color_name = target['color'].upper()
            rospy.loginfo(f"✅ 重新捕获目标 [{color_name}]！（丢失{self.lost_target_count}帧）")
            self.lost_target_count = 0
        
        self.tracking_count += 1
        
        # ==================== 目标锁定 ====================
        target_color = target['color']
        target_center_x = target['center_x']
        target_center_y = target['center_y']
        current_time = rospy.Time.now().to_sec()
        
        if self.locked_target_color is None:
            # 首次锁定
            self.locked_target_color = target_color
            self.locked_target_position = (target_center_x, target_center_y)
            self.target_lock_start_time = current_time
            rospy.loginfo("=" * 80)
            rospy.loginfo(f"🔒 锁定目标: {target_color.upper()} (持续{self.target_lock_duration:.0f}秒)")
            
            # ✅ 关键！通过tracking_status告知waypoint_flight暂停航点
            if self.tracking_start_time is None:
                self.tracking_start_time = rospy.get_time()
                
                # ✅ 发布tracking_status = True（waypoint_flight订阅这个）
                self.tracking_status_pub.publish(Bool(data=True))
                rospy.loginfo("=" * 80)
                rospy.loginfo("✅ 发布tracking_status=True -> waypoint_flight")
                rospy.loginfo("🎯 暂停航点飞行，开始追踪")
                rospy.loginfo("=" * 80)
        else:
            # 持续追踪：更新锁定位置
            self.locked_target_position = (target_center_x, target_center_y)
            
            # ✅ 持续发布tracking_status = True
            self.tracking_status_pub.publish(Bool(data=True))
        
        # ==================== 提取目标信息 ====================
        u = target_center_x - self.image_center_x  # 横向偏移
        v = target_center_y  # 纵向位置
        box_size = target['size']
        
        self.last_u = u
        self.last_v = v
        self.last_box_size = box_size
        
        # 估算距离
        estimated_distance = self.estimate_distance_from_box_size(box_size)
        
        # 更新历史
        self.size_history.append(box_size)
        self.velocity_history.append(estimated_distance)
        
        # 估算人体速度
        self.person_speed = self.estimate_person_speed(estimated_distance)
        
        # ==================== 1. 判断追踪模式 ====================
        # 获取平滑后的box_size
        smoothed_box_size = sum(list(self.size_history)[-5:]) / min(5, len(self.size_history))
        
        # 判断距离分级
        if smoothed_box_size > self.very_close_size:
            distance_level = "VERY_CLOSE"  # <2.5米
        elif smoothed_box_size > self.close_size:
            distance_level = "CLOSE"  # 2.5-2.8米
        elif self.ideal_size_min <= smoothed_box_size <= self.ideal_size_max:
            distance_level = "IDEAL"  # 2.8-3.2米（理想）
        elif smoothed_box_size > self.far_size:
            distance_level = "MEDIUM"  # 3.2-4米
        elif smoothed_box_size > self.very_far_size:
            distance_level = "FAR"  # 4-5米
        else:
            distance_level = "VERY_FAR"  # >5米
        
        # 判断人是否在加速
        is_person_accelerating = self.person_speed > 1.5
        
        # 判断人在屏幕位置
        in_lower = v > self.image_height * 0.65  # 下方
        in_center_x = abs(u) < 150  # 横向中间
        
        # 判断人是否快速接近
        is_approaching_fast = False
        if len(self.size_history) >= 10:
            recent_avg = sum(list(self.size_history)[-5:]) / 5.0
            previous_avg = sum(list(self.size_history)[-10:-5]) / 5.0
            size_change_rate = (recent_avg - previous_avg) / previous_avg if previous_avg > 0 else 0
            is_approaching_fast = size_change_rate > 0.10
        
        # ==================== 2. 模式切换逻辑 ====================
        
        # 快速后退：人朝向无人机快速接近
        if is_approaching_fast and distance_level not in ["VERY_FAR", "FAR"]:
            self.tracking_mode = "RETREAT_FAST"
        
        # 后退模式：人太近且在下方
        elif distance_level in ["VERY_CLOSE", "CLOSE"] and in_lower and in_center_x:
            self.tracking_mode = "RETREAT"
        
        # 减速模式的保持与退出
        elif self.tracking_mode == "DECELERATING":
            if distance_level == "IDEAL":
                self.tracking_mode = "STABLE"
            elif distance_level in ["VERY_CLOSE", "CLOSE"]:
                self.tracking_mode = "ADJUSTING"
        
        # 加速模式：距离>3.5米 或 人在加速
        elif (estimated_distance > 3.5) or (is_person_accelerating and distance_level not in ["VERY_CLOSE", "CLOSE", "IDEAL"]):
            self.tracking_mode = "ACCELERATING"
        
        # 加速切换到减速
        elif self.tracking_mode == "ACCELERATING" and estimated_distance < 3.2 and estimated_distance > 2.5:
            self.tracking_mode = "DECELERATING"
        
        # 稳定模式：理想距离范围
        elif distance_level == "IDEAL":
            self.tracking_mode = "STABLE"
        
        # 默认：根据距离调整
        else:
            self.tracking_mode = "ADJUSTING"
        
        # ==================== 3. 速度计算 ====================
        
        if self.tracking_mode == "STABLE":
            # 稳定追踪：视觉伺服，保持3米
            size_error = (self.ideal_box_size - smoothed_box_size) / self.ideal_box_size
            
            # 计算变化率
            if len(self.size_history) >= 10:
                recent_avg = sum(list(self.size_history)[-5:]) / 5.0
                previous_avg = sum(list(self.size_history)[-10:-5]) / 5.0
                change_rate = (recent_avg - previous_avg) / previous_avg if previous_avg > 0 else 0
            else:
                change_rate = 0
            
            # P + D控制
            base_speed = self.kp_distance * size_error
            speed_increment = -self.kd_distance * change_rate
            desired_speed = base_speed + speed_increment
            
            # 限制在稳定范围
            desired_speed = max(self.stable_tracking_speed_range[0], 
                              min(self.stable_tracking_speed_range[1], desired_speed))
            
            # 平滑滤波
            alpha = 0.15
            self.x_velocity = alpha * desired_speed + (1 - alpha) * self.x_velocity
        
        elif self.tracking_mode == "ACCELERATING":
            # 加速模式：快速匹配人体速度
            distance_error = estimated_distance - self.ideal_distance
            compensation = distance_error * 0.8
            
            desired_speed = self.person_speed + compensation + 1.5
            
            # 平滑滤波（快速响应）
            alpha = 0.70
            self.x_velocity = alpha * desired_speed + (1 - alpha) * self.x_velocity
        
        elif self.tracking_mode == "DECELERATING":
            # 减速模式：平滑减速
            distance_error = estimated_distance - self.ideal_distance
            
            if distance_error > 0.5:
                desired_speed = self.person_speed + distance_error * 0.8
                alpha = 0.30
            elif distance_error > 0.2:
                desired_speed = self.person_speed + distance_error * 0.5
                alpha = 0.25
            else:
                desired_speed = self.person_speed + 0.2
                alpha = 0.18
            
            self.x_velocity = alpha * desired_speed + (1 - alpha) * self.x_velocity
        
        elif self.tracking_mode == "RETREAT_FAST":
            # 快速后退
            if len(self.size_history) >= 10:
                recent_avg = sum(list(self.size_history)[-5:]) / 5.0
                previous_avg = sum(list(self.size_history)[-10:-5]) / 5.0
                size_change_rate = (recent_avg - previous_avg) / previous_avg if previous_avg > 0 else 0
                retreat_speed = -1.5 - size_change_rate * 8.0
                retreat_speed = max(-2.5, min(-1.2, retreat_speed))
            else:
                retreat_speed = -1.8
            
            if abs(u) < 150:
                self.x_velocity = retreat_speed
            else:
                self.x_velocity = retreat_speed * 0.85
        
        elif self.tracking_mode == "RETREAT":
            # 后退模式
            if abs(u) < 150:
                self.x_velocity = -1.2
            else:
                self.x_velocity = -1.0
        
        else:  # ADJUSTING
            # 调整模式
            size_error = (self.ideal_box_size - smoothed_box_size) / self.ideal_box_size
            desired_speed = self.kp_distance * size_error + self.person_speed * 0.5
            
            alpha = 0.40
            self.x_velocity = alpha * desired_speed + (1 - alpha) * self.x_velocity
        
        # ==================== 4. 转向控制（让人居中） ====================
        if self.tracking_mode not in ["RETREAT", "RETREAT_FAST"]:
            abs_u = abs(u)
            
            if abs_u < 80:
                yaw_gain = self.yaw_gains['very_close']
            elif abs_u < 150:
                yaw_gain = self.yaw_gains['close']
            elif abs_u < 220:
                yaw_gain = self.yaw_gains['medium']
            else:
                yaw_gain = self.yaw_gains['far']
            
            desired_yaw_rate = -u * yaw_gain
            
            # 平滑转向
            self.yaw_rate = 0.30 * desired_yaw_rate + 0.70 * self.yaw_rate
            
            # 限制yaw_rate
            self.yaw_rate = max(-0.5, min(0.5, self.yaw_rate))
        elif self.tracking_mode == "RETREAT" and abs(u) >= 150:
            # 后退时需要转向
            self.yaw_rate = 0.3 if u < 0 else -0.3
        elif self.tracking_mode == "RETREAT_FAST" and abs(u) >= 150:
            # 快速后退时需要转向
            self.yaw_rate = 0.35 if u < 0 else -0.35
        else:
            self.yaw_rate = 0.0
        
        # ==================== 5. 发布命令 ====================
        self.publish_cmd_vel(self.x_velocity, 0, 0, self.yaw_rate)
        rospy.logdebug_throttle(1.0, f"✈️ 发布速度: vx={self.x_velocity:.2f}, yaw={self.yaw_rate:.2f}")
        
        # 发布记分系统信息
        self._publish_human_detection(target, target_center_x, target_center_y)
        
        # ==================== 6. 调试输出 ====================
        if self.tracking_count % 15 == 0:
            mode_icons = {
                "STABLE": "🎯稳定",
                "ACCELERATING": "🚀加速",
                "DECELERATING": "🔽减速",
                "RETREAT": "⬅️后退",
                "RETREAT_FAST": "⚡快退",
                "ADJUSTING": "🔄调整"
            }
            mode_str = mode_icons.get(self.tracking_mode, self.tracking_mode)
            
            color_name = target['color'].upper()
            
            rospy.loginfo(f"[{mode_str}][{color_name}] 距离={estimated_distance:.1f}m({distance_level}) "
                         f"人速={self.person_speed:.1f}m/s | "
                         f"v={self.x_velocity:.2f}m/s yaw={self.yaw_rate:.2f} | "
                         f"横偏={int(u):+4d}px size={int(smoothed_box_size)}px²")
    
    def tracking_loop(self):
        """主跟踪循环 - 60Hz"""
        rate_hz = rospy.get_param('~control_rate_hz', 60)
        rate = rospy.Rate(rate_hz)
        
        rospy.loginfo("=" * 80)
        rospy.loginfo("🚀 人体跟踪循环已启动（plan3_optimized算法）")
        rospy.loginfo(f"📡 发布频率: {rate_hz}Hz")
        rospy.loginfo(f"📡 控制话题: /xtdrone/{self.vehicle_ns}/cmd_vel_flu")
        rospy.loginfo(f"📡 命令话题: /xtdrone/{self.vehicle_ns}/cmd")
        rospy.loginfo("=" * 80)
        
        loop_count = 0
        while not rospy.is_shutdown():
            try:
                loop_count += 1
                
                # 云台控制始终保持-40°
                self._publish_gimbal(pitch_deg=-40.0)
                
                # 每5秒输出一次状态
                if loop_count % 300 == 0:
                    det_status = f"有{len(self.detections)}个检测" if self.detections else "无检测"
                    pos_status = f"位置({self.drone_position.x:.1f},{self.drone_position.y:.1f},{self.height:.1f})" if self.drone_position else "无位置"
                    rospy.loginfo(f"📊 状态: {det_status}, {pos_status}, 追踪{'启用' if self.tracking_active else '禁用'}")
                
                # 检查跟踪状态
                if self.tracking_active:
                    self.track_target()
                else:
                    self.tracking_status_pub.publish(Bool(data=False))
                
            except Exception as e:
                rospy.logerr(f"❌ 跟踪循环错误: {e}")
                import traceback
                rospy.logerr(traceback.format_exc())
            
            rate.sleep()
    
    def run(self):
        """运行跟踪器"""
        try:
            rospy.loginfo("🚀 人体跟踪器运行中...")
            self.tracking_loop()
        except rospy.ROSInterruptException:
            rospy.loginfo("人体跟踪节点关闭")

if __name__ == '__main__':
    try:
        tracker = HumanTracker()
        tracker.run()
    except Exception as e:
        rospy.logerr(f"人体跟踪节点启动失败: {e}")
