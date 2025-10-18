#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
方案3优化版：统一框架的视觉伺服追踪
核心思想：
1. 稳定追踪：3米距离，视觉伺服，不抖动
2. 加速匹配：快速匹配人体速度，5米时减速
3. 跟在背后：一定跟在人后面，不跟侧面
4. 智能后退：人太近时后退增大视野
5. 智能搜索：根据消失位置决定搜索策略
"""

import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image
from yolov11_ros_msgs.msg import BoundingBoxes
from cv_bridge import CvBridge
from mavros_msgs.msg import State
from mavros_msgs.msg import MountControl
import std_msgs.msg
from collections import deque

class OptimizedVisualServo:
    def __init__(self):
        rospy.init_node('plan3_optimized', anonymous=True)
        rospy.loginfo("=" * 80)
        rospy.loginfo("🚁 方案3优化版：统一框架视觉伺服追踪系统")
        rospy.loginfo("=" * 80)
        
        # ========== 无人机标识 ==========
        self.vehicle_type = "typhoon_h480"
        self.vehicle_id = "0"
        
        # ========== 位置与航点 ==========
        self.current_pose = PoseStamped()
        self.current_state = State()
        self.takeoff_position = None
        self.flight_height = 4.5
        
        # ========== 云台控制 ==========
        self.gimbal_pitch = -40.0
        self.gimbal_roll = 0.0
        self.gimbal_yaw = 0.0
        self.hover_duration = 100
        
        # 巡航路径
        self.patrol_waypoints_absolute = [
            (-13, 2, self.flight_height), (5, 2, self.flight_height),
            (20, 2, self.flight_height), (35, 2, self.flight_height),
            (43, 10, self.flight_height), (43, 25, self.flight_height),
            (43, 43, self.flight_height), (30, 43, self.flight_height),
            (15, 43, self.flight_height), (0, 43, self.flight_height),
            (-10, 43, self.flight_height), (-13, 30, self.flight_height),
            (-13, 15, self.flight_height), (-13, 2, self.flight_height),
        ]
        self.waypoints = self.patrol_waypoints_absolute
        self.current_waypoint_index = 0
        self.waypoint_tolerance = 3.0
        self.patrol_speed = 2.8
        
        # ========== 状态机 ==========
        self.state = "INIT"
        self.state_counter = 0
        
        # ========== YOLO检测 ==========
        self.bridge = CvBridge()
        self.current_image = None
        self.detections = None
        
        # ========== 图像参数 ==========
        self.image_width = 640
        self.image_height = 480
        self.image_center_x = self.image_width // 2
        self.image_center_y = self.image_height // 2
        
        # ========== YOLO类别配置 ==========
        self.color_classes = ["red", "blue", "yellow", "green", "white", "brown"]
        
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
        self.acceleration_max_speed = 8.0  # 加速追踪无上限（实际由人速度决定）
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
        self.person_position_history = deque(maxlen=10)
        self.person_moving_direction = None  # 人的移动方向
        self.direction_confidence = 0.0
        
        # ========== 人体朝向检测 ==========
        self.person_facing_drone = False  # 人是否朝向无人机
        self.facing_confidence = 0.0  # 朝向置信度
        self.facing_history = deque(maxlen=15)  # 朝向历史
        self.retreat_triggered = False  # 是否已触发后退
        self.retreat_start_time = 0.0  # 后退开始时间
        self.stable_retreat_duration = 3.0  # 稳定后退持续时间（秒）
        
        # ========== 状态标志 ==========
        self.tracking_mode = "IDLE"  # 追踪模式：STABLE/ACCELERATING/DECELERATING/RETREAT
        self.lost_target_count = 0
        self.lost_threshold = 5  # 丢失5帧才触发搜索
        self.last_u = 0
        self.last_v = 0
        self.last_box_size = 0
        
        # ========== 目标锁定 ==========
        self.locked_target_color = None
        self.locked_target_position = None  # 锁定目标的位置（用于区分同颜色不同人）
        self.target_lock_start_time = None
        self.target_lock_duration = 15.0
        self.lock_position_tolerance = 100  # 位置容差（像素）
        
        # ========== 速度控制 ==========
        self.x_velocity = 0.0
        self.y_velocity = 0.0
        self.z_velocity = 0.0
        self.yaw_rate = 0.0
        self.tracking_count = 0
        
        # ========== 高度保持 ==========
        self.target_height = self.flight_height
        self.height_tolerance = 0.8
        self.height_kp = 0.6
        self.max_z_velocity = 0.8
        
        # ========== ROS通信 ==========
        self.cmd_vel_pub = rospy.Publisher(
            f'/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd_vel_flu',
            Twist, queue_size=10)
        self.cmd_pub = rospy.Publisher(
            f'/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd',
            String, queue_size=10)
        self.mount_control_pub = rospy.Publisher(
            f'/{self.vehicle_type}_{self.vehicle_id}/mavros/mount_control/command',
            MountControl, queue_size=10)
        
        rospy.Subscriber(f'/{self.vehicle_type}_{self.vehicle_id}/mavros/state',
                        State, self.state_callback)
        rospy.Subscriber(f'/{self.vehicle_type}_{self.vehicle_id}/mavros/local_position/pose',
                        PoseStamped, self.pose_callback)
        rospy.Subscriber(f'/xtdrone/{self.vehicle_type}_{self.vehicle_id}/image_raw',
                        Image, self.image_callback)
        rospy.Subscriber('/yolov11/darknet_ros/bounding_boxes',
                        BoundingBoxes, self.detection_callback)
        
        rospy.loginfo("✅ 优化版控制器初始化完成")
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)
    
    def state_callback(self, msg):
        self.current_state = msg
    
    def pose_callback(self, msg):
        self.current_pose = msg
        if self.takeoff_position is None:
            self.takeoff_position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
            rospy.loginfo(f"📍 起飞位置: {self.takeoff_position}")
    
    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn(f"图像转换失败: {e}")
    
    def detection_callback(self, msg):
        self.detections = msg.bounding_boxes
    
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
        
        # 转换为米/秒（50Hz → ×50）
        speed = -avg_change_per_frame * 50.0  # 负号：距离变小=人加速远离
        
        # 限制合理范围（0-10m/s）
        speed = max(0, min(10.0, speed))
        
        return speed
    
    def update_person_world_position(self, u, box_size):
        """
        更新人在世界坐标系中的位置，用于判断移动方向
        """
        if self.current_pose is None:
            return
        
        estimated_distance = self.estimate_distance_from_box_size(box_size)
        lateral_offset = (u / 320.0) * estimated_distance * math.tan(math.radians(30))
        
        drone_x = self.current_pose.pose.position.x
        drone_y = self.current_pose.pose.position.y
        
        from tf.transformations import euler_from_quaternion
        orientation = self.current_pose.pose.orientation
        _, _, drone_yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        
        person_x = drone_x + estimated_distance * math.cos(drone_yaw) - lateral_offset * math.sin(drone_yaw)
        person_y = drone_y + estimated_distance * math.sin(drone_yaw) + lateral_offset * math.cos(drone_yaw)
        
        self.person_position_history.append({
            'time': rospy.Time.now().to_sec(),
            'pos': (person_x, person_y)
        })
        
        # 计算移动方向
        if len(self.person_position_history) >= 5:
            oldest = self.person_position_history[0]
            newest = self.person_position_history[-1]
            
            dx = newest['pos'][0] - oldest['pos'][0]
            dy = newest['pos'][1] - oldest['pos'][1]
            
            distance_moved = math.sqrt(dx**2 + dy**2)
            time_elapsed = newest['time'] - oldest['time']
            
            if distance_moved > 0.5 and time_elapsed > 0.1:
                self.person_moving_direction = math.atan2(dy, dx)
                self.direction_confidence = min(1.0, distance_moved / 2.0)
    
    def is_following_behind(self):
        """
        判断是否跟在人背后
        返回：True=跟在背后，False=需要调整位置
        """
        if self.person_moving_direction is None or self.direction_confidence < 0.3:
            return True  # 方向未知，假设正常
        
        if self.current_pose is None:
            return True
        
        from tf.transformations import euler_from_quaternion
        orientation = self.current_pose.pose.orientation
        _, _, drone_yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        
        # 计算角度差
        angle_diff = abs(self.person_moving_direction - drone_yaw)
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        angle_diff = abs(angle_diff)
        
        # 小于45度认为跟在背后
        return angle_diff < math.radians(45)
    
    def detect_person_facing_drone(self, u, v, box_size, estimated_distance):
        """
        检测人体是否朝向无人机
        基于多个指标综合判断：
        1. 人在屏幕中央位置
        2. box_size快速增大（人接近）
        3. 人在屏幕下方（正面视角）
        4. 距离变化趋势
        """
        current_time = rospy.Time.now().to_sec()
        
        # 指标1：人在屏幕中央（横向居中，纵向偏下）
        in_center_x = abs(u) < 120  # 横向居中
        in_lower_center = v > self.image_height * 0.4 and v < self.image_height * 0.8  # 纵向偏下但不过分
        
        # 指标2：box_size快速增大（人快速接近）
        size_growing_fast = False
        if len(self.size_history) >= 8:
            recent_avg = sum(list(self.size_history)[-4:]) / 4.0
            previous_avg = sum(list(self.size_history)[-8:-4]) / 4.0
            if previous_avg > 0:
                size_growth_rate = (recent_avg - previous_avg) / previous_avg
                size_growing_fast = size_growth_rate > 0.08  # 8%增长率
        
        # 指标3：距离快速减小
        distance_decreasing = False
        if len(self.velocity_history) >= 6:
            recent_distances = list(self.velocity_history)[-3:]
            previous_distances = list(self.velocity_history)[-6:-3]
            recent_avg = sum(recent_distances) / len(recent_distances)
            previous_avg = sum(previous_distances) / len(previous_distances)
            distance_decreasing = (previous_avg - recent_avg) > 0.3  # 距离减小超过0.3米
        
        # 指标4：人在合理距离范围内（2-4米）
        in_reasonable_distance = 2.0 < estimated_distance < 4.0
        
        # 综合判断
        facing_score = 0.0
        if in_center_x:
            facing_score += 0.3
        if in_lower_center:
            facing_score += 0.2
        if size_growing_fast:
            facing_score += 0.3
        if distance_decreasing:
            facing_score += 0.2
        if in_reasonable_distance:
            facing_score += 0.1
        
        # 更新朝向历史
        self.facing_history.append(facing_score)
        
        # 计算平均朝向置信度
        if len(self.facing_history) >= 5:
            avg_facing_score = sum(list(self.facing_history)[-5:]) / 5.0
            self.facing_confidence = avg_facing_score
            
            # 阈值判断：综合得分>0.6认为朝向无人机
            self.person_facing_drone = avg_facing_score > 0.6
        else:
            self.person_facing_drone = False
            self.facing_confidence = 0.0
        
        return self.person_facing_drone, self.facing_confidence
    
    def compute_height_control(self):
        """高度保持"""
        if self.current_pose is None:
            return 0.0
        
        current_height = self.current_pose.pose.position.z
        height_error = self.target_height - current_height
        
        if abs(height_error) < self.height_tolerance:
            return 0.0
        
        z_velocity = self.height_kp * height_error
        z_velocity = max(-self.max_z_velocity, min(self.max_z_velocity, z_velocity))
        
        return z_velocity
    
    def publish_cmd_vel(self, vx, vy, vz, yaw_rate):
        """发布速度命令"""
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = vz if vz != 0 else self.compute_height_control()
        twist.angular.z = yaw_rate
        self.cmd_vel_pub.publish(twist)
    
    def send_command(self, cmd_str):
        """发送字符串命令"""
        cmd_msg = String()
        cmd_msg.data = cmd_str
        self.cmd_pub.publish(cmd_msg)
        rospy.loginfo(f"📤 发送命令: {cmd_str}")
    
    def control_gimbal(self, pitch, roll, yaw):
        """控制云台"""
        msg = MountControl()
        msg.header = std_msgs.msg.Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.mode = 2
        msg.pitch = pitch
        msg.roll = roll
        msg.yaw = yaw
        self.mount_control_pub.publish(msg)
    
    def get_best_color_target(self):
        """选择最佳颜色目标"""
        if self.detections is None or self.current_image is None:
            return None
        
        candidates = []
        
        for box in self.detections:
            detected_class = box.Class.lower()
            if detected_class not in self.color_classes:
                continue
            
            center_x = (box.xmin + box.xmax) / 2
            center_y = (box.ymin + box.ymax) / 2
            box_size = (box.xmax - box.xmin) * (box.ymax - box.ymin)
            
            if box_size < 200:
                continue
            
            distance_to_center = abs(center_x - self.image_center_x)
            
            candidates.append({
                'center_x': center_x,
                'center_y': center_y,
                'size': box_size,
                'distance_to_center': distance_to_center,
                'color': detected_class,
                'probability': box.probability
            })
        
        if len(candidates) == 0:
            return None
        
        # 目标锁定机制
        if self.locked_target_color is not None:
            current_time = rospy.Time.now().to_sec()
            lock_elapsed = current_time - self.target_lock_start_time if self.target_lock_start_time else 0
            
            if lock_elapsed < self.target_lock_duration:
                # 第一步：筛选相同颜色的候选
                locked_candidates = [c for c in candidates if c['color'] == self.locked_target_color]
                
                if len(locked_candidates) > 0:
                    # 第二步：如果有锁定位置，选择最接近锁定位置的目标
                    if self.locked_target_position is not None:
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
                        
                        # 如果最接近的候选在容差范围内，只追踪它
                        if min_distance < self.lock_position_tolerance:
                            candidates = [closest_candidate]
                        else:
                            # 否则追踪所有同颜色目标（可能人移动太快或切换了）
                            candidates = locked_candidates
                    else:
                        # 没有位置信息，追踪所有同颜色目标
                        candidates = locked_candidates
                else:
                    # 锁定目标这一帧没检测到，返回None（由track_target处理短暂丢失）
                    return None
            else:
                # 锁定时间到，解锁
                self.locked_target_color = None
                self.locked_target_position = None
                self.target_lock_start_time = None
        
        # 优先级：越靠近中间且越近（box_size越大）的先追踪
        # 归一化处理，让两个因素权重平衡
        if len(candidates) > 0:
            max_size = max([c['size'] for c in candidates])
            min_size = min([c['size'] for c in candidates])
            max_dist = max([c['distance_to_center'] for c in candidates])
            
            for c in candidates:
                # 归一化 size：越大（越近）分数越高，范围[0, 1]
                if max_size > min_size:
                    size_score = (c['size'] - min_size) / (max_size - min_size)
                else:
                    size_score = 1.0
                
                # 归一化 distance_to_center：越小（越居中）分数越高，范围[0, 1]
                if max_dist > 0:
                    center_score = 1.0 - (c['distance_to_center'] / max_dist)
                else:
                    center_score = 1.0
                
                # 综合评分：距离权重60%，居中权重40%
                c['score'] = size_score * 0.6 + center_score * 0.4
        
        best = max(candidates, key=lambda x: x['score'])
        self.target_history.append(best)
        
        # 历史平滑
        if len(self.target_history) >= 5:
            avg_x = sum([t['center_x'] for t in self.target_history]) / len(self.target_history)
            avg_y = sum([t['center_y'] for t in self.target_history]) / len(self.target_history)
            avg_size = sum([t['size'] for t in self.target_history]) / len(self.target_history)
            best['center_x'] = avg_x
            best['center_y'] = avg_y
            best['size'] = avg_size
        
        return best
    
    def track_target(self):
        """
        统一的追踪逻辑
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
            
            # 确认丢失（第5帧）：判断搜索策略
            if self.lost_target_count == 5:
                self.handle_target_lost()
            
            # 执行搜索
            self.execute_search_strategy()
            return
        
        # ==================== 重新捕获目标 ====================
        if self.lost_target_count > 0:
            color_name = target['color'].upper()
            rospy.loginfo(f"✅ 重新捕获目标 [{color_name}]！（丢失{self.lost_target_count}帧）")
            self.lost_target_count = 0
            # 不强制设置模式，让后续逻辑根据距离自动判断
        
        self.tracking_count += 1
        
        # ==================== 目标锁定 ====================
        target_color = target['color']
        target_center_x = target['center_x']
        target_center_y = target['center_y']
        current_time = rospy.Time.now().to_sec()
        
        if self.locked_target_color is None:
            # 首次锁定：记录颜色和位置
            self.locked_target_color = target_color
            self.locked_target_position = (target_center_x, target_center_y)
            self.target_lock_start_time = current_time
            rospy.loginfo(f"🔒 锁定目标: {target_color.upper()} 位置({target_center_x:.0f},{target_center_y:.0f}) (持续{self.target_lock_duration:.0f}秒)")
        else:
            # 持续追踪：更新锁定位置（跟随目标移动）
            self.locked_target_position = (target_center_x, target_center_y)
            
            # 显示锁定状态（每3秒显示一次）
            if self.tracking_count % 150 == 0:
                lock_elapsed = current_time - self.target_lock_start_time
                lock_remaining = max(0, self.target_lock_duration - lock_elapsed)
                if lock_remaining > 0:
                    rospy.loginfo(f"🔒 目标锁定中: {self.locked_target_color.upper()} (剩余{lock_remaining:.1f}秒)")
                else:
                    rospy.loginfo(f"✅ 目标锁定已完成，可自由切换目标")
        
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
        
        # 更新人的位置和方向
        self.update_person_world_position(u, box_size)
        
        # 判断是否跟在背后
        is_behind = self.is_following_behind()
        
        # 检测人体朝向
        person_facing, facing_conf = self.detect_person_facing_drone(u, v, box_size, estimated_distance)
        
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
        is_person_accelerating = self.person_speed > 1.5  # 人速度>1.5m/s认为在加速
        is_person_fast = self.person_speed > 2.5  # 人速度>2.5m/s认为很快
        
        # 判断人在屏幕位置
        in_lower = v > self.image_height * 0.65  # 下方
        in_center_x = abs(u) < 150  # 横向中间
        
        # 判断人是否朝向无人机快速接近（更严格的判断）
        is_approaching_fast = False
        if len(self.size_history) >= 10:
            recent_avg = sum(list(self.size_history)[-5:]) / 5.0
            previous_avg = sum(list(self.size_history)[-10:-5]) / 5.0
            size_change_rate = (recent_avg - previous_avg) / previous_avg if previous_avg > 0 else 0
            # 提高阈值：增长率 > 15% 且 人在中央 且 朝向无人机
            is_approaching_fast = (size_change_rate > 0.15 and in_center_x and person_facing)
        
        # ==================== 2. 模式切换逻辑 ====================
        current_time = rospy.Time.now().to_sec()
        
        # 检查是否在稳定后退期间
        in_stable_retreat = (self.retreat_triggered and 
                           (current_time - self.retreat_start_time) < self.stable_retreat_duration)
        
        # 优先：人朝向无人机快速接近 → 快速后退
        if is_approaching_fast and distance_level not in ["VERY_FAR", "FAR"]:
            if not self.retreat_triggered:
                self.retreat_triggered = True
                self.retreat_start_time = current_time
                rospy.loginfo(f"🚨 检测到人朝向无人机快速接近！开始稳定后退")
            self.tracking_mode = "RETREAT_FAST"
        
        # 稳定后退模式：人在中央且朝向无人机，持续后退保持视野
        elif (person_facing and in_center_x and 
              distance_level in ["VERY_CLOSE", "CLOSE"] and 
              facing_conf > 0.7):
            if not self.retreat_triggered:
                self.retreat_triggered = True
                self.retreat_start_time = current_time
                rospy.loginfo(f"⬅️ 人在中央朝向无人机，开始稳定后退")
            self.tracking_mode = "STABLE_RETREAT"
        
        # 后退模式：人在下方且距离很近（传统条件）
        elif in_lower and distance_level in ["VERY_CLOSE", "CLOSE"] and not person_facing:
            if not self.retreat_triggered:
                self.retreat_triggered = True
                self.retreat_start_time = current_time
            self.tracking_mode = "RETREAT"
        
        # 稳定后退期间，继续后退
        elif in_stable_retreat:
            if self.tracking_mode in ["RETREAT_FAST", "STABLE_RETREAT", "RETREAT"]:
                # 继续当前后退模式
                pass
            else:
                # 如果距离仍然很近，继续后退
                if distance_level in ["VERY_CLOSE", "CLOSE"]:
                    self.tracking_mode = "STABLE_RETREAT"
                else:
                    # 距离合适，退出后退
                    self.retreat_triggered = False
                    rospy.loginfo(f"✅ 稳定后退完成，距离已合适")
        
        # 退出后退条件：距离足够远或人不再朝向无人机
        elif (self.retreat_triggered and 
              (distance_level in ["IDEAL", "MEDIUM", "FAR"] or 
               not person_facing or facing_conf < 0.4)):
            self.retreat_triggered = False
            rospy.loginfo(f"✅ 退出后退模式，距离={estimated_distance:.1f}m，朝向={person_facing}")
        
        # 减速模式的保持与退出：如果当前在减速，只在距离合适时退出
        elif self.tracking_mode == "DECELERATING":
            if distance_level == "IDEAL":
                # 减速完成，进入稳定追踪
                self.tracking_mode = "STABLE"
            elif distance_level in ["VERY_CLOSE", "CLOSE"]:
                # 距离太近，切换到调整模式
                self.tracking_mode = "ADJUSTING"
            # 否则保持减速状态
        
        # 加速模式：距离>3.5米 或 人在加速且距离>3.2米（但不能打断减速和后退）
        elif (not in_stable_retreat and 
              ((estimated_distance > 3.5) or 
               (is_person_accelerating and distance_level not in ["VERY_CLOSE", "CLOSE", "IDEAL"]))):
            self.tracking_mode = "ACCELERATING"
        
        # 加速切换到减速：距离接近3米
        elif self.tracking_mode == "ACCELERATING" and estimated_distance < 3.2 and estimated_distance > 2.5:
            self.tracking_mode = "DECELERATING"
        
        # 稳定模式：理想距离范围
        elif distance_level == "IDEAL" and not in_stable_retreat:
            self.tracking_mode = "STABLE"
        
        # 默认：根据距离调整
        else:
            if not in_stable_retreat:
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
            alpha = 0.15  # 平滑
            self.x_velocity = alpha * desired_speed + (1 - alpha) * self.x_velocity
        
        elif self.tracking_mode == "ACCELERATING":
            # 加速模式：快速匹配人体速度，不设上限
            # 目标速度 = 人体速度 + 距离误差补偿
            distance_error = estimated_distance - self.ideal_distance
            compensation = distance_error * 0.8  # 距离误差补偿
            
            desired_speed = self.person_speed + compensation + 1.5  # 额外加速
            
            # 平滑滤波（快速响应）
            alpha = 0.70
            self.x_velocity = alpha * desired_speed + (1 - alpha) * self.x_velocity
        
        elif self.tracking_mode == "DECELERATING":
            # 减速模式：平滑减速到匹配人体速度
            # 根据距离动态调整减速幅度（从3.2米减速到3米）
            distance_error = estimated_distance - self.ideal_distance
            
            # 距离越接近理想值，减速越慢
            if distance_error > 0.5:  # 距离还比较远(>3.5m)
                desired_speed = self.person_speed + distance_error * 0.8  # 保持较快速度
                alpha = 0.30
            elif distance_error > 0.2:  # 接近中等距离(3.2-3.5m)
                desired_speed = self.person_speed + distance_error * 0.5
                alpha = 0.25
            else:  # 已经很接近理想距离(<3.2m)
                desired_speed = self.person_speed + 0.2
                alpha = 0.18
            
            # 平滑减速
            self.x_velocity = alpha * desired_speed + (1 - alpha) * self.x_velocity
        
        elif self.tracking_mode == "RETREAT_FAST":
            # 快速后退模式：人朝向无人机快速接近，快速后退避让
            # 根据接近速度动态调整后退速度
            if len(self.size_history) >= 10:
                recent_avg = sum(list(self.size_history)[-5:]) / 5.0
                previous_avg = sum(list(self.size_history)[-10:-5]) / 5.0
                size_change_rate = (recent_avg - previous_avg) / previous_avg if previous_avg > 0 else 0
                # 接近越快，后退越快（最快2.5m/s）
                retreat_speed = -1.5 - size_change_rate * 8.0  # 基础-1.5，增长率10%时额外-0.8
                retreat_speed = max(-2.5, min(-1.2, retreat_speed))
            else:
                retreat_speed = -1.8
            
            # 快速响应，保持人在中心
            if abs(u) < 150:
                self.x_velocity = retreat_speed
            elif u < -150:
                self.x_velocity = retreat_speed * 0.85
                self.yaw_rate = 0.35  # 左转
            else:
                self.x_velocity = retreat_speed * 0.85
                self.yaw_rate = -0.35  # 右转
        
        elif self.tracking_mode == "STABLE_RETREAT":
            # 稳定后退模式：人朝向无人机，持续稳定后退保持视野
            # 根据人体速度和距离动态调整后退速度
            base_retreat_speed = -1.0  # 基础后退速度
            
            # 根据距离调整：越近后退越快
            if distance_level == "VERY_CLOSE":
                base_retreat_speed = -1.5
            elif distance_level == "CLOSE":
                base_retreat_speed = -1.2
            
            # 根据人体速度调整：人越快，后退越快
            if self.person_speed > 2.0:
                base_retreat_speed *= 1.3  # 人很快时，后退加速
            elif self.person_speed > 1.0:
                base_retreat_speed *= 1.1  # 人较快时，轻微加速
            
            # 根据朝向置信度调整：置信度越高，后退越稳定
            if facing_conf > 0.8:
                base_retreat_speed *= 1.1  # 高置信度时，稍微加速后退
            
            self.x_velocity = base_retreat_speed
            
            # 保持人在屏幕中央：根据横向偏移调整yaw
            if abs(u) > 80:
                # 有明显偏移，需要转向
                if u < 0:
                    self.yaw_rate = 0.25  # 左转
                else:
                    self.yaw_rate = -0.25  # 右转
            else:
                # 基本居中，保持当前航向
                self.yaw_rate = 0
        
        elif self.tracking_mode == "RETREAT":
            # 后退模式：人在下方，后退增大视野同时保持居中
            self.x_velocity = -1.2  # 统一后退速度
            # 根据横向偏移量调整yaw，保持人在中心
            if abs(u) > 50:
                # 有明显偏移，需要转向
                if u < 0:
                    self.yaw_rate = 0.35  # 左转
                else:
                    self.yaw_rate = -0.35  # 右转
            else:
                # 基本居中，保持当前航向
                self.yaw_rate = 0
        
        else:  # ADJUSTING
            # 调整模式：根据距离误差调整
            size_error = (self.ideal_box_size - smoothed_box_size) / self.ideal_box_size
            desired_speed = self.kp_distance * size_error + self.person_speed * 0.5
            
            # 快速调整
            alpha = 0.40
            self.x_velocity = alpha * desired_speed + (1 - alpha) * self.x_velocity
        
        # ==================== 4. 转向控制（让人居中） ====================
        if self.tracking_mode not in ["RETREAT", "RETREAT_FAST", "STABLE_RETREAT"]:  # 后退模式单独处理yaw
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
        
        # ==================== 5. 发布命令 ====================
        self.publish_cmd_vel(self.x_velocity, 0, 0, self.yaw_rate)
        
        # ==================== 6. 调试输出 ====================
        if self.tracking_count % 15 == 0:
            mode_icons = {
                "STABLE": "🎯稳定",
                "ACCELERATING": "🚀加速",
                "DECELERATING": "🔽减速",
                "RETREAT": "⬅️后退",
                "RETREAT_FAST": "⚡快退",
                "STABLE_RETREAT": "🔄稳退",
                "ADJUSTING": "🔄调整"
            }
            mode_str = mode_icons.get(self.tracking_mode, self.tracking_mode)
            
            color_name = target['color'].upper()
            behind_str = "✅后方" if is_behind else "⚠️侧面"
            facing_str = f"朝向={person_facing}({facing_conf:.2f})" if person_facing else "背向"
            
            rospy.loginfo(f"[{mode_str}][{color_name}] 距离={estimated_distance:.1f}m({distance_level}) "
                         f"人速={self.person_speed:.1f}m/s {behind_str} {facing_str} | "
                         f"v={self.x_velocity:.2f}m/s yaw={self.yaw_rate:.2f} | "
                         f"横偏={int(u):+4d}px size={int(smoothed_box_size)}px²")
    
    def handle_target_lost(self):
        """
        处理目标丢失：判断搜索策略
        """
        # 优先判断：如果之前在后退模式，继续后退
        if self.tracking_mode in ["RETREAT_FAST", "STABLE_RETREAT"]:
            self.search_strategy = "RETREAT_FAST"
            rospy.logwarn(f"⚠️  后退中丢失目标！→ 继续后退")
            return
        
        # 判断消失位置
        in_left = self.last_u < -180  # 收紧左侧判定
        in_right = self.last_u > 180  # 收紧右侧判定
        in_center_x = abs(self.last_u) < 120  # 收紧中央判定
        in_upper = self.last_v < self.image_height * 0.30  # 收紧上方判定
        in_lower = self.last_v > self.image_height * 0.65
        in_middle_y = self.image_height * 0.35 < self.last_v < self.image_height * 0.60  # 真正的中央区域
        
        # 判断距离
        estimated_distance = self.estimate_distance_from_box_size(self.last_box_size)
        is_far = estimated_distance > 3.5  # 调整为3.5米
        is_close = estimated_distance < 3.5
        
        # 决策优先级：下方 > 左右 > 上方远 > 中央
        if in_lower:
            # 优先级1：下方消失，必须后退
            self.search_strategy = "RETREAT"
            rospy.logwarn(f"⚠️  目标从下方消失！→ 后退搜索")
        elif in_left:
            # 优先级2：左侧消失，左转
            self.search_strategy = "TURN_LEFT"
            rospy.logwarn(f"⚠️  目标从左侧消失！→ 左转搜索")
        elif in_right:
            # 优先级2：右侧消失，右转
            self.search_strategy = "TURN_RIGHT"
            rospy.logwarn(f"⚠️  目标从右侧消失！→ 右转搜索")
        elif in_upper and is_far:
            # 优先级3：上方远处消失，加速追击
            self.search_strategy = "ACCELERATE"
            rospy.logwarn(f"⚠️  目标从上方远处消失！→ 加速追击")
        elif in_center_x and in_middle_y and is_close:
            # 优先级4：真正的中央区域且距离近，可能被遮挡，悬停
            self.search_strategy = "HOVER_THEN_TURN"
            rospy.logwarn(f"⚠️  目标从中央消失！→ 悬停后转向搜索")
        else:
            # 默认：转向搜索（根据上次横向位置）
            if self.last_u < 0:
                self.search_strategy = "TURN_LEFT"
                rospy.logwarn(f"⚠️  目标消失！→ 左转搜索")
            else:
                self.search_strategy = "TURN_RIGHT"
                rospy.logwarn(f"⚠️  目标消失！→ 右转搜索")
    
    def execute_search_strategy(self):
        """
        执行搜索策略
        """
        if not hasattr(self, 'search_strategy'):
            self.search_strategy = "HOVER"
        
        if self.search_strategy == "RETREAT_FAST":
            # 快速后退：人快速接近导致丢失，持续快速后退
            if self.lost_target_count <= 100:
                self.publish_cmd_vel(-2.0, 0, 0, 0)  # 快速后退
            elif self.lost_target_count <= 180:
                self.publish_cmd_vel(0, 0, 0, 0)  # 悬停观察
            else:
                self._return_to_patrol()
        
        elif self.search_strategy == "TURN_LEFT":
            # 左转搜索
            if self.lost_target_count <= 100:
                self.publish_cmd_vel(0.3, 0, 0, 0.8)  # 慢速前进+左转
            elif self.lost_target_count <= 200:
                self.publish_cmd_vel(0, 0, 0, 0)  # 悬停
            else:
                self._return_to_patrol()
        
        elif self.search_strategy == "TURN_RIGHT":
            # 右转搜索
            if self.lost_target_count <= 100:
                self.publish_cmd_vel(0.3, 0, 0, -0.8)  # 慢速前进+右转
            elif self.lost_target_count <= 200:
                self.publish_cmd_vel(0, 0, 0, 0)  # 悬停
            else:
                self._return_to_patrol()
        
        elif self.search_strategy == "RETREAT":
            # 后退搜索
            if self.lost_target_count <= 80:
                self.publish_cmd_vel(-1.5, 0, 0, 0)  # 后退
            elif self.lost_target_count <= 150:
                self.publish_cmd_vel(0, 0, 0, 0)  # 悬停
            else:
                self._return_to_patrol()
        
        elif self.search_strategy == "ACCELERATE":
            # 加速追击
            if self.lost_target_count <= 120:
                self.publish_cmd_vel(3.5, 0, 0, 0)  # 高速前进
            elif self.lost_target_count <= 200:
                self.publish_cmd_vel(0, 0, 0, 0)  # 悬停
            else:
                self._return_to_patrol()
        
        elif self.search_strategy == "HOVER_THEN_TURN":
            # 悬停后转向：先悬停等待，如果没找到就转向搜索
            if self.lost_target_count <= 100:
                self.publish_cmd_vel(0, 0, 0, 0)  # 悬停2秒
            elif self.lost_target_count <= 160:
                # 悬停后没找到，开始转向搜索（根据上次位置）
                if self.last_u < 0:
                    self.publish_cmd_vel(0.3, 0, 0, 0.8)  # 左转
                else:
                    self.publish_cmd_vel(0.3, 0, 0, -0.8)  # 右转
            elif self.lost_target_count <= 210:
                self.publish_cmd_vel(0, 0, 0, 0)  # 悬停观察
            else:
                self._return_to_patrol()
        
        else:  # HOVER (兜底)
            # 悬停等待
            if self.lost_target_count <= 80:
                self.publish_cmd_vel(0, 0, 0, 0)  # 悬停
            elif self.lost_target_count <= 180:
                # 悬停后转向搜索
                if self.last_u < 0:
                    self.publish_cmd_vel(0.3, 0, 0, 0.6)
                else:
                    self.publish_cmd_vel(0.3, 0, 0, -0.6)
            else:
                self._return_to_patrol()
    
    def _return_to_patrol(self):
        """恢复巡航"""
        rospy.loginfo(f"🔍 目标丢失过久，恢复巡航模式")
        
        # 重置锁定状态
        self.locked_target_color = None
        self.locked_target_position = None
        self.target_lock_start_time = None
        
        # 重置后退状态
        self.retreat_triggered = False
        self.retreat_start_time = 0.0
        self.person_facing_drone = False
        self.facing_confidence = 0.0
        self.facing_history.clear()
        
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        min_dist = float('inf')
        nearest_index = 0
        
        for i, wp in enumerate(self.waypoints):
            dist = math.sqrt((current_x - wp[0])**2 + (current_y - wp[1])**2)
            if dist < min_dist:
                min_dist = dist
                nearest_index = i
        
        self.current_waypoint_index = nearest_index
        self.state = "PATROL"
        self.lost_target_count = 0
        self.tracking_mode = "IDLE"
        self.size_history.clear()
        self.velocity_history.clear()
        self.person_position_history.clear()
    
    def patrol(self):
        """巡航模式"""
        if len(self.waypoints) == 0:
            return
        
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        
        target_x, target_y, target_z = self.waypoints[self.current_waypoint_index]
        dx = target_x - current_x
        dy = target_y - current_y
        distance_2d = math.sqrt(dx**2 + dy**2)
        
        if distance_2d < self.waypoint_tolerance:
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
            rospy.loginfo(f"✅ 到达航点 {self.current_waypoint_index + 1}/{len(self.waypoints)}")
            return
        
        speed = min(self.patrol_speed, distance_2d * 0.8)
        speed = max(0.5, speed)
        
        if distance_2d > 0.1:
            vx_world = (dx / distance_2d) * speed
            vy_world = (dy / distance_2d) * speed
        else:
            vx_world = 0
            vy_world = 0
        
        q = self.current_pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        vx_body = vx_world * math.cos(yaw) + vy_world * math.sin(yaw)
        vy_body = -vx_world * math.sin(yaw) + vy_world * math.cos(yaw)
        
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - yaw
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        yaw_rate = yaw_error * 0.8
        yaw_rate = max(-1.0, min(1.0, yaw_rate))
        
        self.publish_cmd_vel(vx_body, vy_body, 0, yaw_rate)
        rospy.loginfo_throttle(3.0, 
            f"[巡航 {self.current_waypoint_index + 1}/{len(self.waypoints)}] 距离={distance_2d:.2f}m")
    
    def control_loop(self, event):
        """主控制循环 - 50Hz"""
        if self.state == "INIT":
            self.publish_cmd_vel(0, 0, 0.1, 0)
            self.state_counter += 1
            
            if self.state_counter == 25:
                self.send_command("OFFBOARD")
            if self.state_counter == 50:
                self.send_command("ARM")
            
            if self.state_counter >= 100:
                if self.current_state.armed and self.current_state.mode == "OFFBOARD":
                    rospy.loginfo("🚀 进入起飞状态")
                    self.state = "TAKEOFF"
                    self.state_counter = 0
                else:
                    if self.state_counter == 100:
                        self.send_command("OFFBOARD")
                    if self.state_counter == 105:
                        self.send_command("ARM")
                    self.state_counter = 75
            
        elif self.state == "TAKEOFF":
            current_height = self.current_pose.pose.position.z
            
            if current_height < self.flight_height - 0.3:
                self.publish_cmd_vel(0, 0, 1.5, 0)
                if self.state_counter % 25 == 0:
                    rospy.loginfo(f"⬆️  起飞中... 高度={current_height:.2f}m/{self.flight_height}m")
                self.state_counter += 1
            else:
                rospy.loginfo(f"✅ 到达目标高度，进入悬停")
                self.state = "HOVER"
                self.state_counter = 0
            
        elif self.state == "HOVER":
            self.publish_cmd_vel(0, 0, 0, 0)
            
            if self.state_counter == 0:
                self.control_gimbal(self.gimbal_pitch, self.gimbal_roll, self.gimbal_yaw)
            
            if self.state_counter % 10 == 0 and self.state_counter < 50:
                self.control_gimbal(self.gimbal_pitch, self.gimbal_roll, self.gimbal_yaw)
            
            self.state_counter += 1
            
            if self.state_counter >= self.hover_duration:
                rospy.loginfo(f"✅ 悬停完成，开始巡航")
                self.state = "PATROL"
                self.state_counter = 0
            
        elif self.state == "PATROL":
            self.state_counter += 1
            target = self.get_best_color_target()
            
            if target is not None:
                rospy.loginfo("=" * 80)
                rospy.loginfo(f"🎯 发现目标 [{target['color'].upper()}]！切换追踪模式")
                rospy.loginfo("=" * 80)
                self.state = "TRACKING"
                self.lost_target_count = 0
                self.tracking_count = 0
                # 初始模式设为ADJUSTING，让后续逻辑根据距离判断
                self.tracking_mode = "ADJUSTING"
                self.size_history.clear()
                self.velocity_history.clear()
                self.person_position_history.clear()
                
                for _ in range(10):
                    self.size_history.append(target['size'])
                    self.velocity_history.append(self.estimate_distance_from_box_size(target['size']))
                
                return
            
            self.patrol()
            
        elif self.state == "TRACKING":
            if self.state_counter % 50 == 0:
                self.control_gimbal(self.gimbal_pitch, self.gimbal_roll, self.gimbal_yaw)
            
            self.track_target()
            self.state_counter += 1
    
    def run(self):
        rospy.loginfo("🟢 优化版追踪系统就绪")
        rospy.spin()

if __name__ == '__main__':
    try:
        node = OptimizedVisualServo()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点关闭")


