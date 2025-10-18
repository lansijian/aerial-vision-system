#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Human Tracker v10.2.0-bidirectional - 人体追踪器（双向追踪版）
核心功能：支持前进和后退双向追踪

优化重点：
1. 简化控制逻辑，去掉复杂状态机
2. 纯距离控制：框大→后退，框小→前进
3. 偏航控制：保持人在图像中心
4. 双向追踪效果一致

作者: 东华大学 Astraeus队
日期: 2025-10-17
版本: v10.2.0-bidirectional
"""

import rospy
import math
import numpy as np
from collections import deque
from geometry_msgs.msg import Twist, PoseStamped
from yolov11_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Bool, String, Int32
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo


class HumanTracker:
    def __init__(self):
        rospy.init_node('human_tracker')
        
        # 基本参数
        self.drone_id = rospy.get_param('~drone_id', 0)
        self.vehicle_type = rospy.get_param('~vehicle_type', 'typhoon_h480')
        self.vehicle_ns = f"{self.vehicle_type}_{self.drone_id}"
        self.detection_topic = rospy.get_param('~detection_topic', f'/yolo_detector/drone_{self.drone_id}/detections')
        
        # 追踪参数
        self.ideal_distance = rospy.get_param('~ideal_tracking_distance', 3.0)
        self.max_speed = rospy.get_param('~max_tracking_speed', 2.0)
        
        # ===== 坐标计算参数 =====
        self.fx = 205.47
        self.fy = 205.47
        self.cx = 320.0
        self.cy = 180.0
        self.image_width = 640
        self.image_height = 360
        
        # 云台角度
        self.gimbal_pitch = rospy.get_param('~gimbal_pitch', -45.0)
        self.gimbal_yaw = rospy.get_param('~gimbal_yaw', 0.0)
        self.gimbal_roll = rospy.get_param('~gimbal_roll', 0.0)
        
        # 目标高度
        self.target_height = rospy.get_param('~target_height', 0.9)
        self.spawn_offset_x = rospy.get_param('~spawn_offset_x', 0.0)
        self.spawn_offset_y = rospy.get_param('~spawn_offset_y', 0.0)
        self.true_human_height = rospy.get_param('/actor_height', 1.78)
        self.assumed_human_height = self.true_human_height
        
        # ===== 追踪控制参数 =====
        self.Kp_yaw = rospy.get_param('~Kp_yaw', 0.003)
        self.Kp_distance = rospy.get_param('~Kp_distance', 2.0)
        self.ideal_box_height = rospy.get_param('~ideal_box_height', 160)
        
        # 速度限制
        self.max_vx = rospy.get_param('~max_vx', 3.0)
        self.max_vy = rospy.get_param('~max_vy', 0.0)
        self.max_vz = rospy.get_param('~max_vz', 0.0)
        self.max_yaw_rate = rospy.get_param('~max_yaw_rate', 0.8)
        
        # 控制频率
        self.control_frequency = rospy.get_param('~control_frequency', 100.0)
        
        # ===== 双向追踪参数（简化版） =====
        # 允许后退追踪
        self.allow_backward = rospy.get_param('~allow_backward', True)
        
        # 后退速度限制（负数表示后退）
        self.max_backward_speed = rospy.get_param('~max_backward_speed', -2.5)
        
        # 纵向位置控制参数（保持人在图像中心）
        self.ideal_v_center = rospy.get_param('~ideal_v_center', 180.0)  # 图像中心纵向位置
        self.Kp_vertical = rospy.get_param('~Kp_vertical', 0.015)  # 纵向位置控制增益（提高）
        self.v_dead_zone = rospy.get_param('~v_dead_zone', 30.0)  # 纵向死区（像素）
        self.v_emergency_threshold = rospy.get_param('~v_emergency_threshold', 250.0)  # 紧急后退阈值
        self.emergency_retreat_speed = rospy.get_param('~emergency_retreat_speed', -3.0)  # 紧急后退速度
        
        # 高度限制参数
        self.max_altitude = rospy.get_param('~max_altitude', 5.5)  # 最大高度限制
        self.altitude_control_enabled = rospy.get_param('~altitude_control_enabled', True)
        
        # 稳定性参数
        self.bbox_smooth_frames = rospy.get_param('~bbox_smooth_frames', 3)
        self.bbox_weights = [0.5, 0.3, 0.2]
        self.dead_zone_px = rospy.get_param('~dead_zone_px', 20)
        self.small_box_threshold = rospy.get_param('~small_box_threshold', 50)
        self.small_box_factor_slope = rospy.get_param('~small_box_factor_slope', 0.003)
        
        # 状态变量
        self.current_pose = None
        self.tracking_active = False
        self.tracking_requested = False
        self.last_detection_time = rospy.Time(0)
        self.lost_count = 0
        self.current_target_color = None
        self.cmd_vel = Twist()
        self.flight_mode = "IDLE"
        self.last_detection = None
        
        # 协调器相关状态（简化版：使用话题名称通信）
        self.locked_colors = set()  # 被其他无人机锁定的颜色
        self.my_locked_color = None  # 本机锁定的颜色
        
        # 双向追踪状态
        self.last_direction = "forward"  # forward or backward
        
        # 坐标平滑
        self.coord_history_x = []
        self.coord_history_y = []
        self.coord_history_size = 5
        
        # 检测框平滑队列
        self.bbox_smooth = deque(maxlen=self.bbox_smooth_frames)
        
        # 订阅者
        self.detection_sub = rospy.Subscriber(
            self.detection_topic, BoundingBoxes, self._detection_callback
        )
        self.pose_sub = rospy.Subscriber(
            f'/{self.vehicle_ns}/mavros/local_position/pose',
            PoseStamped, self._pose_callback
        )
        self.mode_sub = rospy.Subscriber(
            f'/drone_{self.drone_id}/flight_mode',
            String, self._mode_callback
        )
        
        # 协调器订阅者（监听其他无人机的锁定状态）
        self.coordinator_lock_sub = rospy.Subscriber(
            '/coordinator/locked_colors',
            String,
            self._locked_colors_callback
        )
        
        # 发布者
        self.cmd_vel_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/human_tracker/cmd_vel',
            Twist, queue_size=1
        )
        self.tracking_mode_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/request_tracking_mode',
            Bool, queue_size=1
        )
        self.waypoint_mode_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/request_waypoint_mode',
            Bool, queue_size=1
        )
        
        # 协调器发布者（发布本机锁定的颜色）
        self.my_lock_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/locked_color',
            String,
            queue_size=10
        )
        
        # ActorInfo发布者
        self.actor_pubs = {
            'blue': rospy.Publisher('/actor_blue_info', ActorInfo, queue_size=1),
            'green': rospy.Publisher('/actor_green_info', ActorInfo, queue_size=1),
            'white': rospy.Publisher('/actor_white_info', ActorInfo, queue_size=1),
            'brown': rospy.Publisher('/actor_brown_info', ActorInfo, queue_size=1),
            'red1': rospy.Publisher('/actor_red1_info', ActorInfo, queue_size=1),
            'red2': rospy.Publisher('/actor_red2_info', ActorInfo, queue_size=1),
        }
        
        rospy.loginfo("=" * 70)
        rospy.loginfo(f"✅ 无人机{self.drone_id} 追踪器v10.2.0-双向追踪版初始化完成")
        rospy.loginfo(f"   🎯 核心功能: 支持前进和后退双向追踪")
        rospy.loginfo(f"   ⬆️  前进追踪: 框小于{self.ideal_box_height}px → 前进靠近")
        rospy.loginfo(f"   ⬇️  后退追踪: 框大于{self.ideal_box_height}px → 后退保持距离")
        rospy.loginfo(f"   🎯 偏航控制: 保持目标在图像中心")
        rospy.loginfo(f"   📊 速度范围: 前进{self.max_vx}m/s, 后退{self.max_backward_speed}m/s")
        rospy.loginfo("=" * 70)
    
    def _mode_callback(self, msg):
        """飞行模式回调"""
        old_mode = self.flight_mode
        self.flight_mode = msg.data
        
        if old_mode != self.flight_mode:
            rospy.loginfo(f"[DEBUG] 无人机{self.drone_id} 模式切换: {old_mode} → {self.flight_mode}")
            
            if self.flight_mode == "TRACKING":
                self.tracking_active = True
                rospy.loginfo(f"✅ 无人机{self.drone_id} 进入TRACKING模式，开始追踪")
            else:
                self.tracking_active = False
                self.tracking_requested = False
        
        if self.flight_mode != "TRACKING":
            self.cmd_vel = Twist()
            self.cmd_vel_pub.publish(self.cmd_vel)
            # 退出追踪模式时，释放锁定
            if old_mode == "TRACKING" and self.my_locked_color:
                self.my_lock_pub.publish(String(data=""))
                self.my_locked_color = None
    
    def _locked_colors_callback(self, msg):
        """接收其他无人机锁定的颜色列表"""
        # 格式: "drone_0:blue,drone_1:green,drone_2:white"
        self.locked_colors.clear()
        if msg.data:
            for lock in msg.data.split(','):
                if ':' in lock:
                    drone_id, color = lock.split(':')
                    # 排除本机的锁定
                    if int(drone_id.replace('drone_', '')) != self.drone_id:
                        self.locked_colors.add(color)
    
    def _detection_callback(self, msg):
        """处理YOLO检测结果"""
        if not msg.bounding_boxes:
            self.last_detection = None
            self.bbox_smooth.clear()
            if self.flight_mode == "TRACKING":
                self.lost_count += 1
                if self.lost_count > 15:
                    rospy.logwarn(f"⚠️ 无人机{self.drone_id}: 目标丢失，立即恢复航点模式")
                    self._stop_tracking()
            return
        
        # 选择最佳目标（最大框）
        best_target = max(msg.bounding_boxes, 
                         key=lambda b: (b.xmax - b.xmin) * (b.ymax - b.ymin))
        
        # 添加到平滑队列
        self.bbox_smooth.append({
            'u': (best_target.xmax + best_target.xmin) / 2.0,
            'v': (best_target.ymax + best_target.ymin) / 2.0,
            'h': best_target.ymax - best_target.ymin
        })
        
        # 保存检测结果
        self.last_detection = best_target
        self.last_detection_time = rospy.Time.now()
        self.lost_count = 0
        
        # 检查目标是否被其他无人机锁定（红色除外）
        target_color = best_target.Class
        can_track = True
        
        if target_color != 'red' and target_color in self.locked_colors:
            can_track = False
            rospy.loginfo_throttle(5.0, f"⚠️ 无人机{self.drone_id}: {target_color}目标已被其他无人机锁定，跳过")
        
        # 发现新目标且可以追踪
        if can_track and (self.flight_mode != "TRACKING" or self.current_target_color != target_color):
            self.current_target_color = target_color
            self.my_locked_color = target_color
            # 发布锁定信息
            self.my_lock_pub.publish(String(data=target_color))
            self._start_tracking()
        
        # 发布ActorInfo（用于记分）
        if can_track:
            self._publish_actor_info(best_target)
            rospy.loginfo_throttle(5.0, f"[记分] 无人机{self.drone_id} 发布{target_color}目标坐标")
    
    def _compute_tracking_velocity(self):
        """
        双向追踪算法（增强版）
        核心逻辑：
        1. 框大 → 后退（倒车）
        2. 框小 → 前进
        3. 偏航 → 保持人在横向中心
        4. 纵向位置 → 保持人在图像中心（下方→后退，上方→前进）
        """
        if self.last_detection is None or self.current_pose is None:
            self.cmd_vel = Twist()
            return
        
        # ========== 1. 3帧加权平滑 ==========
        if len(self.bbox_smooth) == self.bbox_smooth_frames:
            weights = self.bbox_weights
            u_center = sum(h['u'] * w for h, w in zip(self.bbox_smooth, weights))
            v_center = sum(h['v'] * w for h, w in zip(self.bbox_smooth, weights))
            box_height = sum(h['h'] * w for h, w in zip(self.bbox_smooth, weights))
        else:
            u_center = self.bbox_smooth[-1]['u']
            v_center = self.bbox_smooth[-1]['v']
            box_height = self.bbox_smooth[-1]['h']
        
        # ========== 2. 小框修正 ==========
        box_height_corrected = box_height
        if box_height < self.small_box_threshold:
            small_box_factor = 1.0 + (self.small_box_threshold - box_height) * self.small_box_factor_slope
            box_height_corrected = box_height * small_box_factor
        
        # ========== 3. 偏航控制（保持人在中心）==========
        u_offset = u_center - self.cx
        
        if abs(u_offset) < self.dead_zone_px:
            yaw_rate = 0.0
        else:
            yaw_rate = self.Kp_yaw * u_offset
        
        self.cmd_vel.angular.z = np.clip(yaw_rate, -self.max_yaw_rate, self.max_yaw_rate)
        
        # ========== 4. 前后距离控制（双向 + 纵向位置 + 紧急后退）==========
        v_error = v_center - self.ideal_v_center
        
        # 4.1 紧急后退模式：人在图像最下方
        if v_center > self.v_emergency_threshold:
            # 人太靠近图像下边界，强制紧急后退
            forward_speed = self.emergency_retreat_speed
            direction = "emergency_retreat"
            rospy.logwarn_throttle(0.5, f"⚠️ 紧急后退！人在图像下方 v={v_center:.0f}px")
        else:
            # 4.2 基于框高度的距离控制
            height_error = self.ideal_box_height - box_height_corrected
            speed_from_height = self.Kp_distance * (height_error / self.ideal_box_height)
            
            # 4.3 基于纵向位置的控制（保持人在图像中心）
            speed_from_vertical = 0.0
            
            if abs(v_error) > self.v_dead_zone:
                # v_error > 0: 人在图像下方 → 需要后退
                # v_error < 0: 人在图像上方 → 需要前进
                speed_from_vertical = -self.Kp_vertical * v_error
            
            # 4.4 综合控制：纵向位置优先（权重更高）
            if abs(v_error) > self.v_dead_zone:
                # 纵向偏离较大时，优先纵向控制
                forward_speed = speed_from_vertical * 0.8 + speed_from_height * 0.2
            else:
                # 纵向位置正常时，使用框高度控制
                forward_speed = speed_from_height
            
            # 4.5 速度限制和方向判断
            if abs(forward_speed) < 0.1 and abs(height_error) < self.dead_zone_px:
                # 死区：保持静止
                forward_speed = 0.0
                direction = "hold"
            elif forward_speed > 0:
                # 前进
                forward_speed = min(forward_speed, self.max_vx)
                direction = "forward"
            else:
                # 后退（倒车）
                if self.allow_backward:
                    forward_speed = max(forward_speed, self.max_backward_speed)
                    direction = "backward"
                else:
                    forward_speed = 0.0
                    direction = "blocked"
        
        self.cmd_vel.linear.x = forward_speed
        
        # 横向速度：禁用
        self.cmd_vel.linear.y = 0.0
        
        # ========== 高度控制：防止超过6米 ==========
        if self.altitude_control_enabled and self.current_pose is not None:
            current_altitude = self.current_pose.position.z
            
            if current_altitude > self.max_altitude:
                # 超过最大高度，强制下降
                self.cmd_vel.linear.z = -0.5
                rospy.logwarn_throttle(1.0, f"⚠️ 高度超限{current_altitude:.1f}m > {self.max_altitude}m，强制下降")
            elif current_altitude > self.max_altitude - 0.5:
                # 接近最大高度，缓慢下降
                self.cmd_vel.linear.z = -0.2
            else:
                # 高度正常，保持
                self.cmd_vel.linear.z = 0.0
        else:
            self.cmd_vel.linear.z = 0.0
        
        # ========== 5. 状态日志 ==========
        # 横向对中状态
        abs_u = abs(u_offset)
        if abs_u < self.dead_zone_px:
            yaw_status = "✅居中"
        elif abs_u < 30:
            yaw_status = "🔄微调"
        elif abs_u < 100:
            yaw_status = "🔄调整"
        else:
            yaw_status = "⚠️偏离"
        
        # 纵向位置状态
        v_error = v_center - self.ideal_v_center
        if abs(v_error) < self.v_dead_zone:
            v_status = "✅中心"
        elif v_error > 0:
            v_status = f"⬇️下方+{v_error:.0f}px"
        else:
            v_status = f"⬆️上方{v_error:.0f}px"
        
        # 前后距离状态
        if direction == "emergency_retreat":
            dist_status = "🚨紧急后退"
            self.last_direction = "backward"
        elif direction == "forward":
            dist_status = "⬆️前进"
            self.last_direction = "forward"
        elif direction == "backward":
            dist_status = "⬇️倒车"
            self.last_direction = "backward"
        elif direction == "hold":
            dist_status = "✅保持"
        else:
            dist_status = "🚫禁止后退"
        
        # 获取当前高度
        alt_str = f"alt={self.current_pose.position.z:.1f}m" if self.current_pose else "alt=N/A"
        
        rospy.loginfo_throttle(1.0, 
            f"【{yaw_status}】u={u_offset:+4.1f}px 【{v_status}】h={box_height:3.0f}px | "
            f"【{dist_status}】vx={forward_speed:+.2f} vz={self.cmd_vel.linear.z:+.2f} yaw={yaw_rate:+.3f} | "
            f"{alt_str}")
    
    def _pixel_to_camera_ray(self, u, v):
        """像素坐标 → 相机归一化射线"""
        x_norm = (u - self.cx) / self.fx
        y_norm = (v - self.cy) / self.fy
        z_norm = 1.0
        
        length = math.sqrt(x_norm**2 + y_norm**2 + z_norm**2)
        return np.array([x_norm / length, y_norm / length, z_norm / length])
    
    def _quaternion_to_rotation_matrix(self, q):
        """四元数 → 旋转矩阵"""
        w, x, y, z = q.w, q.x, q.y, q.z
        
        if w < 0:
            w, x, y, z = -w, -x, -y, -z
        
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        if norm > 1e-6:
            w, x, y, z = w/norm, x/norm, y/norm, z/norm
        
        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])
        return R
    
    def _compute_gimbal_rotation(self):
        """计算云台旋转矩阵"""
        pitch_rad = math.radians(self.gimbal_pitch)
        yaw_rad = math.radians(self.gimbal_yaw)
        roll_rad = math.radians(self.gimbal_roll)
        
        R_pitch = np.array([
            [math.cos(pitch_rad), 0, math.sin(pitch_rad)],
            [0, 1, 0],
            [-math.sin(pitch_rad), 0, math.cos(pitch_rad)]
        ])
        
        R_yaw = np.array([
            [math.cos(yaw_rad), -math.sin(yaw_rad), 0],
            [math.sin(yaw_rad), math.cos(yaw_rad), 0],
            [0, 0, 1]
        ])
        
        R_roll = np.array([
            [1, 0, 0],
            [0, math.cos(roll_rad), -math.sin(roll_rad)],
            [0, math.sin(roll_rad), math.cos(roll_rad)]
        ])
        
        return R_yaw @ R_pitch @ R_roll
    
    def _compute_camera_to_enu_rotation(self):
        """计算相机→ENU完整旋转矩阵"""
        if self.current_pose is None:
            return np.eye(3)
        
        R_body_to_enu = self._quaternion_to_rotation_matrix(self.current_pose.orientation)
        
        R_cam_base = np.array([
            [0, 0, 1],
            [1, 0, 0],
            [0, 1, 0]
        ])
        
        R_gimbal = self._compute_gimbal_rotation()
        
        R_cam_to_enu = R_body_to_enu @ R_gimbal @ R_cam_base
        
        return R_cam_to_enu
    
    def _publish_actor_info(self, target):
        """发布ActorInfo"""
        if self.current_pose is None:
            return
        
        # 计算目标在图像中的位置
        u = (target.xmax + target.xmin) / 2.0
        v = (target.ymax + target.ymin) / 2.0
        
        # 基于检测框大小估算距离
        box_width = target.xmax - target.xmin
        box_height = target.ymax - target.ymin
        
        estimated_distance = (self.assumed_human_height * self.fy) / max(box_height, 10)
        estimated_distance = np.clip(estimated_distance, 1.5, 15.0)
        
        # 考虑云台俯角修正
        horizontal_distance = estimated_distance * 0.707
        
        # 计算目标相对无人机的方位角
        u_offset = u - self.cx
        
        horizontal_fov_deg = 60.0
        lateral_angle_offset = (u_offset / self.image_width) * math.radians(horizontal_fov_deg)
        
        # 获取无人机偏航角
        import tf.transformations as tf_trans
        q = self.current_pose.orientation
        _, _, drone_yaw = tf_trans.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # 目标相对于北的角度
        target_bearing = drone_yaw + lateral_angle_offset
        
        # 计算目标世界坐标
        drone_x = self.current_pose.position.x
        drone_y = self.current_pose.position.y
        
        actor_x_local = drone_x + horizontal_distance * math.cos(target_bearing)
        actor_y_local = drone_y + horizontal_distance * math.sin(target_bearing)
        
        # 转换到Gazebo世界坐标
        actor_x = actor_x_local + self.spawn_offset_x
        actor_y = actor_y_local + self.spawn_offset_y
        
        # 坐标平滑
        self.coord_history_x.append(actor_x)
        self.coord_history_y.append(actor_y)
        
        if len(self.coord_history_x) > self.coord_history_size:
            self.coord_history_x.pop(0)
            self.coord_history_y.pop(0)
        
        if len(self.coord_history_x) >= 3:
            actor_x_smoothed = sum(self.coord_history_x) / len(self.coord_history_x)
            actor_y_smoothed = sum(self.coord_history_y) / len(self.coord_history_y)
        else:
            actor_x_smoothed = actor_x
            actor_y_smoothed = actor_y
        
        # 创建并发布ActorInfo
        actor_info = ActorInfo()
        actor_info.x = actor_x_smoothed
        actor_info.y = actor_y_smoothed
        actor_info.cls = target.Class
        
        if len(self.coord_history_x) >= 3:
            rospy.loginfo_throttle(1.0,
                f"[坐标-{target.Class}] 原始:({actor_x:.2f},{actor_y:.2f}) "
                f"平滑:({actor_x_smoothed:.2f},{actor_y_smoothed:.2f}) | "
                f"无人机:({drone_x:.2f},{drone_y:.2f}) 偏航:{math.degrees(drone_yaw):.0f}° | "
                f"距离:{horizontal_distance:.2f}m 方位:{math.degrees(target_bearing):.0f}°")
        else:
            rospy.loginfo_throttle(1.0,
                f"[坐标-{target.Class}] 位置:({actor_x:.2f},{actor_y:.2f}) | "
                f"无人机:({drone_x:.2f},{drone_y:.2f}) 偏航:{math.degrees(drone_yaw):.0f}° | "
                f"距离:{horizontal_distance:.2f}m [收集{len(self.coord_history_x)}/3]")
        
        # 发布到对应话题
        if target.Class == 'red':
            self.actor_pubs['red1'].publish(actor_info)
            self.actor_pubs['red2'].publish(actor_info)
        elif target.Class in self.actor_pubs:
            self.actor_pubs[target.Class].publish(actor_info)
    
    def _start_tracking(self):
        """开始追踪"""
        if self.flight_mode == "TRACKING":
            rospy.loginfo_throttle(5.0, f"[DEBUG] 无人机{self.drone_id}: 已经在TRACKING模式")
            return
        
        if self.tracking_requested:
            rospy.loginfo_throttle(2.0, f"[DEBUG] 无人机{self.drone_id}: 追踪请求已发送，等待模式切换...")
            return
        
        self.tracking_requested = True
        self.tracking_mode_pub.publish(Bool(data=True))
        rospy.loginfo(f"🎯 无人机{self.drone_id}: 发现{self.current_target_color}目标，请求切换到追踪模式")
        rospy.loginfo(f"[DEBUG] 发布追踪请求到话题: /drone_{self.drone_id}/request_tracking_mode")
    
    def _stop_tracking(self):
        """停止追踪"""
        # 释放锁定
        if self.my_locked_color:
            self.my_lock_pub.publish(String(data=""))
            self.my_locked_color = None
        
        self.tracking_active = False
        self.tracking_requested = False
        self.current_target_color = None
        self.waypoint_mode_pub.publish(Bool(data=True))
        self.cmd_vel = Twist()
        self.cmd_vel_pub.publish(self.cmd_vel)
        rospy.loginfo(f"🔄 无人机{self.drone_id}: 目标丢失，请求恢复航点模式")
    
    def _control_loop(self, event):
        """控制循环（100Hz定时器回调）"""
        if self.flight_mode == "TRACKING" and self.last_detection is not None:
            time_since_detection = (rospy.Time.now() - self.last_detection_time).to_sec()
            if time_since_detection > 0.5:
                rospy.logwarn(f"⚠️ 无人机{self.drone_id}: 检测超时，停止追踪")
                self._stop_tracking()
                return
            
            self._compute_tracking_velocity()
            self.cmd_vel_pub.publish(self.cmd_vel)
    
    def _pose_callback(self, msg):
        """更新无人机位姿"""
        self.current_pose = msg.pose
    
    def run(self):
        """主循环"""
        rospy.loginfo("="*70)
        rospy.loginfo(f"🚀 无人机{self.drone_id} 追踪器v10.2.0-双向追踪版开始运行")
        rospy.loginfo(f"   🎯 核心功能: 支持前进和后退双向追踪")
        rospy.loginfo(f"   ⬆️  前进追踪: 框小于{self.ideal_box_height}px → 前进靠近")
        rospy.loginfo(f"   ⬇️  后退追踪: 框大于{self.ideal_box_height}px → 后退保持距离")
        rospy.loginfo(f"   🎯 偏航控制: 保持目标在图像中心")
        rospy.loginfo(f"   📊 速度范围: 前进{self.max_vx}m/s, 后退{self.max_backward_speed}m/s")
        rospy.loginfo("="*70)
        
        control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_frequency),
            self._control_loop
        )
        
        rospy.spin()
        control_timer.shutdown()
        rospy.loginfo("✅ 追踪器停止")


if __name__ == '__main__':
    try:
        tracker = HumanTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass