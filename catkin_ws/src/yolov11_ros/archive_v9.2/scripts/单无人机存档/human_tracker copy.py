#!/home/cty/miniconda3/envs/yolov11/bin/python
# -*- coding: utf-8 -*-

"""
人体跟踪节点 - 改进版本
功能：
1. 订阅YOLO检测结果
2. 实时跟踪行人并保持在图像中心
3. 激光测距检测距离并发布到记分系统
4. 强制高度锁定，避免越飞越高问题
5. 正确检测后消除行人模型
"""

import rospy
import numpy as np
import time
import threading
import math
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String, Float32MultiArray
from mavros_msgs.msg import PositionTarget, MountControl
from mavros_msgs.srv import CommandBool, SetMode, MountConfigure
from yolov11_ros_msgs.msg import BoundingBox, BoundingBoxes
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
from sensor_msgs.msg import LaserScan, Range
from pyquaternion import Quaternion

class HumanTracker:
    def __init__(self):
        rospy.init_node('human_tracker', anonymous=True)
        
        # 跟踪状态 - 默认启用
        self.tracking_active = True
        self.find_cnt = 0
        self.find_cnt_last = 0
        self.not_find_time = 0
        self.get_time = False
        
        # 无人机状态
        self.height = 0.0
        self.target_height = 3.0  # 固定目标高度3米
        self.target_set = True  # 强制设定目标高度
        self.theta = math.radians(-45.0)  # 默认云台俯仰角-45度
        self.cam_pose_received = False  # 是否已收到相机姿态
        
        # 高度硬性限制 - 用于测距精度保证
        self.max_height_limit = 4.5  # 绝对最大高度限制（米）
        self.emergency_descent_threshold = 4.0  # 紧急下降阈值（米）
        
        # 激光测距数据
        self.laser_range = None
        self.laser_valid = False
        self.distance_threshold = 3.0  # 检测距离阈值（米）
        
        # 记分系统相关
        self.detected_humans = {}  # 检测到的人体字典 {actor_id: detection_time}
        self.detection_duration = 5.0  # 检测5秒后发送消除信号
        self.human_position = None  # 当前检测到的人体位置
        self.drone_position = None  # 无人机位置
        
        # 高度控制改进 - 强制锁定模式
        self.altitude_lock_enabled = True
        self.altitude_lock_height = 3.0  # 锁定高度
        self.altitude_tolerance = 0.1  # 高度容差
        self.last_height_correction_time = 0.0
        
        # 参数配置 - 基于XTDrone算法
        self.detection_confidence = rospy.get_param("~detection_confidence", 0.3)  # 降低置信度阈值
        self.max_altitude = 3.5  # 最大飞行高度
        self.min_altitude = 2.5  # 最小飞行高度
        self.safe_tracking_altitude = 3.0  # 跟踪时的安全高度
        
        # 相机参数
        self.image_width = rospy.get_param("~image_width", 640)
        self.image_height = rospy.get_param("~image_height", 360)
        self.u_center = self.image_width / 2.0  # 图像中心x
        self.v_center = self.image_height / 2.0  # 图像中心y
        self.fx = rospy.get_param("~fx", 205.46963709898583)  # 焦距x
        self.fy = rospy.get_param("~fy", 205.46963709898583)  # 焦距y
        
        # 控制增益 - XTDrone原始算法
        self.Kp_xy = rospy.get_param("~Kp_xy", 0.5)  # 水平控制增益（与XTDrone一致）
        self.Kp_z = rospy.get_param("~Kp_z", 1.0)   # 高度控制增益（与XTDrone一致）
        
        # 控制方向可配置反转（用于适配不同飞控/中间件坐标约定）
        self.invert_x = rospy.get_param("~invert_x", False)  # 前后方向
        self.invert_y = rospy.get_param("~invert_y", False)  # 左右方向
        self.invert_z = rospy.get_param("~invert_z", False)  # 上下方向
        
        # 记分系统相关
        self.color_classes = set(['blue', 'green', 'white', 'brown', 'red', 'red1', 'red2'])
        
        # 距离融合参数（用于记分系统）
        self.dist_alpha_laser = rospy.get_param("~dist_alpha_laser", 0.7)  # 距离融合-激光平滑
        self.dist_alpha_vision = rospy.get_param("~dist_alpha_vision", 0.35)  # 距离融合-视觉平滑
        self.fused_distance = None
        self.vision_distance_k = 0.5  # 视觉距离估计系数
        
        # 固定高度 - XTDrone算法
        self.fixed_height = 3.0  # 固定3米高度
        
        # 速度限幅（与XTDrone一致）
        self.max_vel = rospy.get_param("~max_vel", 2.0)  # 最大水平速度
        self.max_vel_z = rospy.get_param("~max_vel_z", 1.0)  # Z轴速度限制
        
        # 跟踪计时（用于日志显示）
        self.tracking_start_time = None
        
        # 视觉距离估计（用于记分系统）
        self.target_size_ratio = rospy.get_param("~target_size_ratio", 0.15)  # 期望框高占比15%
        self.target_tracking_distance = 3.0  # 期望跟踪距离（仅用于显示）
        
        # 检测结果
        self.latest_detections = None
        self.human_classes = ['person', 'human']  # 添加可能的类别名称
        self.person_id = rospy.get_param('~person_id', 0)  # 兼容按ID识别人
        
        # 控制指令
        self.twist = Twist()
        self.cmd_string = ""
        
        # 获取vehicle命名空间
        vehicle_ns = rospy.get_param('~vehicle_ns', 'typhoon_h480_0')
        
        # ROS发布者 - XTDrone速度控制
        self.tracking_status_pub = rospy.Publisher('/human_tracker/status', Bool, queue_size=1)
        self.tracking_info_pub = rospy.Publisher('/human_tracker/info', String, queue_size=1)
        
        # XTDrone控制话题（主要）
        self.cmd_vel_pub_xt = rospy.Publisher(f'/xtdrone/{vehicle_ns}/cmd_vel_flu', Twist, queue_size=1)
        self.cmd_pub_xt = rospy.Publisher(f'/xtdrone/{vehicle_ns}/cmd', String, queue_size=1)
        
        # 兼容话题
        self.cmd_vel_pub = rospy.Publisher('/human_tracker/cmd_vel', Twist, queue_size=1)
        self.cmd_pub = rospy.Publisher('/human_tracker/cmd', String, queue_size=1)
        
        # 云台控制
        self.mount_pub = rospy.Publisher(f'/{vehicle_ns}/mavros/mount_control/command', MountControl, queue_size=1)
        
        # 记分系统发布者 - 发布ActorInfo消息来消除行人
        self.actor_blue_pub = rospy.Publisher('/actor_blue_info', ActorInfo, queue_size=1)
        self.actor_green_pub = rospy.Publisher('/actor_green_info', ActorInfo, queue_size=1)
        self.actor_white_pub = rospy.Publisher('/actor_white_info', ActorInfo, queue_size=1)
        self.actor_brown_pub = rospy.Publisher('/actor_brown_info', ActorInfo, queue_size=1)
        self.actor_red1_pub = rospy.Publisher('/actor_red1_info', ActorInfo, queue_size=1)
        self.actor_red2_pub = rospy.Publisher('/actor_red2_info', ActorInfo, queue_size=1)
        
        # ROS订阅者 - 基于XTDrone架构
        detection_topic = rospy.get_param("~detection_topic", "/yolov11/bounding_boxes")
        vehicle_ns = rospy.get_param('~vehicle_ns', 'typhoon_h480_0')
        pose_topic = rospy.get_param("~pose_topic", f"/{vehicle_ns}/mavros/local_position/pose")
        cam_pose_topic = rospy.get_param("~cam_pose_topic", f"/xtdrone/{vehicle_ns}/cam_pose")
        
        self.detection_sub = rospy.Subscriber(detection_topic, BoundingBoxes, self._detection_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self._pose_callback, queue_size=1)
        self.cam_pose_sub = rospy.Subscriber(cam_pose_topic, PoseStamped, self._cam_pose_callback, queue_size=1)
        self.enable_sub = rospy.Subscriber('/human_tracker/enable', Bool, self._enable_callback)
        
        # 订阅激光测距传感器（多种类型支持）
        laser_scan_topic = rospy.get_param("~laser_scan_topic", f"/{vehicle_ns}/scan")
        laser_range_topic = rospy.get_param("~laser_range_topic", f"/{vehicle_ns}/laser_rangefinder/distance")
        
        # 2D激光雷达订阅
        try:
            self.laser_scan_sub = rospy.Subscriber(laser_scan_topic, LaserScan, self._laser_scan_callback, queue_size=1)
            rospy.loginfo(f"📡 订阅2D激光雷达: {laser_scan_topic}")
        except Exception as e:
            rospy.logwarn(f"无法订阅2D激光雷达: {e}")
        
        # 激光测距仪订阅
        try:
            self.laser_range_sub = rospy.Subscriber(laser_range_topic, Range, self._laser_range_callback, queue_size=1)
            rospy.loginfo(f"📡 订阅激光测距仪: {laser_range_topic}")
        except Exception as e:
            rospy.logwarn(f"无法订阅激光测距仪: {e}")
        
        # 线程锁
        self.tracking_lock = threading.Lock()
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("🤖 人体跟踪节点已启动 - 水平速度控制模式")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"📡 检测话题: {detection_topic}")
        rospy.loginfo(f"📡 位置话题: {pose_topic}")
        rospy.loginfo(f"📡 相机姿态话题: {cam_pose_topic}")
        rospy.loginfo(f"📡 控制话题: /xtdrone/{vehicle_ns}/cmd_vel_flu")
        rospy.loginfo(f"🎯 控制模式: 水平速度控制 (XY + 偏航)")
        rospy.loginfo(f"📍 高度控制: 由航点飞行控制器负责维持")
        rospy.loginfo(f"⚙️  控制增益: Kp_xy={self.Kp_xy}")
        rospy.loginfo(f"🚀 最大水平速度: {self.max_vel}m/s")
        rospy.loginfo(f"⚠️  Z速度固定为0，不干预高度控制")
        rospy.loginfo("=" * 60)
        
    def _detection_callback(self, data):
        """YOLO检测结果回调函数 - 改进版本，集成记分系统"""
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

            is_human_by_name = (cls_name in self.human_classes)
            is_human_by_id = (cls_id == self.person_id)

            if (is_human_by_name or is_human_by_id) and prob >= self.detection_confidence:
                human_detected = True
                best_human_target = target
                detected_cls_name = cls_name if cls_name else None
                tag = cls_name if cls_name else f'id:{cls_id}'
                rospy.loginfo_throttle(1.0, f'🎯 发现人体 ({tag}, 置信度: {prob:.3f})')
                break  # 使用第一个符合条件的人体检测
            elif (is_human_by_name or is_human_by_id):
                tag = cls_name if cls_name else f'id:{cls_id}'
                rospy.logwarn_throttle(2.0, f'❌ 人体置信度过低: {tag} ({prob:.3f} < {self.detection_confidence})')
        
        # 记录所有检测结果
        if all_detections:
            rospy.logdebug_throttle(2.0, f"所有检测结果: {', '.join(all_detections)}")
        
        if human_detected and best_human_target:
            # 计算图像坐标
            u = (best_human_target.xmax + best_human_target.xmin) / 2.0
            v = (best_human_target.ymax + best_human_target.ymin) / 2.0
            u_ = u - self.u_center
            v_ = v - self.v_center

            # 记录跟踪开始时间
            if self.tracking_start_time is None:
                self.tracking_start_time = rospy.get_time()
                rospy.loginfo("🎯 开始人体追踪")

            # 更新融合距离
            try:
                self._update_fused_distance(best_human_target, u, v)
            except Exception as e:
                rospy.logwarn_throttle(5.0, f"距离融合更新失败: {e}")

            # 计算速度控制指令（XTDrone算法）
            velocity_cmd = self._compute_velocity_control(u_, v_)
            
            # 设置Twist消息
            self.twist.linear.x = velocity_cmd['x']
            self.twist.linear.y = velocity_cmd['y']
            self.twist.linear.z = velocity_cmd['z']
            self.twist.angular.z = velocity_cmd['yaw_rate']

            self.cmd_string = ''
            self.find_cnt += 1
            self.get_time = False
            
            # 提取值用于日志
            x_velocity = velocity_cmd['x']
            y_velocity = velocity_cmd['y']
            z_velocity = velocity_cmd['z']
            yaw_rate = velocity_cmd['yaw_rate']

            # 发布记分系统信息（持续发布，依赖融合距离）
            self._publish_human_detection(best_human_target, u, v, detected_cls_name)

            # XTDrone算法状态信息
            center_offset = math.sqrt(u_*u_ + v_*v_)
            distance_info = f"激光{self.laser_range:.2f}m" if (self.laser_valid and self.laser_range) else "无激光"
            
            # 跟踪状态
            if center_offset < 15:
                status = "✅居中"
            elif center_offset < 40:
                status = "🔄跟踪"
            else:
                status = "⚡调整"
            
            fused_d = self.fused_distance if self.fused_distance is not None else -1.0
            
            # 偏航状态
            yaw_status = ""
            if abs(yaw_rate) > 0.05:
                yaw_dir = "右转" if yaw_rate > 0 else "左转"
                yaw_status = f" | {yaw_dir}({abs(yaw_rate):.2f}rad/s)"
            
            info_msg = (f"🎯 水平跟踪 | {status}({center_offset:.0f}px) | {distance_info} | "
                       f"高度:{self.height:.2f}m(航点维持){yaw_status} | "
                       f"速度XY({x_velocity:.2f},{y_velocity:.2f}) Z=0.00")
            
            self.tracking_info_pub.publish(String(data=info_msg))
            rospy.loginfo_throttle(1.0, info_msg)
            
        
        # 如果没有检测到人体，重置跟踪计时
        if not human_detected:
            if self.tracking_start_time is not None:
                rospy.loginfo("⚠️ 丢失目标，重置追踪计时")
                self.tracking_start_time = None
            rospy.logdebug_throttle(3.0, f"未检测到人体，当前检测框数: {len(data.bounding_boxes)}, 置信度阈值: {self.detection_confidence}")

    def _compute_velocity_control(self, u_, v_):
        """计算速度控制指令 - XTDrone算法 + 严格高度控制（≥3米）"""
        # 像素速度（XTDrone原始算法）
        u_velocity = -self.Kp_xy * u_
        v_velocity = -self.Kp_xy * v_

        # 相机姿态（俯仰角）
        theta = self.theta if self.cam_pose_received else math.radians(-45.0)

        # 深度估计（XTDrone原始公式）
        sin_theta = math.sin(theta)
        if abs(sin_theta) < 1e-3:
            sin_theta = 1e-3 if sin_theta >= 0.0 else -1e-3
        z_depth = self.height / sin_theta

        # 世界坐标速度（XTDrone原始几何变换）
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

        # 高度控制：严格保证≥3米
        z_velocity = self._compute_strict_altitude_control()

        # 限幅速度
        x_velocity = np.clip(x_velocity, -self.max_vel, self.max_vel)
        y_velocity = np.clip(y_velocity, -self.max_vel, self.max_vel)
        z_velocity = np.clip(z_velocity, -self.max_vel_z, self.max_vel_z)

        # 偏航控制
        yaw_rate = self._compute_yaw_control(u_)

        return {
            'x': x_velocity,
            'y': y_velocity,
            'z': z_velocity,
            'yaw_rate': yaw_rate
        }
    
    def _compute_strict_altitude_control(self):
        """高度控制 - 由航点飞行控制器负责，跟踪时不控制高度
        
        返回值：
            0.0 - 跟踪模式下始终返回0，不干预高度
        """
        # 跟踪时不控制高度，由航点飞行控制器维持
        # 这样可以利用航点飞行的升力速度维持高度，避免冲突
        
        # 仅记录高度信息用于监控
        if self.height > 4.0:
            rospy.logwarn_throttle(2.0, f"⚠️ 高度监控：当前{self.height:.2f}m（由航点飞行控制）")
        
        # 返回0，不控制高度
        return 0.0
    
    def _compute_yaw_control(self, u_offset):
        """计算偏航控制 - 参考multirotor_keyboard_control.py
        使机身转向目标，让目标保持在图像中心
        """
        # 偏航死区：±30像素内不转向，避免抖动
        yaw_deadzone = 30
        if abs(u_offset) < yaw_deadzone:
            return 0.0
        
        # 偏航增益：像素偏移转角速度
        # 参考multirotor_keyboard_control: MAX_ANG_VEL = 3 rad/s
        Kp_yaw = 0.003  # 每像素对应的角速度增益
        max_yaw_rate = 0.8  # 最大角速度 (rad/s)，比keyboard的3小，更平稳
        
        # 右侧为正，需要右转（正角速度）
        # 左侧为负，需要左转（负角速度）
        yaw_rate = Kp_yaw * u_offset
        
        # 限幅
        yaw_rate = np.clip(yaw_rate, -max_yaw_rate, max_yaw_rate)
        
        # 平滑处理，避免突变
        if hasattr(self, '_last_yaw_rate'):
            max_yaw_change = 0.3  # 每次最大0.3rad/s的角速度变化
            yaw_rate = np.clip(yaw_rate,
                              self._last_yaw_rate - max_yaw_change,
                              self._last_yaw_rate + max_yaw_change)
        self._last_yaw_rate = yaw_rate
        
        return yaw_rate

    def _update_fused_distance(self, target, center_u, center_v):
        """融合激光/视觉距离估计，输出 self.fused_distance。
        - 激光有效：指数平滑
        - 激光无：基于bbox高度的视觉估距，指数平滑
        """
        # 激光优先
        if self.laser_valid and self.laser_range and self.laser_range > 0.05:
            if self.fused_distance is None:
                self.fused_distance = float(self.laser_range)
            else:
                a = self.dist_alpha_laser
                self.fused_distance = a * float(self.laser_range) + (1.0 - a) * self.fused_distance
            return

        # 视觉估距（根据bbox高占比）
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
        """位置回调函数 - 保存位置和姿态"""
        self.height = data.pose.position.z
        self.drone_position = data.pose.position
        
        # 获取当前偏航角
        q = Quaternion(data.pose.orientation.w, data.pose.orientation.x,
                      data.pose.orientation.y, data.pose.orientation.z)
        self._current_yaw = q.yaw_pitch_roll[0]  # yaw角
        
        # 强制设置目标高度
        self.target_height = self.fixed_height
        self.target_set = True
    
    def _cam_pose_callback(self, data):
        """相机姿态回调函数 - 基于XTDrone算法"""
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
        """2D激光雷达回调 - 检测前方最近距离"""
        try:
            ranges = np.array(msg.ranges)
            # 过滤无效数据
            valid_ranges = ranges[np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)]
            
            if len(valid_ranges) > 0:
                # 取前方90度范围内的最小距离
                front_angle_range = int(len(ranges) * 0.25)  # 前方90度范围
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
        """激光测距仪回调 - 单点距离测量"""
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
        """发布人体检测信息到记分系统（改进版 - 包含激光测距）"""
        if not self.drone_position:
            return
        
        try:
            # 基于图像位置和无人机位置计算人体位置
            # 使用像素偏移和假定距离计算
            horizontal_offset = (center_u - self.u_center) * 0.01  # 水平偏移
            
            # 使用融合距离或退化到激光/视觉
            if self.fused_distance is not None:
                distance = float(np.clip(self.fused_distance, 0.5, 30.0))
            elif self.laser_valid and self.laser_range:
                distance = float(self.laser_range)
            else:
                bbox_h_ratio = max(0.05, (target.ymax - target.ymin) / float(self.image_height))
                distance = max(2.0, min(12.0, self.target_size_ratio / bbox_h_ratio * 3.0))
            
            # 计算人体世界坐标（简化版本）
            human_x = self.drone_position.x + distance * 0.8  # 前方偏移量
            human_y = self.drone_position.y + horizontal_offset
            
            # 创建ActorInfo消息
            actor_msg = ActorInfo()
            # 根据检测类别映射颜色通道，默认blue
            if detected_cls_name and detected_cls_name.lower() in self.color_classes:
                actor_msg.cls = detected_cls_name.lower()
            else:
                actor_msg.cls = 'blue'
            actor_msg.x = human_x
            actor_msg.y = human_y
            actor_msg.z = 0.0
            
            # ✅ 添加激光测距信息
            if self.laser_valid and self.laser_range:
                actor_msg.distance = float(self.laser_range)
            else:
                actor_msg.distance = distance
            
            # 只发布到一个颜色通道，避免误删多个actor
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
            
            # 更详细的日志，包括激光测距状态
            laser_status = f"激光{self.laser_range:.2f}m" if (self.laser_valid and self.laser_range) else "视觉估计"
            rospy.loginfo_throttle(5.0, f"📡 发布人体检测: 位置({human_x:.2f}, {human_y:.2f}), 距离{distance:.2f}m ({laser_status})")
            
        except Exception as e:
            rospy.logwarn(f"发布人体检测信息失败: {e}")

    def _publish_gimbal(self, pitch_deg=-45.0, yaw_deg=0.0, roll_deg=0.0):
        """发布云台控制指令，保持-45°俯仰角"""
        msg = MountControl()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.mode = 2  # MAV_MOUNT_MODE_MAVLINK_TARGETING
        msg.pitch = pitch_deg
        msg.yaw = yaw_deg
        msg.roll = roll_deg
        self.mount_pub.publish(msg)
    
    def tracking_loop(self):
        """主跟踪循环 - 水平速度控制模式"""
        rate_hz = rospy.get_param('~control_rate_hz', 50)
        rate = rospy.Rate(rate_hz)
        
        rospy.loginfo("🚀 人体跟踪循环已启动（水平速度控制 + 航点高度维持）")
        rospy.loginfo(f"📡 发布频率: {rate_hz}Hz")
        rospy.loginfo(f"📍 发布话题: /xtdrone/{rospy.get_param('~vehicle_ns', 'typhoon_h480_0')}/cmd_vel_flu")
        rospy.loginfo(f"⚠️  高度控制: 由航点飞行控制器负责")
        
        while not rospy.is_shutdown():
            try:
                # 云台控制始终保持-45°
                self._publish_gimbal(pitch_deg=-45.0)
                
                # 检查跟踪状态
                if self.tracking_active:
                    # 持续发布速度指令到XTDrone
                    self.cmd_vel_pub_xt.publish(self.twist)
                    self.cmd_vel_pub.publish(self.twist)
                    
                    # 检查是否丢失目标
                    if self.find_cnt == self.find_cnt_last:
                        if not self.get_time:
                            self.not_find_time = rospy.get_time()
                            self.get_time = True

                        if rospy.get_time() - self.not_find_time > 2.0:
                            # 丢失目标，悬停（速度为0，高度由航点维持）
                            rospy.loginfo_throttle(3.0, "⏰ 目标丢失，悬停")
                            zero_twist = Twist()
                            # 所有速度为0，高度由航点飞行控制器维持
                            zero_twist.linear.x = 0.0
                            zero_twist.linear.y = 0.0
                            zero_twist.linear.z = 0.0
                            zero_twist.angular.z = 0.0
                            self.cmd_vel_pub_xt.publish(zero_twist)
                            self.cmd_vel_pub.publish(zero_twist)
                            self.cmd_string = 'HOVER'
                            self.cmd_pub_xt.publish(String(data=self.cmd_string))
                            self.cmd_pub.publish(String(data=self.cmd_string))
                            self.tracking_status_pub.publish(Bool(data=False))
                    else:
                        # 有新检测，继续跟踪
                        self.get_time = False
                        self.tracking_status_pub.publish(Bool(data=True))
                        rospy.logdebug_throttle(1.0, f"🎯 跟踪: vx={self.twist.linear.x:.3f}, vy={self.twist.linear.y:.3f}, vz={self.twist.linear.z:.3f}, 高度={self.height:.2f}m")
                else:
                    # 跟踪未启用，发布停止状态
                    self.tracking_status_pub.publish(Bool(data=False))
                
                self.find_cnt_last = self.find_cnt
                
            except Exception as e:
                rospy.logerr(f"跟踪循环错误: {e}")
                import traceback
                rospy.logerr(traceback.format_exc())
            
            rate.sleep()
    
    def run(self):
        """运行跟踪器 - 基于XTDrone架构"""
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