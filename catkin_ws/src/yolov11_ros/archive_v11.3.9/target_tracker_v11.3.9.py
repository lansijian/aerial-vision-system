#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Target Tracker for YOLOv11 ROS Multi-Drone System v11.2
目标追踪器 - 基于plan3_optimized视觉伺服算法

主要功能：
1. 接收YOLO检测结果
2. 使用视觉伺服保持人在图像中心
3. 基于目标大小控制前后距离
4. 纯速度控制（不控制z轴）
5. 与多机管理器协调目标分配

作者: 东华大学 Astraeus队
版本: v11.2
日期: 2025-01-16
"""

import rospy
import math
import numpy as np
from threading import Lock
from collections import deque

# ROS消息
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Header
from yolov11_ros_msgs.msg import BoundingBoxes
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo

class TargetTracker:
    """
    目标追踪器类
    
    基于plan3_optimized的视觉伺服算法，实现精确目标追踪
    """
    
    def __init__(self):
        """初始化目标追踪器"""
        # 必须先初始化节点
        rospy.init_node('target_tracker', anonymous=True)
        
        # 获取参数
        self.drone_id = rospy.get_param('~drone_id', 0)
        self.vehicle_type = rospy.get_param('~vehicle_type', 'typhoon_h480')
        self.detection_topic = rospy.get_param('~detection_topic', f'/yolo_detector/drone_{self.drone_id}/detections')
        
        # 追踪参数
        self.lost_target_timeout = rospy.get_param('~lost_target_timeout', 1.5)
        self.min_confidence = rospy.get_param('~min_confidence', 0.7)
        
        # 相机参数（v10.0.1精确值）
        self.camera_fx = 205.47
        self.camera_fy = 205.47
        self.camera_cx = 320.5  # 640/2
        self.camera_cy = 180.5  # 360/2
        self.image_width = 640
        self.image_height = 360
        
        # 云台参数
        self.gimbal_pitch = rospy.get_param('~gimbal_pitch', -45.0)
        self.gimbal_yaw = rospy.get_param('~gimbal_yaw', 0.0)
        self.gimbal_roll = rospy.get_param('~gimbal_roll', 0.0)
        
        # 目标参数
        self.target_height = rospy.get_param('~target_height', 0.9)
        self.assumed_human_height = rospy.get_param('~assumed_human_height', 1.7)
        
        # 坐标系偏移
        self.spawn_offset_x = rospy.get_param('~spawn_offset_x', 0.0)
        self.spawn_offset_y = rospy.get_param('~spawn_offset_y', 0.0)
        
        # ========== 视觉伺服参数（参考plan3） ==========
        # 距离控制
        self.ideal_distance = 3.0
        self.ideal_box_size = 1067  # 3米时的box_size
        self.kp_distance = 2.0      # 距离误差增益
        
        # 横向控制（分段增益）
        self.yaw_gains = {
            'very_close': 0.0012,   # |u| < 80
            'close': 0.0025,        # 80 < |u| < 150
            'medium': 0.0040,       # 150 < |u| < 220
            'far': 0.0060           # |u| > 220
        }
        
        # 速度限制
        self.max_forward_speed = 2.0
        self.max_backward_speed = -1.5
        self.max_lateral_speed = 1.0
        self.max_yaw_rate = 0.5
        
        # 状态变量
        self.current_pose = None
        self.current_detections = []
        self.tracking_target = None
        self.tracking_color = None
        self.last_detection_time = None
        self.flight_mode = "WAYPOINT"  # 默认为航点模式
        self.is_tracking_approved = False
        
        # 历史记录（用于平滑）
        self.size_history = deque(maxlen=10)
        self.velocity_history = deque(maxlen=5)
        
        # 线程锁
        self.lock = Lock()
        
        rospy.loginfo(f"[目标追踪] 初始化 - 无人机{self.drone_id}")
        
        # 订阅话题
        rospy.Subscriber(f'/{self.vehicle_type}_{self.drone_id}/mavros/local_position/pose', 
                        PoseStamped, self._pose_callback)
        rospy.Subscriber(self.detection_topic, BoundingBoxes, self._detection_callback)
        rospy.Subscriber(f'/drone_{self.drone_id}/flight_mode', String, self._flight_mode_callback)
        rospy.Subscriber('/shared/target_assignments', String, self._target_assignment_callback)
        
        # 发布话题
        self.cmd_vel_pub = rospy.Publisher(f'/drone_{self.drone_id}/target_tracker/cmd_vel', 
                                          Twist, queue_size=1)
        self.request_tracking_pub = rospy.Publisher(f'/drone_{self.drone_id}/request_tracking_mode', 
                                                   Bool, queue_size=1)
        self.request_waypoint_pub = rospy.Publisher(f'/drone_{self.drone_id}/request_waypoint_mode', 
                                                   Bool, queue_size=1)
        self.target_claim_pub = rospy.Publisher(f'/drone_{self.drone_id}/target_claim', 
                                              String, queue_size=1)
        
        # ActorInfo发布器（记分系统）
        self.actor_pubs = {
            'blue': rospy.Publisher('/actor_blue_info', ActorInfo, queue_size=1),
            'green': rospy.Publisher('/actor_green_info', ActorInfo, queue_size=1),
            'white': rospy.Publisher('/actor_white_info', ActorInfo, queue_size=1),
            'brown': rospy.Publisher('/actor_brown_info', ActorInfo, queue_size=1),
            'red1': rospy.Publisher('/actor_red1_info', ActorInfo, queue_size=1),
            'red2': rospy.Publisher('/actor_red2_info', ActorInfo, queue_size=1)
        }
        
        # 控制定时器
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self._control_loop)  # 50Hz
        
        rospy.loginfo("[目标追踪] 初始化完成")
    
    # ====================== 回调函数 ======================
    
    def _pose_callback(self, msg):
        """位姿回调"""
        self.current_pose = msg.pose
    
    def _detection_callback(self, msg):
        """YOLO检测回调"""
        with self.lock:
            self.current_detections = msg.bounding_boxes
            if self.current_detections:
                self.last_detection_time = rospy.Time.now()
    
    def _flight_mode_callback(self, msg):
        """飞行模式回调"""
        self.flight_mode = msg.data
        rospy.loginfo_throttle(5.0, f"[目标追踪] 当前飞行模式: {self.flight_mode}")
    
    def _target_assignment_callback(self, msg):
        """目标分配回调"""
        try:
            assignments = eval(msg.data)
            if self.drone_id in assignments:
                assigned_color = assignments[self.drone_id]
                with self.lock:
                    self.tracking_color = assigned_color
                    self.is_tracking_approved = True
            else:
                if self.flight_mode == 'waypoint' and self.is_tracking_approved:
                    with self.lock:
                        self.is_tracking_approved = False
                        self.tracking_target = None
                        self.tracking_color = None
        except:
            pass
    
    # ====================== 目标选择 ======================
    
    def _select_target(self):
        """选择追踪目标"""
        if not self.current_detections:
            return None
        
        # 如果已有追踪目标，优先继续追踪
        if self.tracking_target and self.tracking_color:
            for detection in self.current_detections:
                if detection.Class == self.tracking_color:
                    return detection
        
        # 选择最近的目标（最大的检测框）
        best_target = None
        best_size = 0
        
        for detection in self.current_detections:
            # 过滤低置信度
            if detection.probability < self.min_confidence:
                continue
                
            # 计算box大小
            box_width = detection.xmax - detection.xmin
            box_height = detection.ymax - detection.ymin
            box_size = box_width * box_height
            
            if box_size > best_size:
                best_size = box_size
                best_target = detection
        
        return best_target
    
    def _claim_target(self, color):
        """申请追踪目标"""
        claim_msg = String()
        claim_msg.data = color
        try:
            self.target_claim_pub.publish(claim_msg)
        except rospy.ROSException:
            pass
    
    # ====================== 坐标计算 ======================
    
    def _compute_world_position(self, detection):
        """计算目标的世界坐标（用于ActorInfo）"""
        if not self.current_pose:
            return None, None
        
        # 检测框中心点
        u = (detection.xmin + detection.xmax) / 2.0
        v = (detection.ymin + detection.ymax) / 2.0
        
        # 简化的世界坐标估算
        # 基于无人机位置和检测框大小
        drone_x = self.current_pose.position.x
        drone_y = self.current_pose.position.y
        
        # 根据box大小估算距离
        box_width = detection.xmax - detection.xmin
        box_height = detection.ymax - detection.ymin
        box_size = box_width * box_height
        
        # 估算距离
        if box_size > 50:
            estimated_distance = math.sqrt(self.ideal_box_size / box_size) * self.ideal_distance
        else:
            estimated_distance = 10.0
        
        # 简化的位置估算（假设无人机朝向目标）
        # 获取无人机偏航角
        q = self.current_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # 计算目标位置
        actor_x = drone_x + estimated_distance * math.cos(yaw)
        actor_y = drone_y + estimated_distance * math.sin(yaw)
        
        # 应用spawn偏移
        actor_x += self.spawn_offset_x
        actor_y += self.spawn_offset_y
        
        return actor_x, actor_y
    
    # ====================== 视觉伺服控制 ======================
    
    def _compute_tracking_velocity(self, target):
        """
        计算追踪速度（基于plan3_optimized视觉伺服）
        保持人在图像中心，不控制z轴
        """
        cmd_vel = Twist()
        
        if not self.current_pose:
            return cmd_vel
        
        # 目标中心像素
        u = (target.xmax + target.xmin) / 2.0
        v = (target.ymax + target.ymin) / 2.0
        
        # 图像中心
        u_center = self.image_width / 2.0
        v_center = self.image_height / 2.0
        
        # 像素偏移
        u_offset = u - u_center
        v_offset = v - v_center
        
        # 目标大小
        box_width = target.xmax - target.xmin
        box_height = target.ymax - target.ymin
        box_size = box_width * box_height
        
        # 更新历史
        self.size_history.append(box_size)
        smoothed_box_size = sum(self.size_history) / len(self.size_history)
        
        # ========== 前后速度控制（基于目标大小） ==========
        # 距离误差
        size_error = (self.ideal_box_size - smoothed_box_size) / self.ideal_box_size
        
        # P控制
        forward_speed = self.kp_distance * size_error
        
        # 速度限制
        cmd_vel.linear.x = max(self.max_backward_speed, 
                              min(self.max_forward_speed, forward_speed))
        
        # ========== 横向速度和偏航控制（保持目标居中） ==========
        abs_u = abs(u_offset)
        
        # 分段增益选择
        if abs_u < 80:
            yaw_gain = self.yaw_gains['very_close']
        elif abs_u < 150:
            yaw_gain = self.yaw_gains['close']
        elif abs_u < 220:
            yaw_gain = self.yaw_gains['medium']
        else:
            yaw_gain = self.yaw_gains['far']
        
        # 偏航控制
        yaw_rate = -u_offset * yaw_gain
        cmd_vel.angular.z = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_rate))
        
        # 横向速度（辅助居中）
        lateral_gain = 0.002
        cmd_vel.linear.y = -u_offset * lateral_gain
        cmd_vel.linear.y = max(-self.max_lateral_speed, 
                              min(self.max_lateral_speed, cmd_vel.linear.y))
        
        # ========== z轴速度（不控制） ==========
        cmd_vel.linear.z = 0.0
        
        # ========== 速度平滑 ==========
        if len(self.velocity_history) > 0:
            alpha = 0.3  # 30%新 + 70%旧
            last_vel = self.velocity_history[-1]
            cmd_vel.linear.x = alpha * cmd_vel.linear.x + (1 - alpha) * last_vel['x']
            cmd_vel.linear.y = alpha * cmd_vel.linear.y + (1 - alpha) * last_vel['y']
            cmd_vel.angular.z = alpha * cmd_vel.angular.z + (1 - alpha) * last_vel['yaw']
        
        # 保存历史
        self.velocity_history.append({
            'x': cmd_vel.linear.x,
            'y': cmd_vel.linear.y,
            'yaw': cmd_vel.angular.z
        })
        
        return cmd_vel
    
    # ====================== 控制循环 ======================
    
    def _control_loop(self, event):
        """主控制循环"""
        if rospy.is_shutdown():
            return
        
        cmd_vel = Twist()
        
        with self.lock:
            # 持续检测目标并发布坐标信息
            target = self._select_target()
            
            if target and target.probability > self.min_confidence:
                # 无论什么模式，检测到目标就发布ActorInfo坐标信息（记分系统需要）
                actor_x, actor_y = self._compute_world_position(target)
                if actor_x is not None and actor_y is not None:
                    self._publish_actor_info(actor_x, actor_y, target.Class)
                    rospy.loginfo_throttle(2.0, 
                        f"[目标追踪] 发布{target.Class}色目标坐标: ({actor_x:.2f}, {actor_y:.2f})")
            
            # 在航点模式下检测目标
            if self.flight_mode == "WAYPOINT":
                if target and target.probability > self.min_confidence:
                    # 发现目标，请求切换到追踪模式
                    self.tracking_target = target
                    self.tracking_color = target.Class
                    
                    # 立即请求追踪模式
                    request_msg = Bool()
                    request_msg.data = True
                    try:
                        self.request_tracking_pub.publish(request_msg)
                    except rospy.ROSException:
                        pass
                    
                    # 申请目标
                    self._claim_target(target.Class)
                    
                    rospy.loginfo(f"[目标追踪] 发现{target.Class}色目标，请求切换到追踪模式")
                    rospy.loginfo(f"[目标追踪] 发布追踪请求到: /drone_{self.drone_id}/request_tracking_mode")
                
                # 航点模式下不发布速度命令，直接返回
                return
                
            elif self.flight_mode != "TRACKING":
                # 非航点也非追踪模式，不处理
                return
            
            # 追踪模式 - 计算并发布速度命令
            if target:
                # 有目标
                self.tracking_target = target
                self.tracking_color = target.Class
                self.last_detection_time = rospy.Time.now()
                
                # 计算追踪速度
                cmd_vel = self._compute_tracking_velocity(target)
                rospy.loginfo_throttle(2.0, 
                    f"[目标追踪] 追踪{target.Class}色目标 - "
                    f"速度: vx={cmd_vel.linear.x:.2f} vy={cmd_vel.linear.y:.2f} "
                    f"vz={cmd_vel.linear.z:.2f} yaw={cmd_vel.angular.z:.2f}")
                
            else:
                # 目标丢失
                if self.last_detection_time:
                    lost_time = (rospy.Time.now() - self.last_detection_time).to_sec()
                    
                    if lost_time > self.lost_target_timeout:
                        # 超时，请求返回航点模式
                        self.tracking_target = None
                        self.tracking_color = None
                        self.is_tracking_approved = False
                        
                        request_msg = Bool()
                        request_msg.data = True
                        try:
                            self.request_waypoint_pub.publish(request_msg)
                        except rospy.ROSException:
                            pass
                        
                        rospy.loginfo("[目标追踪] 目标丢失超时，返回航点模式")
        
        # 发布速度指令（仅在追踪模式）
        if self.flight_mode == "TRACKING":
            try:
                self.cmd_vel_pub.publish(cmd_vel)
            except rospy.ROSException:
                pass
    
    def _publish_actor_info(self, x, y, color):
        """发布ActorInfo消息"""
        actor_info = ActorInfo()
        actor_info.x = float(x)
        actor_info.y = float(y)
        actor_info.cls = color
        
        try:
            if color == 'red':
                self.actor_pubs['red1'].publish(actor_info)
                self.actor_pubs['red2'].publish(actor_info)
            elif color in self.actor_pubs:
                self.actor_pubs[color].publish(actor_info)
        except rospy.ROSException:
            pass


def main():
    """主函数"""
    try:
        tracker = TargetTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"[目标追踪] 异常: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
