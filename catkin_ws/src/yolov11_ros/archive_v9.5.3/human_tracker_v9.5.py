#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Human Tracker v9.5 - 简洁版人体追踪器
参考XTDrone的yolo_human_tracking.py设计，保持简洁高效

功能：
1. 订阅YOLO检测结果
2. 选择最佳目标进行追踪
3. 计算追踪速度命令
4. 发布ActorInfo到记分系统

作者: 东华大学 Astraeus队
日期: 2025-10-12
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from yolov11_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Bool, Float64
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
# from sensor_msgs.msg import Range  # 不需要订阅distance


class HumanTracker:
    def __init__(self):
        rospy.init_node('human_tracker')
        
        # 参数
        self.drone_id = rospy.get_param('~drone_id', 0)
        self.vehicle_type = rospy.get_param('~vehicle_type', 'typhoon_h480')
        self.vehicle_ns = f"{self.vehicle_type}_{self.drone_id}"
        self.detection_topic = rospy.get_param('~detection_topic', f'/yolo_detector/drone_{self.drone_id}/detections')
        self.ideal_distance = rospy.get_param('~ideal_tracking_distance', 3.0)
        self.max_speed = rospy.get_param('~max_tracking_speed', 2.0)
        
        # 相机参数
        self.fx = 205.47  # 焦距
        self.fy = 205.47
        self.cx = 320.0  # 图像中心
        self.cy = 240.0
        self.image_width = 640
        self.image_height = 480
        
        # 控制参数（参考XTDrone）
        self.Kp_xy = 0.5  # 水平控制增益
        self.Kp_z = 1.0   # 垂直控制增益
        self.Kp_yaw = 0.002  # 偏航控制增益
        
        # 状态变量
        self.current_height = 3.0
        self.current_pose = None  # 无人机完整位姿
        self.tracking_active = False
        self.last_detection_time = rospy.Time(0)
        self.detection_count = 0
        self.lost_count = 0
        self.current_target_color = None
        
        # 速度命令
        self.cmd_vel = Twist()
        
        # 订阅者
        self.detection_sub = rospy.Subscriber(
            self.detection_topic, BoundingBoxes, self._detection_callback
        )
        self.pose_sub = rospy.Subscriber(
            f'/{self.vehicle_ns}/mavros/local_position/pose',
            PoseStamped, self._pose_callback
        )
        # 不需要订阅distance话题（与原版本一致）
        
        # 发布者
        self.cmd_vel_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/human_tracker/cmd_vel',
            Twist, queue_size=1
        )
        self.tracking_status_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/tracking_request',
            Bool, queue_size=1
        )
        
        # ActorInfo发布者（记分系统需要）
        self.actor_pubs = {}
        for color in ['blue', 'green', 'white', 'brown', 'red']:
            self.actor_pubs[color] = rospy.Publisher(
                f'/actor_{color}_info', ActorInfo, queue_size=1
            )
        # 红色恐怖分子特殊处理
        self.actor_pubs['red1'] = rospy.Publisher('/actor_red1_info', ActorInfo, queue_size=1)
        self.actor_pubs['red2'] = rospy.Publisher('/actor_red2_info', ActorInfo, queue_size=1)
            
        rospy.loginfo(f"✅ 无人机{self.drone_id}追踪器初始化完成")
        
    def _detection_callback(self, msg):
        """处理YOLO检测结果"""
        if not msg.bounding_boxes:
            # 没有检测到目标
            self._handle_no_detection()
            return
            
        # 调试日志
        rospy.loginfo_throttle(2.0, f"🔍 无人机{self.drone_id}: 收到{len(msg.bounding_boxes)}个检测框")
            
        # 选择最佳目标
        best_target = self._select_best_target(msg.bounding_boxes)
        if best_target is None:
            self._handle_no_detection()
            return
            
        # 更新追踪状态
        self.last_detection_time = rospy.Time.now()
        self.detection_count += 1
        self.lost_count = 0
        
        # 如果是第一次检测或切换目标
        if not self.tracking_active or self.current_target_color != best_target.Class:
            self.current_target_color = best_target.Class
            self._start_tracking()
            
        # 计算追踪速度
        self._compute_tracking_velocity(best_target)
        
        # 发布速度命令
        self.cmd_vel_pub.publish(self.cmd_vel)
        
        # 发布ActorInfo
        self._publish_actor_info(best_target)
        
    def _select_best_target(self, bounding_boxes):
        """选择最佳追踪目标"""
        if not bounding_boxes:
            return None
            
        # 调试：打印检测到的类别
        detected_classes = [bbox.Class for bbox in bounding_boxes]
        rospy.loginfo_throttle(2.0, f"🎯 检测到的类别: {detected_classes}")
            
        # 如果已经在追踪，优先选择同颜色目标
        if self.tracking_active and self.current_target_color:
            for bbox in bounding_boxes:
                if bbox.Class == self.current_target_color:
                    return bbox
                    
        # 否则选择最大的目标
        best_target = None
        max_area = 0
        
        for bbox in bounding_boxes:
            area = (bbox.xmax - bbox.xmin) * (bbox.ymax - bbox.ymin)
            if area > max_area:
                max_area = area
                best_target = bbox
                
        return best_target
        
    def _compute_tracking_velocity(self, target):
        """计算追踪速度（参考XTDrone算法）"""
        # 计算目标中心
        u = (target.xmax + target.xmin) / 2.0
        v = (target.ymax + target.ymin) / 2.0
        
        # 计算像素偏移
        u_offset = u - self.cx
        v_offset = v - self.cy
        
        # 估算距离（基于相机俯仰角45度）
        theta = math.radians(45)
        z = self.current_height / math.sin(theta)
        
        # 计算速度（XTDrone算法）
        u_velocity = -self.Kp_xy * u_offset
        v_velocity = -self.Kp_xy * v_offset
        
        # 转换到机体坐标系
        self.cmd_vel.linear.x = v_velocity * z / (v_offset * math.cos(theta) + self.fy * math.sin(theta))
        self.cmd_vel.linear.y = (z * u_velocity - u_offset * math.cos(theta) * self.cmd_vel.linear.x) / self.fx
        self.cmd_vel.linear.z = self.Kp_z * (self.ideal_distance - self.current_height)
        
        # 偏航控制（使目标保持在画面中心）
        self.cmd_vel.angular.z = -self.Kp_yaw * u_offset
        
        # 速度限制
        self._limit_velocity()
        
    def _limit_velocity(self):
        """限制速度"""
        # 水平速度限制
        horizontal_speed = math.sqrt(self.cmd_vel.linear.x**2 + self.cmd_vel.linear.y**2)
        if horizontal_speed > self.max_speed:
            scale = self.max_speed / horizontal_speed
            self.cmd_vel.linear.x *= scale
            self.cmd_vel.linear.y *= scale
            
        # 垂直速度限制
        self.cmd_vel.linear.z = np.clip(self.cmd_vel.linear.z, -1.0, 1.0)
        
        # 偏航速度限制
        self.cmd_vel.angular.z = np.clip(self.cmd_vel.angular.z, -0.5, 0.5)
        
    def _start_tracking(self):
        """开始追踪"""
        self.tracking_active = True
        self.tracking_status_pub.publish(Bool(data=True))
        rospy.loginfo(f"🎯 无人机{self.drone_id}: 开始追踪{self.current_target_color}目标")
        
    def _stop_tracking(self):
        """停止追踪"""
        self.tracking_active = False
        self.current_target_color = None
        self.tracking_status_pub.publish(Bool(data=False))
        self.cmd_vel = Twist()  # 清零速度
        self.cmd_vel_pub.publish(self.cmd_vel)
        rospy.loginfo(f"⏸️ 无人机{self.drone_id}: 停止追踪")
        
    def _handle_no_detection(self):
        """处理未检测到目标的情况"""
        self.lost_count += 1
        
        # 如果持续2秒未检测到，停止追踪
        if self.lost_count > 60:  # 30Hz * 2秒
            if self.tracking_active:
                self._stop_tracking()
        else:
            # 短暂丢失，保持当前速度但逐渐减小
            self.cmd_vel.linear.x *= 0.95
            self.cmd_vel.linear.y *= 0.95
            self.cmd_vel.linear.z = 0
            self.cmd_vel.angular.z *= 0.95
            self.cmd_vel_pub.publish(self.cmd_vel)
            
    def _publish_actor_info(self, target):
        """发布ActorInfo到记分系统"""
        # 如果没有位置信息，不发布
        if not hasattr(self, 'current_pose') or self.current_pose is None:
            return
            
        # 计算目标在世界坐标系中的位置
        u = (target.xmax + target.xmin) / 2.0
        v = (target.ymax + target.ymin) / 2.0
        
        # 简化计算（假设相机向下45度）
        theta = math.radians(45)
        distance = self.current_height / math.sin(theta)
        
        # 相对位置
        relative_x = distance * math.cos(theta)
        relative_y = (u - self.cx) * distance / self.fx
        
        # 转换到世界坐标系
        # 使用无人机当前位置
        drone_x = self.current_pose.position.x
        drone_y = self.current_pose.position.y
        
        # 创建ActorInfo消息（根据官方score_cal.py，ActorInfo只有x,y,cls属性）
        actor_info = ActorInfo()
        actor_info.x = drone_x + relative_x  # 加上无人机世界坐标
        actor_info.y = drone_y + relative_y
        actor_info.cls = target.Class
        
        # 发布
        if target.Class == 'red':
            # 红色目标发布到两个话题
            self.actor_pubs['red1'].publish(actor_info)
            self.actor_pubs['red2'].publish(actor_info)
        elif target.Class in self.actor_pubs:
            self.actor_pubs[target.Class].publish(actor_info)
            
    def _pose_callback(self, msg):
        """更新无人机位置"""
        self.current_height = msg.pose.position.z
        self.current_pose = msg.pose  # 保存完整位姿信息
        
    # 删除了_distance_callback，不需要距离传感器数据
        
    def run(self):
        """主循环"""
        rate = rospy.Rate(30)  # 30Hz
        
        rospy.loginfo(f"🚀 无人机{self.drone_id}追踪器开始运行")
        
        while not rospy.is_shutdown():
            # 主要处理在回调函数中完成
            rate.sleep()
            
        rospy.loginfo("追踪器停止")


if __name__ == '__main__':
    try:
        tracker = HumanTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
