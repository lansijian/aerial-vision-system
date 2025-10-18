#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
方案1: 基于MAVROS Local Position发布人体ENU坐标
不使用Gazebo话题，仅使用MAVROS提供的位置信息
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Image
from yolov11_ros_msgs.msg import BoundingBox, BoundingBoxes
from mavros_msgs.msg import MountControl
from pyquaternion import Quaternion
import sys

class HumanPositionPublisherLocal:
    def __init__(self, vehicle_type, vehicle_id):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        
        # 无人机位置和姿态
        self.uav_position = None
        self.uav_orientation = None
        
        # 云台角度（度）
        self.gimbal_pitch = -45.0  # 默认向下45度
        self.gimbal_yaw = 0.0
        self.gimbal_roll = 0.0
        
        # 相机参数（根据typhoon_h480的cgo3相机 - 已校准）
        self.image_width = 640
        self.image_height = 360  # 实际是360，不是480！
        self.focal_length = 205.47  # 像素单位的焦距（校准值）
        self.cx = 320.5  # 光心x
        self.cy = 180.5  # 光心y（360/2）
        
        # 人体检测结果
        self.latest_human_box = None
        self.human_detected = False
        
        # 距离估计参数（基于检测框大小）
        # 假设人体平均高度1.7米，用检测框高度估计距离
        self.assumed_human_height = 1.7  # 米
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("  人体位置发布器 - 方案1: MAVROS Local Position (ENU)")
        rospy.loginfo("=" * 70)
        rospy.loginfo("  车辆: %s_%s" % (vehicle_type, vehicle_id))
        rospy.loginfo("  坐标系: ENU (东-北-天)")
        rospy.loginfo("=" * 70)
        
        # 订阅器
        rospy.Subscriber(
            vehicle_type + '_' + vehicle_id + "/mavros/local_position/pose",
            PoseStamped, 
            self.uav_pose_callback, 
            queue_size=1
        )
        
        rospy.Subscriber(
            "/yolov11/BoundingBoxes",
            BoundingBoxes,
            self.yolo_callback,
            queue_size=10
        )
        
        # 如果有云台控制话题，订阅它
        # 注意：实际云台角度应该从MAVROS获取，而不是命令值
        # 这里简化处理，假设云台按命令角度工作
        
        # 发布器
        self.human_pos_pub = rospy.Publisher(
            '/xtdrone/' + vehicle_type + '_' + vehicle_id + '/human_position_local',
            PointStamped,
            queue_size=10
        )
        
        self.human_pose_pub = rospy.Publisher(
            '/xtdrone/' + vehicle_type + '_' + vehicle_id + '/human_pose_local',
            PoseStamped,
            queue_size=10
        )
        
        rospy.loginfo("[就绪] 等待MAVROS位置和YOLO检测...")
        
    def uav_pose_callback(self, msg):
        """接收无人机的位置和姿态"""
        self.uav_position = msg.pose.position
        self.uav_orientation = msg.pose.orientation
        
    def yolo_callback(self, data):
        """接收YOLO检测结果"""
        # 筛选人类目标
        valid_humans = []
        for target in data.bounding_boxes:
            class_lower = target.Class.lower()
            is_human = (class_lower == 'person' or class_lower == 'human' or target.id == 0)
            
            if is_human and target.probability > 0.4:
                valid_humans.append(target)
        
        if valid_humans:
            # 选择置信度最高的
            self.latest_human_box = max(valid_humans, key=lambda t: t.probability)
            self.human_detected = True
        else:
            self.human_detected = False
            self.latest_human_box = None
    
    def estimate_distance_from_box(self, box):
        """
        根据检测框大小估计距离
        假设人体高度约1.7米，通过相似三角形估算
        """
        box_height_pixels = box.ymax - box.ymin
        
        if box_height_pixels < 10:
            return 20.0  # 太小，估计很远
        
        # 相似三角形: distance = (real_height * focal_length) / pixel_height
        distance = (self.assumed_human_height * self.focal_length) / box_height_pixels
        
        # 限制距离范围
        distance = max(1.0, min(50.0, distance))
        
        return distance
    
    def pixel_to_camera_ray(self, u, v):
        """
        将像素坐标转换为相机坐标系下的归一化射线方向
        返回：(x, y, z) 相机坐标系下的单位向量
        """
        # 去畸变（简化，假设无畸变）
        x_norm = (u - self.cx) / self.focal_length
        y_norm = (v - self.cy) / self.focal_length
        z_norm = 1.0
        
        # 归一化
        length = math.sqrt(x_norm**2 + y_norm**2 + z_norm**2)
        return np.array([x_norm/length, y_norm/length, z_norm/length])
    
    def compute_camera_rotation_matrix(self):
        """
        计算相机在世界坐标系（ENU）中的旋转矩阵
        考虑：无人机姿态 + 云台角度 + 相机安装方式
        
        坐标系定义：
        - 相机坐标系: X右, Y下, Z前(光轴)
        - Body坐标系(FRD): X前, Y右, Z下
        - ENU坐标系: X东, Y北, Z上
        """
        if self.uav_orientation is None:
            return np.eye(3)
        
        # 1. 无人机姿态（body系到ENU系）
        q_uav = Quaternion(
            self.uav_orientation.w,
            self.uav_orientation.x,
            self.uav_orientation.y,
            self.uav_orientation.z
        )
        R_body_to_enu = q_uav.rotation_matrix
        
        # 2. 相机基础安装（相机坐标系到body坐标系的固定变换）
        # 相机: X右, Y下, Z前  ->  Body(FRD): X前, Y右, Z下
        # 变换: 相机Z->Body X, 相机X->Body Y, 相机Y->Body Z
        R_cam_base = np.array([
            [0, 0, 1],   # Body X = 相机Z (前)
            [1, 0, 0],   # Body Y = 相机X (右)
            [0, 1, 0]    # Body Z = 相机Y (下)
        ])
        
        # 3. 云台角度（绕body坐标系旋转）
        pitch_rad = math.radians(self.gimbal_pitch)
        yaw_rad = math.radians(self.gimbal_yaw)
        roll_rad = math.radians(self.gimbal_roll)
        
        # Pitch绕Body Y轴旋转（俯仰，负值向下）
        R_pitch = np.array([
            [math.cos(pitch_rad), 0, math.sin(pitch_rad)],
            [0, 1, 0],
            [-math.sin(pitch_rad), 0, math.cos(pitch_rad)]
        ])
        
        # Yaw绕Body Z轴旋转（偏航）
        R_yaw = np.array([
            [math.cos(yaw_rad), -math.sin(yaw_rad), 0],
            [math.sin(yaw_rad), math.cos(yaw_rad), 0],
            [0, 0, 1]
        ])
        
        # Roll绕Body X轴旋转（滚转）
        R_roll = np.array([
            [1, 0, 0],
            [0, math.cos(roll_rad), -math.sin(roll_rad)],
            [0, math.sin(roll_rad), math.cos(roll_rad)]
        ])
        
        # 应用云台旋转到相机基础安装上
        R_gimbal = R_yaw @ R_pitch @ R_roll
        R_cam_to_body = R_gimbal @ R_cam_base
        
        # 相机系到ENU系的旋转
        R_cam_to_enu = R_body_to_enu @ R_cam_to_body
        
        return R_cam_to_enu
    
    def compute_human_position(self):
        """
        计算人体在ENU坐标系下的位置
        使用射线与地面交点的方法，更准确！
        """
        if not self.human_detected or self.latest_human_box is None:
            return None
        
        if self.uav_position is None or self.uav_orientation is None:
            return None
        
        box = self.latest_human_box
        
        # 1. 检测框中心像素坐标
        u = (box.xmin + box.xmax) / 2.0
        v = (box.ymin + box.ymax) / 2.0
        
        # 2. 像素坐标转相机射线
        ray_camera = self.pixel_to_camera_ray(u, v)
        
        # 3. 获取相机旋转矩阵
        R_cam_to_enu = self.compute_camera_rotation_matrix()
        
        # 4. 将射线转到ENU坐标系
        ray_enu = R_cam_to_enu @ ray_camera
        
        # 调试：打印射线方向和无人机状态
        rospy.loginfo_throttle(2.0, 
            "[调试] 无人机高度: %.2fm | 射线ENU: [%.3f, %.3f, %.3f] | 云台pitch: %.1f°" % 
            (self.uav_position.z, ray_enu[0], ray_enu[1], ray_enu[2], self.gimbal_pitch))
        
        # 5. 使用地面约束计算交点（假设人体重心高度0.9米）
        # 射线方程: P = P_uav + t * ray_enu
        # 地面方程: z = 0.9
        # 求解: P_uav.z + t * ray_enu[2] = 0.9
        
        target_height = 0.9  # 人体重心高度（米）
        
        if abs(ray_enu[2]) < 0.01:  # 射线几乎水平，无法与地面相交
            rospy.logwarn_throttle(5.0, "[警告] 射线接近水平，无法准确定位 | ray_z=%.3f" % ray_enu[2])
            # 退回到距离估计方法
            distance = self.estimate_distance_from_box(box)
        else:
            # 计算射线与地面的交点
            t = (target_height - self.uav_position.z) / ray_enu[2]
            
            if t <= 0:  # 射线向上，无法与地面相交
                rospy.logwarn_throttle(5.0, 
                    "[警告] 射线向上! UAV_z=%.2f, target_z=%.2f, ray_z=%.3f, t=%.2f" % 
                    (self.uav_position.z, target_height, ray_enu[2], t))
                distance = self.estimate_distance_from_box(box)
                t = distance  # 使用估计距离
            else:
                distance = t  # 真实距离就是参数t
                rospy.loginfo_throttle(2.0, "[成功] 地面交点距离: %.2fm" % distance)
        
        # 6. 计算人体位置
        human_x = self.uav_position.x + distance * ray_enu[0]
        human_y = self.uav_position.y + distance * ray_enu[1]
        human_z = self.uav_position.z + distance * ray_enu[2]
        
        # 7. 应用地面约束（限制在合理范围内）
        human_z = max(0.5, min(1.5, human_z))  # 人体重心高度应在0.5-1.5米之间
        
        return {
            'x': human_x,
            'y': human_y,
            'z': human_z,
            'distance': distance,
            'confidence': box.probability
        }
    
    def publish_human_position(self):
        """发布人体位置"""
        human_pos = self.compute_human_position()
        
        if human_pos is None:
            return
        
        # 发布PointStamped
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = "map"  # ENU坐标系
        point_msg.point.x = human_pos['x']
        point_msg.point.y = human_pos['y']
        point_msg.point.z = human_pos['z']
        self.human_pos_pub.publish(point_msg)
        
        # 发布PoseStamped（位置+姿态，姿态为单位四元数）
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = human_pos['x']
        pose_msg.pose.position.y = human_pos['y']
        pose_msg.pose.position.z = human_pos['z']
        pose_msg.pose.orientation.w = 1.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        self.human_pose_pub.publish(pose_msg)
        
        rospy.loginfo_throttle(1.0, 
            "[人体位置] ENU: (%.2f, %.2f, %.2f) | 距离: %.2fm | 置信度: %.2f" % 
            (human_pos['x'], human_pos['y'], human_pos['z'], 
             human_pos['distance'], human_pos['confidence'])
        )
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(30)  # 30Hz
        
        while not rospy.is_shutdown():
            self.publish_human_position()
            rate.sleep()


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("用法: python human_position_publisher_local.py <vehicle_type> <vehicle_id>")
        print("示例: python human_position_publisher_local.py typhoon_h480 0")
        sys.exit(1)
    
    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    
    rospy.init_node('human_position_publisher_local')
    
    publisher = HumanPositionPublisherLocal(vehicle_type, vehicle_id)
    
    try:
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[退出] 人体位置发布器已关闭")

