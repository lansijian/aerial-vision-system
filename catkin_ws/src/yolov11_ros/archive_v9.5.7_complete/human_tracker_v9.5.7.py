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
        
        # 坐标系偏移补偿（MAVROS local_position与Gazebo世界坐标系的偏移）
        self.coordinate_offset_x = rospy.get_param('~coordinate_offset_x', 0.0)
        self.coordinate_offset_y = rospy.get_param('~coordinate_offset_y', 0.0)  # 默认无偏移
        self.coordinate_offset_z = rospy.get_param('~coordinate_offset_z', 0.0)
        
        # 目标高度参数
        self.target_height = rospy.get_param('~target_height', 0.9)  # 人体重心高度
        self.assumed_human_height = rospy.get_param('~assumed_human_height', 1.7)  # 假设人体高度
        
        # 相机参数（根据typhoon_h480的cgo3相机 - 校准值）
        self.fx = 205.47  # 焦距
        self.fy = 205.47
        self.cx = 320.5  # 光心x（精确值）
        self.cy = 180.5  # 光心y（360/2的精确值）
        self.image_width = 640
        self.image_height = 360  # 实际高度
        
        # 云台角度（默认向下45度）
        self.gimbal_pitch = -45.0  # 度
        self.gimbal_yaw = 0.0
        self.gimbal_roll = 0.0
        
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
        
        # 航点重试发布者
        self.waypoint_retry_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/waypoint_retry',
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
        
        # 如果持续1秒未检测到，请求重新执行航点
        if self.lost_count > 30:  # 30Hz * 1秒
            if self.tracking_active:
                rospy.loginfo(f"⚠️ 无人机{self.drone_id}: 1秒内未检测到目标，请求重新执行航点")
                self.waypoint_retry_pub.publish(Bool(data=True))
                self._stop_tracking()
        else:
            # 短暂丢失，保持当前速度但逐渐减小
            self.cmd_vel.linear.x *= 0.95
            self.cmd_vel.linear.y *= 0.95
            self.cmd_vel.linear.z = 0
            self.cmd_vel.angular.z *= 0.95
            self.cmd_vel_pub.publish(self.cmd_vel)
            
    def _pixel_to_camera_ray(self, u, v):
        """将像素坐标转换为相机坐标系下的归一化射线方向"""
        x_norm = (u - self.cx) / self.fx
        y_norm = (v - self.cy) / self.fy
        z_norm = 1.0
        
        # 归一化
        length = math.sqrt(x_norm**2 + y_norm**2 + z_norm**2)
        return np.array([x_norm/length, y_norm/length, z_norm/length])
    
    def _compute_camera_rotation_matrix(self):
        """计算相机在世界坐标系（ENU）中的旋转矩阵"""
        if self.current_pose is None:
            return np.eye(3)
        
        # 1. 无人机姿态（body系到ENU系）
        orientation = self.current_pose.orientation
        # 四元数转旋转矩阵（处理符号问题）
        w, x, y, z = orientation.w, orientation.x, orientation.y, orientation.z
        
        # 处理四元数符号问题（确保w为正）
        if w < 0:
            w, x, y, z = -w, -x, -y, -z
        
        # 归一化四元数（防止数值误差）
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        if norm > 0:
            w, x, y, z = w/norm, x/norm, y/norm, z/norm
        
        R_body_to_enu = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]
        ])
        
        # 2. 相机基础安装（相机坐标系到body坐标系）
        # 相机: X右, Y下, Z前  ->  Body(FRD): X前, Y右, Z下
        R_cam_base = np.array([
            [0, 0, 1],   # Body X = 相机Z (前)
            [1, 0, 0],   # Body Y = 相机X (右)
            [0, 1, 0]    # Body Z = 相机Y (下)
        ])
        
        # 3. 云台角度（完整的三轴旋转）
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
    
    def _publish_actor_info(self, target):
        """发布ActorInfo到记分系统（使用准确的坐标变换）"""
        # 如果没有位置信息，不发布
        if not hasattr(self, 'current_pose') or self.current_pose is None:
            return
            
        # 计算目标在世界坐标系中的位置
        u = (target.xmax + target.xmin) / 2.0
        v = (target.ymax + target.ymin) / 2.0
        
        # 获取无人机当前位置
        drone_x = self.current_pose.position.x
        drone_y = self.current_pose.position.y
        drone_z = self.current_pose.position.z
        
        # 如果需要坐标系补偿，可通过参数配置（默认不补偿）
        if self.coordinate_offset_x != 0.0 or self.coordinate_offset_y != 0.0 or self.coordinate_offset_z != 0.0:
            drone_x += self.coordinate_offset_x
            drone_y += self.coordinate_offset_y
            drone_z += self.coordinate_offset_z
            rospy.logdebug_throttle(5.0, f"应用坐标偏移: ({self.coordinate_offset_x}, {self.coordinate_offset_y}, {self.coordinate_offset_z})")
        
        # 1. 像素坐标转相机射线
        ray_camera = self._pixel_to_camera_ray(u, v)
        
        # 2. 获取相机旋转矩阵
        R_cam_to_enu = self._compute_camera_rotation_matrix()
        
        # 3. 将射线转到ENU坐标系
        ray_enu = R_cam_to_enu @ ray_camera
        
        # 调试：打印射线方向和无人机状态
        rospy.logdebug_throttle(2.0, 
            f"[调试] 无人机高度: {drone_z:.2f}m | 射线ENU: [{ray_enu[0]:.3f}, {ray_enu[1]:.3f}, {ray_enu[2]:.3f}] | 云台pitch: {self.gimbal_pitch:.1f}°")
        
        # 4. 使用地面约束计算交点
        # 注意：target_height应该是世界坐标系中的绝对高度
        # 如果起飞点有Z轴偏移，需要考虑进去
        target_height = self.target_height  # 使用可配置的人体重心高度（世界坐标系）
        
        # 基于检测框大小估计距离（备用方案）
        box_height = target.ymax - target.ymin
        estimated_distance = (self.assumed_human_height * self.fy) / max(box_height, 10)  # 使用可配置的人体高度
        estimated_distance = max(1.0, min(50.0, estimated_distance))  # 限制范围
        
        if abs(ray_enu[2]) < 0.01:  # 射线几乎水平，无法与地面相交
            rospy.logwarn_throttle(5.0, f"[警告] 射线接近水平，使用估计距离 {estimated_distance:.2f}m")
            t = estimated_distance
        else:
            # 计算射线与地面的交点
            t = (target_height - drone_z) / ray_enu[2]
            
            if t <= 0:  # 射线向上，无法与地面相交
                rospy.logwarn_throttle(5.0, 
                    f"[警告] 射线向上! UAV_z={drone_z:.2f}, target_z={target_height:.2f}, ray_z={ray_enu[2]:.3f}, t={t:.2f}")
                t = estimated_distance
            elif t > 50:  # 距离异常远（降低阈值到50米）
                rospy.logwarn_throttle(5.0, 
                    f"[警告] 计算距离异常: t={t:.2f}m > 50m，使用估计值{estimated_distance:.2f}m | "
                    f"高度差={(target_height - drone_z):.2f}, ray_z={ray_enu[2]:.3f}")
                t = estimated_distance
            else:
                rospy.logdebug_throttle(2.0, f"[成功] 地面交点距离: {t:.2f}m")
        
        # 5. 计算人体位置
        # 安全限制：即使t值很大，也限制最终位移在合理范围内
        max_displacement = 30.0  # 最大位移30米
        if abs(t * ray_enu[0]) > max_displacement or abs(t * ray_enu[1]) > max_displacement:
            rospy.logwarn_throttle(2.0, 
                f"[警告] 位移过大，限制t值: 原始t={t:.2f}, X位移={t*ray_enu[0]:.2f}, Y位移={t*ray_enu[1]:.2f}")
            # 重新计算t，使最大位移不超过限制
            t_x = max_displacement / abs(ray_enu[0]) if abs(ray_enu[0]) > 0.01 else t
            t_y = max_displacement / abs(ray_enu[1]) if abs(ray_enu[1]) > 0.01 else t
            t = min(t, t_x, t_y)
            
        actor_x = drone_x + t * ray_enu[0]
        actor_y = drone_y + t * ray_enu[1]
        actor_z = target_height
        
        # 6. 应用地面约束（限制在合理范围内）
        actor_z = max(0.5, min(1.5, actor_z))  # 人体重心高度应在0.5-1.5米之间
        
        # 创建ActorInfo消息
        actor_info = ActorInfo()
        actor_info.x = actor_x
        actor_info.y = actor_y
        actor_info.cls = target.Class
        
        # 详细调试输出
        rospy.loginfo_throttle(1.0, 
            f"📍 [{target.Class}] 世界坐标: ({actor_x:.2f}, {actor_y:.2f}) | "
            f"无人机: ({drone_x:.2f}, {drone_y:.2f}, {drone_z:.2f}) | "
            f"像素: ({u:.0f},{v:.0f}) | 距离t: {t:.2f}m | "
            f"射线: [{ray_enu[0]:.3f},{ray_enu[1]:.3f},{ray_enu[2]:.3f}] | "
            f"置信度: {target.probability:.2f}")
        
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