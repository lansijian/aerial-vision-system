#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Human Tracker v10.0.1-restored - 恢复稳定版
恢复到v10.0.1版本，该版本追踪稳定，误差约2米

v10.2追踪效果差的原因：
- 控制增益改动过大，响应太慢
- 追踪算法改动影响了稳定性  
- 需要更多测试才能优化

功能：
1. YOLO检测结果处理
2. 高精度坐标计算（像素→世界坐标）
3. 追踪速度控制
4. ActorInfo发布（符合官方记分系统要求）

作者: 东华大学 Astraeus队
日期: 2025-10-14
版本: v10.0.1-restored
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from yolov11_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Bool
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
        
        # ===== 坐标计算参数（v10.0优化） =====
        # 相机参数（精确校准值）
        self.fx = 205.47
        self.fy = 205.47
        self.cx = 320.5  # 640/2
        self.cy = 180.5  # 360/2
        self.image_width = 640
        self.image_height = 360
        
        # 云台角度（默认向下45度）
        self.gimbal_pitch = rospy.get_param('~gimbal_pitch', -45.0)
        self.gimbal_yaw = rospy.get_param('~gimbal_yaw', 0.0)
        self.gimbal_roll = rospy.get_param('~gimbal_roll', 0.0)
        
        # 目标高度（人体重心高度，世界坐标系）
        self.target_height = rospy.get_param('~target_height', 0.9)
        
        # 坐标系偏移补偿（MAVROS→Gazebo世界坐标）
        # 因为无人机起飞点不在(0,0)，需要补偿偏移
        self.spawn_offset_x = rospy.get_param('~spawn_offset_x', 0.0)
        self.spawn_offset_y = rospy.get_param('~spawn_offset_y', 0.0)
        
        # 备用估算参数
        self.assumed_human_height = rospy.get_param('~assumed_human_height', 1.7)
        
        # 控制增益（保持原始值）
        self.Kp_xy = 0.5
        self.Kp_z = 1.0
        self.Kp_yaw = 0.002
        
        # 状态变量
        self.current_pose = None
        self.tracking_active = False
        self.last_detection_time = rospy.Time(0)
        self.lost_count = 0
        self.current_target_color = None
        self.cmd_vel = Twist()
        
        # 订阅者
        self.detection_sub = rospy.Subscriber(
            self.detection_topic, BoundingBoxes, self._detection_callback
        )
        self.pose_sub = rospy.Subscriber(
            f'/{self.vehicle_ns}/mavros/local_position/pose',
            PoseStamped, self._pose_callback
        )
        
        # 发布者
        self.cmd_vel_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/human_tracker/cmd_vel',
            Twist, queue_size=1
        )
        self.tracking_status_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/tracking_request',
            Bool, queue_size=1
        )
        self.waypoint_retry_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/waypoint_retry',
            Bool, queue_size=1
        )
        
        # ActorInfo发布者（与官方完全一致）
        self.actor_pubs = {
            'blue': rospy.Publisher('/actor_blue_info', ActorInfo, queue_size=1),
            'green': rospy.Publisher('/actor_green_info', ActorInfo, queue_size=1),
            'white': rospy.Publisher('/actor_white_info', ActorInfo, queue_size=1),
            'brown': rospy.Publisher('/actor_brown_info', ActorInfo, queue_size=1),
            'red1': rospy.Publisher('/actor_red1_info', ActorInfo, queue_size=1),
            'red2': rospy.Publisher('/actor_red2_info', ActorInfo, queue_size=1),
        }
        
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"✅ 无人机{self.drone_id} 追踪器v10.0.1-restored初始化完成")
        rospy.loginfo(f"   相机参数: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")
        rospy.loginfo(f"   云台角度: pitch={self.gimbal_pitch}°")
        rospy.loginfo(f"   目标高度: {self.target_height}m")
        rospy.loginfo(f"   坐标偏移补偿: x={self.spawn_offset_x}m, y={self.spawn_offset_y}m")
        rospy.loginfo(f"   【恢复稳定版本】追踪算法保持原始设计")
        rospy.loginfo("=" * 60)
    
    def _detection_callback(self, msg):
        """处理YOLO检测结果"""
        if not msg.bounding_boxes:
            self._handle_no_detection()
            return
        
        # 选择最佳目标
        best_target = self._select_best_target(msg.bounding_boxes)
        if best_target is None:
            self._handle_no_detection()
            return
        
        # 更新追踪状态
        self.last_detection_time = rospy.Time.now()
        self.lost_count = 0
        
        if not self.tracking_active or self.current_target_color != best_target.Class:
            self.current_target_color = best_target.Class
            self._start_tracking()
        
        # 计算追踪速度
        self._compute_tracking_velocity(best_target)
        self.cmd_vel_pub.publish(self.cmd_vel)
        
        # 发布ActorInfo（v10.0高精度算法）
        self._publish_actor_info(best_target)
    
    def _select_best_target(self, bounding_boxes):
        """选择最佳追踪目标"""
        if not bounding_boxes:
            return None
        
        # 优先选择当前追踪的目标
        if self.tracking_active and self.current_target_color:
            for bbox in bounding_boxes:
                if bbox.Class == self.current_target_color:
                    return bbox
        
        # 否则选择最大的目标
        best_target = max(bounding_boxes, 
                         key=lambda b: (b.xmax - b.xmin) * (b.ymax - b.ymin))
        return best_target
    
    def _compute_tracking_velocity(self, target):
        """计算追踪速度（原始算法）"""
        u = (target.xmax + target.xmin) / 2.0
        v = (target.ymax + target.ymin) / 2.0
        
        u_offset = u - self.cx
        v_offset = v - self.cy
        
        # 基于高度和云台角度估算距离
        if self.current_pose:
            height = self.current_pose.position.z
            theta = math.radians(abs(self.gimbal_pitch))
            z = height / max(math.sin(theta), 0.1)
        else:
            z = 3.0
        
        # 计算速度（原始公式）
        u_velocity = -self.Kp_xy * u_offset
        v_velocity = -self.Kp_xy * v_offset
        
        self.cmd_vel.linear.x = v_velocity * z / max(v_offset * math.cos(math.radians(45)) + self.fy * math.sin(math.radians(45)), 1.0)
        self.cmd_vel.linear.y = (z * u_velocity - u_offset * math.cos(math.radians(45)) * self.cmd_vel.linear.x) / max(self.fx, 1.0)
        self.cmd_vel.linear.z = self.Kp_z * (self.ideal_distance - (self.current_pose.position.z if self.current_pose else 3.0))
        self.cmd_vel.angular.z = -self.Kp_yaw * u_offset
        
        # 限速
        self._limit_velocity()
    
    def _limit_velocity(self):
        """限制速度"""
        horizontal_speed = math.sqrt(self.cmd_vel.linear.x**2 + self.cmd_vel.linear.y**2)
        if horizontal_speed > self.max_speed:
            scale = self.max_speed / horizontal_speed
            self.cmd_vel.linear.x *= scale
            self.cmd_vel.linear.y *= scale
        
        self.cmd_vel.linear.z = np.clip(self.cmd_vel.linear.z, -1.0, 1.0)
        self.cmd_vel.angular.z = np.clip(self.cmd_vel.angular.z, -0.5, 0.5)
    
    def _pixel_to_camera_ray(self, u, v):
        """像素坐标 → 相机归一化射线（v10.0精确算法）"""
        x_norm = (u - self.cx) / self.fx
        y_norm = (v - self.cy) / self.fy
        z_norm = 1.0
        
        length = math.sqrt(x_norm**2 + y_norm**2 + z_norm**2)
        return np.array([x_norm / length, y_norm / length, z_norm / length])
    
    def _quaternion_to_rotation_matrix(self, q):
        """四元数 → 旋转矩阵（v10.0优化，处理符号问题）"""
        w, x, y, z = q.w, q.x, q.y, q.z
        
        # 确保w为正（消除符号歧义）
        if w < 0:
            w, x, y, z = -w, -x, -y, -z
        
        # 归一化
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        if norm > 1e-6:
            w, x, y, z = w/norm, x/norm, y/norm, z/norm
        
        # 旋转矩阵
        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])
        return R
    
    def _compute_gimbal_rotation(self):
        """计算云台旋转矩阵（v10.0完整三轴）"""
        pitch_rad = math.radians(self.gimbal_pitch)
        yaw_rad = math.radians(self.gimbal_yaw)
        roll_rad = math.radians(self.gimbal_roll)
        
        # Pitch (绕Y轴)
        R_pitch = np.array([
            [math.cos(pitch_rad), 0, math.sin(pitch_rad)],
            [0, 1, 0],
            [-math.sin(pitch_rad), 0, math.cos(pitch_rad)]
        ])
        
        # Yaw (绕Z轴)
        R_yaw = np.array([
            [math.cos(yaw_rad), -math.sin(yaw_rad), 0],
            [math.sin(yaw_rad), math.cos(yaw_rad), 0],
            [0, 0, 1]
        ])
        
        # Roll (绕X轴)
        R_roll = np.array([
            [1, 0, 0],
            [0, math.cos(roll_rad), -math.sin(roll_rad)],
            [0, math.sin(roll_rad), math.cos(roll_rad)]
        ])
        
        return R_yaw @ R_pitch @ R_roll
    
    def _compute_camera_to_enu_rotation(self):
        """计算相机→ENU完整旋转矩阵（v10.0最优算法）"""
        if self.current_pose is None:
            return np.eye(3)
        
        # 1. Body → ENU（无人机姿态）
        R_body_to_enu = self._quaternion_to_rotation_matrix(self.current_pose.orientation)
        
        # 2. 相机基础安装（相机→Body）
        # 相机坐标系: X右, Y下, Z前
        # Body坐标系(FRD): X前, Y右, Z下
        R_cam_base = np.array([
            [0, 0, 1],  # Body_X = Cam_Z
            [1, 0, 0],  # Body_Y = Cam_X
            [0, 1, 0]   # Body_Z = Cam_Y
        ])
        
        # 3. 云台旋转
        R_gimbal = self._compute_gimbal_rotation()
        
        # 4. 完整变换链
        R_cam_to_enu = R_body_to_enu @ R_gimbal @ R_cam_base
        
        return R_cam_to_enu
    
    def _publish_actor_info(self, target):
        """发布ActorInfo（v10.0高精度算法，确保1米误差阈值）"""
        if self.current_pose is None:
            return
        
        # 1. 计算目标像素中心
        u = (target.xmax + target.xmin) / 2.0
        v = (target.ymax + target.ymin) / 2.0
        
        # 2. 无人机位置（ENU坐标系，MAVROS local_position）
        drone_x = self.current_pose.position.x
        drone_y = self.current_pose.position.y
        drone_z = self.current_pose.position.z
        
        # 3. 像素 → 相机射线
        ray_camera = self._pixel_to_camera_ray(u, v)
        
        # 4. 相机 → ENU旋转
        R_cam_to_enu = self._compute_camera_to_enu_rotation()
        ray_enu = R_cam_to_enu @ ray_camera
        
        # 5. 射线-地面交点法计算距离
        # 目标在地面高度 = target_height（世界坐标系）
        # 射线方程: P = drone_pos + t * ray_enu
        # 求解: drone_z + t * ray_enu[2] = target_height
        
        # 备用方案：基于检测框估算距离
        box_height = max(target.ymax - target.ymin, 10)
        estimated_distance = (self.assumed_human_height * self.fy) / box_height
        estimated_distance = np.clip(estimated_distance, 1.0, 30.0)
        
        if abs(ray_enu[2]) < 0.01:
            # 射线接近水平
            rospy.logwarn_throttle(5.0, f"[无人机{self.drone_id}] 射线接近水平，使用估计距离{estimated_distance:.2f}m")
            t = estimated_distance
        else:
            t = (self.target_height - drone_z) / ray_enu[2]
            
            if t <= 0:
                # 射线向上
                rospy.logwarn_throttle(5.0, f"[无人机{self.drone_id}] 射线向上，t={t:.2f}，使用估计距离")
                t = estimated_distance
            elif t > 30:
                # 距离异常大
                rospy.logwarn_throttle(5.0, f"[无人机{self.drone_id}] 距离异常t={t:.2f}m，使用估计距离{estimated_distance:.2f}m")
                t = estimated_distance
        
        # 6. 计算目标位置（MAVROS坐标系）
        actor_x_local = drone_x + t * ray_enu[0]
        actor_y_local = drone_y + t * ray_enu[1]
        
        # 7. 转换到Gazebo世界坐标系（补偿起飞点偏移）
        actor_x = actor_x_local + self.spawn_offset_x
        actor_y = actor_y_local + self.spawn_offset_y
        
        # 8. 合理性检查
        displacement = math.sqrt((t * ray_enu[0])**2 + (t * ray_enu[1])**2)
        if displacement > 30:
            rospy.logwarn_throttle(5.0, f"[无人机{self.drone_id}] 位移过大{displacement:.2f}m，限制到30m")
            scale = 30.0 / displacement
            actor_x = actor_x_local + (actor_x - actor_x_local) * scale + self.spawn_offset_x
            actor_y = actor_y_local + (actor_y - actor_y_local) * scale + self.spawn_offset_y
        
        # 9. 创建并发布ActorInfo
        actor_info = ActorInfo()
        actor_info.x = actor_x
        actor_info.y = actor_y
        actor_info.cls = target.Class
        
        # 调试日志
        rospy.loginfo_throttle(1.0,
            f"[{target.Class}] 世界坐标:({actor_x:.2f},{actor_y:.2f}) | "
            f"无人机:({drone_x:.2f},{drone_y:.2f},{drone_z:.2f}) | "
            f"距离t:{t:.2f}m | 射线ENU:[{ray_enu[0]:.3f},{ray_enu[1]:.3f},{ray_enu[2]:.3f}]")
        
        # 发布到对应话题
        if target.Class == 'red':
            self.actor_pubs['red1'].publish(actor_info)
            self.actor_pubs['red2'].publish(actor_info)
        elif target.Class in self.actor_pubs:
            self.actor_pubs[target.Class].publish(actor_info)
    
    def _start_tracking(self):
        """开始追踪"""
        self.tracking_active = True
        self.tracking_status_pub.publish(Bool(data=True))
        rospy.loginfo(f"🎯 无人机{self.drone_id}: 开始追踪{self.current_target_color}")
    
    def _stop_tracking(self):
        """停止追踪"""
        self.tracking_active = False
        self.current_target_color = None
        self.tracking_status_pub.publish(Bool(data=False))
        self.cmd_vel = Twist()
        self.cmd_vel_pub.publish(self.cmd_vel)
        rospy.loginfo(f"⏸️ 无人机{self.drone_id}: 停止追踪")
    
    def _handle_no_detection(self):
        """处理未检测到目标"""
        self.lost_count += 1
        
        if self.lost_count > 30:  # 1秒无检测
            if self.tracking_active:
                rospy.loginfo(f"⚠️ 无人机{self.drone_id}: 1秒未检测到目标，请求重新执行航点")
                self.waypoint_retry_pub.publish(Bool(data=True))
                self._stop_tracking()
        else:
            # 短暂丢失，保持减速
            self.cmd_vel.linear.x *= 0.95
            self.cmd_vel.linear.y *= 0.95
            self.cmd_vel.linear.z = 0
            self.cmd_vel.angular.z *= 0.95
            self.cmd_vel_pub.publish(self.cmd_vel)
    
    def _pose_callback(self, msg):
        """更新无人机位姿"""
        self.current_pose = msg.pose
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(30)
        rospy.loginfo(f"🚀 无人机{self.drone_id} 追踪器v10.0.1-restored开始运行")
        
        while not rospy.is_shutdown():
            rate.sleep()
        
        rospy.loginfo("追踪器停止")


if __name__ == '__main__':
    try:
        tracker = HumanTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass