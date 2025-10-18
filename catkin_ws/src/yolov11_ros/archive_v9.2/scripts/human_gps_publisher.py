#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
行人GPS发布节点
基于：
- YOLO检测框中心 -> 相机光轴方向的像素偏差
- 相机姿态（世界系）与无人机/相机位姿
- 几何射线与地面交点计算

输出：
- /human_tracker/target_gps (sensor_msgs/NavSatFix)
- ActorInfo消息到记分系统
"""

import math
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
from yolov11_ros_msgs.msg import BoundingBoxes
from pyquaternion import Quaternion


class HumanGPSPublisher:
    def __init__(self):
        rospy.init_node('human_gps_publisher', anonymous=True)

        # 参数
        self.vehicle_ns = rospy.get_param('~vehicle_ns', 'typhoon_h480_0')
        self.detection_topic = rospy.get_param('~detection_topic', '/yolov11/BoundingBoxes')
        self.cam_pose_topic = rospy.get_param('~cam_pose_topic', f'/xtdrone/{self.vehicle_ns}/cam_pose')
        self.local_pose_topic = rospy.get_param('~pose_topic', f'/{self.vehicle_ns}/mavros/local_position/pose')
        self.gps_topic = rospy.get_param('~gps_topic', f'/{self.vehicle_ns}/mavros/global_position/global')
        self.rel_alt_topic = rospy.get_param('~rel_alt_topic', f'/{self.vehicle_ns}/mavros/global_position/rel_alt')

        self.image_width = rospy.get_param('~image_width', 640)
        self.image_height = rospy.get_param('~image_height', 360)
        self.fx = rospy.get_param('~fx', 205.46963709898583)
        self.fy = rospy.get_param('~fy', 205.46963709898583)
        self.u_center = self.image_width / 2.0
        self.v_center = self.image_height / 2.0

        # 几何计算参数
        self.avg_human_height_m = rospy.get_param('~avg_human_height_m', 1.70)
        self.fallback_cam_pitch_deg = rospy.get_param('~fallback_cam_pitch_deg', -45.0)

        self.human_classes = set(rospy.get_param('~human_classes', ['person', 'human']))
        self.person_id = rospy.get_param('~person_id', 0)
        self.detection_confidence = rospy.get_param('~detection_confidence', 0.3)

        # 状态
        self.cam_pose = None  # PoseStamped
        self.local_pose = None  # PoseStamped
        self.gps_fix = None  # NavSatFix
        self.rel_alt = 0.0

        self.latest_detection = None
        self.last_pub_time = 0.0
        self.pub_interval = rospy.get_param('~pub_interval', 0.2)  # 5 Hz
        self.human_detected = False  # 是否检测到人体

        # 发布者
        self.target_gps_pub = rospy.Publisher('/human_tracker/target_gps', NavSatFix, queue_size=1)
        self.info_pub = rospy.Publisher('/human_tracker/gps_info', String, queue_size=1)

        # 计分系统 ActorInfo 多路发布（按颜色/类别映射到不同话题）
        # 默认映射与 XTDrone/robocup/score_cal.py 订阅的话题一致
        self.actor_topic_map = rospy.get_param('~actor_topic_map', {
            'green': '/actor_green_info',
            'blue': '/actor_blue_info',
            'brown': '/actor_brown_info',
            'white': '/actor_white_info',
            'red1': '/actor_red1_info',
            'red2': '/actor_red2_info'
        })
        self.default_color = rospy.get_param('~default_color', 'red')
        self.red_publish_mode = rospy.get_param('~red_publish_mode', 'round_robin')  # 或 'red1'/'red2'
        self._red_toggle = False
        self._actor_pubs = {}

        # 订阅
        rospy.Subscriber(self.cam_pose_topic, PoseStamped, self._on_cam_pose, queue_size=1)
        rospy.Subscriber(self.local_pose_topic, PoseStamped, self._on_local_pose, queue_size=1)
        rospy.Subscriber(self.gps_topic, NavSatFix, self._on_gps, queue_size=1)
        rospy.Subscriber(self.detection_topic, BoundingBoxes, self._on_detection, queue_size=1)

        # 可选：相对高度
        try:
            from std_msgs.msg import Float64
            rospy.Subscriber(self.rel_alt_topic, Float64, self._on_rel_alt, queue_size=1)
        except Exception:
            pass

        rospy.loginfo('📌 行人GPS发布节点已启动（基于几何计算）')

    def _on_cam_pose(self, msg):
        self.cam_pose = msg

    def _on_local_pose(self, msg):
        self.local_pose = msg

    def _on_gps(self, msg):
        self.gps_fix = msg

    def _on_rel_alt(self, msg):
        try:
            self.rel_alt = float(msg.data)
        except Exception:
            self.rel_alt = 0.0


    def _select_human(self, boxes):
        """选择检测到的人体"""
        for b in boxes:
            cls_name = getattr(b, 'Class', '')
            cls_id = getattr(b, 'id', None)
            prob = getattr(b, 'probability', 0.0)
            if ((cls_name in self.human_classes) or (cls_id == self.person_id)) and prob >= self.detection_confidence:
                return b
        return None
    
    def _detect_actor_color(self, target, target_pos=None):
        """
        检测actor颜色 - 完全基于位置匹配
        优先级：
        1. 基于位置匹配最近的actor
        2. 默认颜色
        """
        # 完全基于位置匹配，不使用视觉颜色检测
        if target_pos is not None:
            color = self._detect_color_by_position(target_pos)
            return color
        
        # 如果没有位置信息，返回默认颜色
        return self.default_color
    
    def _detect_color_by_position(self, target_pos):
        """
        基于位置匹配最近的actor并返回对应颜色
        根据actor初始位置（从base.world读取）
        参考score_cal.py中的actor_id_dict
        """
        # Actor初始位置（从XTDrone/robocup/base.world中读取）
        # 坐标系：ground_plane
        actor_positions = {
            0: (0, 0, 'green'),          # actor_0: green衣服
            1: (70, 22, 'blue'),         # actor_1: blue衣服
            2: (18, -18, 'brown'),       # actor_2: brown衣服
            3: (72, 32, 'white'),        # actor_3: white衣服
            4: (-32, -27, 'red'),        # actor_4: red衣服
            5: (-35, -28, 'red')         # actor_5: red衣服
        }
        
        x, y = target_pos[0], target_pos[1]
        min_dist = float('inf')
        detected_color = self.default_color
        closest_id = -1
        
        # 匹配最近的actor
        for actor_id, (px, py, color) in actor_positions.items():
            dist = ((x - px)**2 + (y - py)**2)**0.5
            if dist < min_dist:
                min_dist = dist
                detected_color = color
                closest_id = actor_id
        
        # 参考score_cal.py中err_threshold=1，这里使用宽松一些的阈值
        # 考虑到actor会移动，使用50米阈值
        if min_dist < 50.0:
            rospy.loginfo_throttle(2.0, f"📍 匹配actor_{closest_id} (距离: {min_dist:.1f}m) -> {detected_color}")
            return detected_color
        
        rospy.logwarn_throttle(5.0, f"⚠️ 未找到距离<50m的actor，最近距离: {min_dist:.1f}m")
        return self.default_color

    @staticmethod
    def _normalize(v):
        n = np.linalg.norm(v)
        if n < 1e-9:
            return v
        return v / n

    def _pixel_ray_world(self, u, v):
        # 相机位姿（世界系），若无相机姿态，则回退使用无人机位姿+固定俯仰
        if self.cam_pose is not None:
            p = self.cam_pose.pose.position
            o = self.cam_pose.pose.orientation
            q = Quaternion(o.w, o.x, o.y, o.z)
            cam_world = np.array([p.x, p.y, p.z], dtype=float)
        else:
            if self.local_pose is None:
                return None, None
            p = self.local_pose.pose.position
            o = self.local_pose.pose.orientation
            q_local = Quaternion(o.w, o.x, o.y, o.z)
            # 在机体系绕Y轴添加固定俯仰（向下）
            q_pitch = Quaternion(axis=[0, 1, 0], angle=math.radians(self.fallback_cam_pitch_deg))
            q = q_local * q_pitch
            cam_world = np.array([p.x, p.y, p.z], dtype=float)

        # 像素到相机坐标（相机坐标系：x右，y下，z前）
        # 但在无人机上，相机通常是向下的，所以我们需要正确映射
        x = (u - self.u_center) / self.fx
        y = (v - self.v_center) / self.fy
        z = 1.0
        
        # 对于向下的相机，我们可能需要调整坐标系
        # 如果相机是向下的，z应该是负数（向下）
        if self.fallback_cam_pitch_deg < 0:  # 相机向下倾斜
            z = -1.0  # 射线向下
        
        dir_cam = np.array([x, y, z], dtype=float)
        dir_cam = self._normalize(dir_cam)

        # 旋转到世界系
        # pyquaternion: q.rotate(v) 旋转向量到同一参考系
        dir_world = np.array(q.rotate(dir_cam), dtype=float)
        return cam_world, dir_world

    def _enu_to_gps(self, lat0, lon0, alt0, dx, dy, dz):
        # 简化小范围近似
        R = 6378137.0
        dlat = dy / R
        dlon = dx / (R * math.cos(math.radians(lat0)))
        return lat0 + math.degrees(dlat), lon0 + math.degrees(dlon), alt0 + dz

    def _on_detection(self, msg):
        # 仅要求 GPS 与无人机局部位姿；相机姿态可缺省（内部会回退）
        if self.gps_fix is None or self.local_pose is None:
            return

        target = self._select_human(msg.bounding_boxes)
        if target is None:
            self.human_detected = False  # 未检测到人体
            return
        
        self.human_detected = True  # 检测到人体

        u = (target.xmax + target.xmin) / 2.0
        v = (target.ymax + target.ymin) / 2.0
        u_ = u - self.u_center
        v_ = v - self.v_center

        cam_world, dir_world = self._pixel_ray_world(u, v)
        if cam_world is None:
            return

        # 基于几何计算估计目标在ground_plane坐标系下的位置
        # 地面平面假设：z = 0 (ground_plane地面高度)  
        ground_z = 0.0
        
        # 以相机世界Z与方向Z求交计算地面交点
        dz = dir_world[2]
        
        if abs(dz) > 1e-3:
            t = (ground_z - cam_world[2]) / dz
            if t > 0:
                target_ground_plane = cam_world + t * dir_world
                rospy.logdebug(f"使用地面交点估算")
            else:
                # 如果射线向上，使用单目尺度估计
                h_pix = max(1.0, float(target.ymax - target.ymin))
                depth_m = self.fx * self.avg_human_height_m / h_pix
                target_ground_plane = cam_world + depth_m * dir_world
                rospy.logdebug(f"射线向上，使用单目估算: depth={depth_m:.2f}m")
        else:
            # 地面交线不可用，使用单目尺度估计
            h_pix = max(1.0, float(target.ymax - target.ymin))
            depth_m = self.fx * self.avg_human_height_m / h_pix
            target_ground_plane = cam_world + depth_m * dir_world
            rospy.logdebug(f"dz太小，使用单目估算: depth={depth_m:.2f}m")

        if target_ground_plane is None:
            return

        # 简化GPS发布：仍然保持GPS功能，但使用简化的坐标转换
        drone_lat = self.gps_fix.latitude
        drone_lon = self.gps_fix.longitude
        drone_alt = self.gps_fix.altitude

        # 直接使用计算出的世界坐标（target_ground_plane就是世界坐标系下的位置）
        dx = 0.0  # 不需要相对偏移，直接使用绝对坐标
        dy = 0.0
        dz = 0.0

        lat, lon, alt = self._enu_to_gps(drone_lat, drone_lon, drone_alt, dx, dy, dz)

        nav = NavSatFix()
        nav.header.stamp = rospy.Time.now()
        nav.header.frame_id = 'map'
        nav.latitude = lat
        nav.longitude = lon
        nav.altitude = alt
        nav.status.status = 0
        nav.status.service = 1

        self.target_gps_pub.publish(nav)

        # 检测actor颜色并选择输出话题（传入目标位置用于基于位置的颜色判断）
        detected_color = self._detect_actor_color(target, target_ground_plane)
        color = detected_color if detected_color in ['green', 'blue', 'brown', 'white', 'red'] else self.default_color

        topic = None
        if color == 'red':
            if self.red_publish_mode in ('red1', 'red2'):
                key = self.red_publish_mode
            else:
                key = 'red1' if not self._red_toggle else 'red2'
                self._red_toggle = not self._red_toggle
            topic = self.actor_topic_map.get(key)
        else:
            topic = self.actor_topic_map.get(color)

        if topic:
            if topic not in self._actor_pubs:
                self._actor_pubs[topic] = rospy.Publisher(topic, ActorInfo, queue_size=1)
            actor_pub = self._actor_pubs[topic]
            actor_msg = ActorInfo()
            actor_msg.cls = color if color != 'red' else 'red'
            # 直接使用计算出的世界坐标，适配robocup.world坐标系
            # robocup.world坐标范围: x[-50,150], y[-50,50]
            actor_msg.x = float(target_ground_plane[0])
            actor_msg.y = float(target_ground_plane[1])
            
            # 坐标范围检查和约束
            if actor_msg.x < -50.0 or actor_msg.x > 150.0 or actor_msg.y < -50.0 or actor_msg.y > 50.0:
                rospy.logwarn(f"检测到的人体坐标超出robocup.world范围: x={actor_msg.x:.2f}, y={actor_msg.y:.2f}")
                # 将坐标约束到有效范围内
                actor_msg.x = max(-50.0, min(150.0, actor_msg.x))
                actor_msg.y = max(-50.0, min(50.0, actor_msg.y))
            
            actor_pub.publish(actor_msg)

            info = f"目标坐标: x={actor_msg.x:.2f}, y={actor_msg.y:.2f}, 颜色: {actor_msg.cls}"
            self.info_pub.publish(String(data=info))
            rospy.loginfo_throttle(1.0, f"📍 {actor_msg.cls}: ({actor_msg.x:.2f}, {actor_msg.y:.2f})")
        else:
            info = f"human_gps lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f}, actor: no topic for color '{color}'"
            self.info_pub.publish(String(data=info))
            rospy.logwarn(f"⚠️ 未找到颜色 '{color}' 对应的话题")


if __name__ == '__main__':
    try:
        node = HumanGPSPublisher()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"human_gps_publisher 启动失败: {e}")


