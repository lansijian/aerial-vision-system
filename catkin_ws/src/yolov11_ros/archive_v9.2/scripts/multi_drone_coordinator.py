#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
多无人机协调器 - 颜色目标版本
功能：
1. 防止多架无人机同时追踪同一个颜色目标
2. 基于距离优先原则分配追踪任务
3. 分布式协调机制，无单点故障
4. 支持2-6架无人机扩展
5. 支持blue, green, white, brown, red颜色目标
"""

import rospy
import math
from typing import Dict, Optional
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, String, Float32
from yolov11_ros_msgs.msg import BoundingBoxes


class TrackingClaim:
    """追踪声明"""
    def __init__(self, drone_id: int, target_position: Point, distance: float, timestamp: float):
        self.drone_id = drone_id
        self.target_position = target_position
        self.distance = distance
        self.timestamp = timestamp


class MultiDroneCoordinator:
    """多无人机协调器节点"""
    
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('multi_drone_coordinator', anonymous=True)
        
        # === 无人机配置 ===
        self.drone_id = rospy.get_param('~drone_id', 0)
        self.num_drones = rospy.get_param('~num_drones', 2)
        
        # === 协调参数 ===
        self.same_target_threshold = rospy.get_param('~same_target_threshold', 5.0)  # 同目标判定距离（米）
        self.claim_timeout = rospy.get_param('~claim_timeout', 3.0)  # 追踪声明超时时间（秒）
        self.position_update_rate = rospy.get_param('~position_update_rate', 10)  # 位置更新频率（Hz）
        
        # === 状态变量 ===
        self.my_position: Optional[Point] = None
        self.my_tracking_active = False
        self.my_target_position: Optional[Point] = None
        self.my_target_distance: float = float('inf')
        self.my_target_color: Optional[str] = None  # 当前追踪的颜色
        
        # 其他无人机的追踪声明
        self.other_claims: Dict[int, TrackingClaim] = {}
        
        # 目标颜色列表
        self.target_colors = ['blue', 'green', 'white', 'brown', 'red']
        
        # === ROS发布者 ===
        vehicle_ns = rospy.get_param('~vehicle_ns', f'typhoon_h480_{self.drone_id}')
        
        # 发布追踪声明（让其他无人机知道我在追踪什么目标）
        self.claim_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/tracking_claim',
            String, queue_size=1
        )
        
        # 发布追踪许可（告诉本机追踪器是否可以追踪）
        self.tracking_permission_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/tracking_permission',
            Bool, queue_size=1
        )
        
        # 发布协调状态
        self.coord_status_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/coordinator_status',
            String, queue_size=1
        )
        
        # === ROS订阅者 ===
        # 订阅本机位置
        rospy.Subscriber(
            f'/{vehicle_ns}/mavros/local_position/pose',
            PoseStamped, self._position_callback, queue_size=1
        )
        
        # 订阅本机追踪状态
        rospy.Subscriber(
            f'/drone_{self.drone_id}/human_tracker/status',
            Bool, self._tracking_status_callback, queue_size=1
        )
        
        # 订阅本机检测结果（用于估计目标位置）
        rospy.Subscriber(
            '/yolov11/bounding_boxes',
            BoundingBoxes, self._detection_callback, queue_size=1
        )
        
        # 订阅其他无人机的追踪声明
        for other_id in range(self.num_drones):
            if other_id != self.drone_id:
                rospy.Subscriber(
                    f'/drone_{other_id}/tracking_claim',
                    String, 
                    lambda msg, drone_id=other_id: self._other_claim_callback(msg, drone_id),
                    queue_size=1
                )
        
        rospy.loginfo("="*60)
        rospy.loginfo(f"🤝 多无人机协调器已启动 - 无人机 {self.drone_id}")
        rospy.loginfo("="*60)
        rospy.loginfo(f"   无人机总数: {self.num_drones}")
        rospy.loginfo(f"   同目标阈值: {self.same_target_threshold}m")
        rospy.loginfo(f"   声明超时: {self.claim_timeout}s")
        rospy.loginfo(f"   协调模式: 分布式（距离优先）")
        rospy.loginfo("="*60)
    
    def _position_callback(self, msg: PoseStamped):
        """位置回调"""
        self.my_position = msg.pose.position
    
    def _tracking_status_callback(self, msg: Bool):
        """追踪状态回调"""
        self.my_tracking_active = msg.data
    
    def _detection_callback(self, msg: BoundingBoxes):
        """检测结果回调 - 估计颜色目标位置"""
        if not self.my_position:
            return
        
        # 寻找颜色目标
        color_detected = False
        for bbox in msg.bounding_boxes:
            cls_name = getattr(bbox, 'Class', '').lower()
            prob = getattr(bbox, 'probability', 0.0)
            
            if cls_name in self.target_colors and prob >= 0.3:
                color_detected = True
                self.my_target_color = cls_name
                
                # 简单估计目标位置（基于图像中心偏移和假定距离）
                bbox_width = bbox.xmax - bbox.xmin
                bbox_height = bbox.ymax - bbox.ymin
                
                # 根据边界框大小估计距离（简化版本）
                image_height = 360  # 默认图像高度
                bbox_height_ratio = bbox_height / float(image_height)
                estimated_distance = max(2.0, min(10.0, 0.5 / max(0.05, bbox_height_ratio)))
                
                # 估计目标世界坐标（假设目标在无人机前方）
                target_x = self.my_position.x + estimated_distance * 0.8
                target_y = self.my_position.y
                
                self.my_target_position = Point(x=target_x, y=target_y, z=0.0)
                self.my_target_distance = estimated_distance
                
                rospy.logdebug(f"检测到{cls_name}色目标，距离{estimated_distance:.2f}m")
                break
        
        if not color_detected:
            self.my_target_position = None
            self.my_target_distance = float('inf')
            self.my_target_color = None
    
    def _other_claim_callback(self, msg: String, drone_id: int):
        """其他无人机追踪声明回调"""
        try:
            # 解析追踪声明消息格式: "x,y,z,distance,color"
            parts = msg.data.split(',')
            if len(parts) >= 5:
                target_pos = Point(
                    x=float(parts[0]),
                    y=float(parts[1]),
                    z=float(parts[2])
                )
                distance = float(parts[3])
                color = parts[4]
                timestamp = rospy.get_time()
                
                # 存储追踪声明，包括颜色信息
                claim = TrackingClaim(
                    drone_id=drone_id,
                    target_position=target_pos,
                    distance=distance,
                    timestamp=timestamp
                )
                claim.color = color  # 添加颜色属性
                self.other_claims[drone_id] = claim
                
                rospy.logdebug(f"收到无人机{drone_id}的追踪声明: {color}色目标，距离{distance:.2f}m")
                
        except Exception as e:
            rospy.logwarn(f"解析无人机{drone_id}追踪声明失败: {e}")
    
    def _is_same_target(self, pos1: Point, pos2: Point) -> bool:
        """判断两个位置是否为同一目标"""
        if pos1 is None or pos2 is None:
            return False
        
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        return distance < self.same_target_threshold
    
    def _cleanup_expired_claims(self):
        """清理过期的追踪声明"""
        current_time = rospy.get_time()
        expired_ids = []
        
        for drone_id, claim in self.other_claims.items():
            if current_time - claim.timestamp > self.claim_timeout:
                expired_ids.append(drone_id)
        
        for drone_id in expired_ids:
            del self.other_claims[drone_id]
            rospy.logdebug(f"清理无人机{drone_id}的过期追踪声明")
    
    def _check_tracking_permission(self) -> bool:
        """
        检查是否允许追踪当前目标
        
        返回值:
            True - 允许追踪
            False - 其他无人机距离更近，应让位
        """
        if not self.my_tracking_active or not self.my_target_position:
            return True  # 不在追踪状态，默认允许
        
        # 清理过期声明
        self._cleanup_expired_claims()
        
        # 检查其他无人机是否在追踪相同颜色目标
        for drone_id, claim in self.other_claims.items():
            # 检查是否同一颜色
            if hasattr(claim, 'color') and claim.color == self.my_target_color:
                # 检查是否同一位置（在阈值范围内）
                if self._is_same_target(self.my_target_position, claim.target_position):
                    # 相同目标，比较距离
                    if claim.distance < self.my_target_distance - 0.5:  # 0.5m的容差，避免频繁切换
                        rospy.loginfo_throttle(2.0, 
                            f"⏸️ 让位: 无人机{drone_id}距离{claim.color}色目标更近 "
                            f"({claim.distance:.2f}m < {self.my_target_distance:.2f}m)"
                        )
                        return False
        
        return True
    
    def _publish_tracking_claim(self):
        """发布追踪声明"""
        if self.my_tracking_active and self.my_target_position and self.my_target_color:
            # 格式: "x,y,z,distance,color"
            claim_msg = (f"{self.my_target_position.x:.2f},"
                        f"{self.my_target_position.y:.2f},"
                        f"{self.my_target_position.z:.2f},"
                        f"{self.my_target_distance:.2f},"
                        f"{self.my_target_color}")
            
            self.claim_pub.publish(String(data=claim_msg))
            
            rospy.logdebug_throttle(2.0, 
                f"发布追踪声明: {self.my_target_color}色目标，距离{self.my_target_distance:.2f}m"
            )
    
    def _publish_tracking_permission(self, permission: bool):
        """发布追踪许可"""
        self.tracking_permission_pub.publish(Bool(data=permission))
    
    def _publish_status(self):
        """发布协调状态"""
        active_claims = len([c for c in self.other_claims.values() 
                            if rospy.get_time() - c.timestamp < self.claim_timeout])
        
        if self.my_tracking_active and self.my_target_color:
            status = f"🎯 追踪{self.my_target_color}色目标 | 距离{self.my_target_distance:.2f}m | 其他{active_claims}架活跃"
        else:
            status = f"⏸️ 待命 | 其他{active_claims}架活跃"
        
        self.coord_status_pub.publish(String(data=status))
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(self.position_update_rate)
        
        rospy.loginfo(f"🚀 协调器运行中 - 无人机 {self.drone_id}")
        
        while not rospy.is_shutdown():
            try:
                # 检查追踪许可
                permission = self._check_tracking_permission()
                
                # 发布追踪许可
                self._publish_tracking_permission(permission)
                
                # 发布追踪声明
                self._publish_tracking_claim()
                
                # 发布状态信息
                self._publish_status()
                
            except Exception as e:
                rospy.logerr(f"协调器错误: {e}")
                import traceback
                rospy.logerr(traceback.format_exc())
            
            rate.sleep()


if __name__ == '__main__':
    try:
        coordinator = MultiDroneCoordinator()
        coordinator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("协调器节点关闭")
    except Exception as e:
        rospy.logerr(f"协调器启动失败: {e}")
