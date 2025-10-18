#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
优雅避障系统 - 非侵入式设计
功能：
- 监听声纳数据，检测前方障碍物
- 根据飞行模式采用不同避障策略  
- 通过高优先级速度指令实现避障
- 不修改现有航点飞行和人体追踪节点
"""

import rospy
import math
import numpy as np
import time
from enum import Enum
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from std_msgs.msg import String, Bool
from yolov11_ros_msgs.msg import BoundingBoxes

class FlightMode(Enum):
    WAYPOINT = "waypoint"
    TRACKING = "tracking"
    IDLE = "idle"

class AvoidanceState(Enum):
    NORMAL = "normal"
    DETECTING = "detecting" 
    AVOIDING = "avoiding"
    RECOVERING = "recovering"

class ElegantObstacleAvoidance:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('obstacle_avoidance', anonymous=True)
        
        # === 系统状态 ===
        self.current_flight_mode = FlightMode.IDLE
        self.avoidance_state = AvoidanceState.NORMAL
        self.sonar_distance = float('inf')
        self.last_sonar_time = 0.0
        
        # === 避障参数（优雅参数化设计）===
        self.config = {
            # 检测阈值
            'waypoint_safe_distance': rospy.get_param('~waypoint_safe_distance', 3.5),
            'tracking_safe_distance': rospy.get_param('~tracking_safe_distance', 2.5),
            'emergency_distance': rospy.get_param('~emergency_distance', 1.5),
            
            # 避障机动参数
            'waypoint_turn_angle': rospy.get_param('~waypoint_turn_angle', 45),
            'tracking_turn_angle': rospy.get_param('~tracking_turn_angle', 25),
            'avoidance_speed': rospy.get_param('~avoidance_speed', 1.0),
            'recovery_distance': rospy.get_param('~recovery_distance', 4.0),
            
            # 系统参数
            'sonar_timeout': rospy.get_param('~sonar_timeout', 2.0),
            'avoidance_duration': rospy.get_param('~avoidance_duration', 3.0),
            'control_frequency': rospy.get_param('~control_frequency', 20),
        }
        
        # === 避障状态变量 ===
        self.avoidance_start_time = 0.0
        self.avoidance_direction = 1  # 1为右转，-1为左转
        self.human_detected = False
        self.drone_position = None
        
        # === 状态记录变量 ===
        self.last_avoidance_action = "正常飞行"
        self.total_avoidances = 0  # 总避障次数
        self.current_turn_angle = 0.0  # 当前转向角度
        
        # === 优雅的发布者设计 ===
        vehicle_ns = rospy.get_param('~vehicle_ns', 'typhoon_h480_0')
        
        # 优雅覆盖：直接发布到主控制话题，通过高频率获得控制权
        self.avoidance_cmd_pub = rospy.Publisher(
            f'/xtdrone/{vehicle_ns}/cmd_vel_flu', 
            Twist, queue_size=1
        )
        
        # 优雅的状态通信
        self.avoidance_status_pub = rospy.Publisher(
            '/obstacle_avoidance/status', String, queue_size=1
        )
        self.avoidance_active_pub = rospy.Publisher(
            '/obstacle_avoidance/active', Bool, queue_size=1
        )
        self.flight_mode_pub = rospy.Publisher(
            '/obstacle_avoidance/flight_mode', String, queue_size=1
        )
        
        # === 优雅的订阅者设计 ===
        # 监听声纳数据
        rospy.Subscriber(f'/{vehicle_ns}/sonar', Range, 
                        self._sonar_callback, queue_size=1)
        
        # 监听系统状态（非侵入式状态推断）
        rospy.Subscriber('/human_tracker/status', Bool, 
                        self._tracking_status_callback, queue_size=1)
        rospy.Subscriber('/waypoint_flight/status', String, 
                        self._waypoint_status_callback, queue_size=1)
        rospy.Subscriber(f'/{vehicle_ns}/mavros/local_position/pose', 
                        PoseStamped, self._position_callback, queue_size=1)
        
        # 监听检测状态
        rospy.Subscriber('/yolov11/bounding_boxes', BoundingBoxes,
                        self._detection_callback, queue_size=1)
        
        rospy.loginfo("🛡️ 优雅避障系统已启动")
        rospy.loginfo(f"📊 航点模式安全距离: {self.config['waypoint_safe_distance']}m")
        rospy.loginfo(f"📊 追踪模式安全距离: {self.config['tracking_safe_distance']}m")
        
    def _sonar_callback(self, msg):
        """声纳数据回调 - 核心避障触发"""
        if msg.range > msg.min_range and msg.range < msg.max_range:
            self.sonar_distance = msg.range
            self.last_sonar_time = rospy.get_time()
        else:
            self.sonar_distance = float('inf')
            
    def _tracking_status_callback(self, msg):
        """追踪状态回调 - 优雅的状态推断"""
        if msg.data and self.human_detected:
            self.current_flight_mode = FlightMode.TRACKING
        elif self.current_flight_mode == FlightMode.TRACKING and not msg.data:
            self.current_flight_mode = FlightMode.WAYPOINT
            
    def _waypoint_status_callback(self, msg):
        """航点飞行状态回调"""
        if "flying" in msg.data.lower() and self.current_flight_mode != FlightMode.TRACKING:
            self.current_flight_mode = FlightMode.WAYPOINT
        elif "idle" in msg.data.lower() and self.current_flight_mode != FlightMode.TRACKING:
            self.current_flight_mode = FlightMode.IDLE
            
    def _detection_callback(self, msg):
        """检测状态回调 - 用于判断追踪模式"""
        self.human_detected = len([box for box in msg.bounding_boxes 
                                 if getattr(box, 'Class', '') in ['person', 'human'] 
                                 and getattr(box, 'probability', 0) > 0.3]) > 0
        
    def _position_callback(self, msg):
        """位置回调 - 用于避障决策"""
        self.drone_position = msg.pose.position
        
    def _get_safe_distance(self):
        """优雅的参数选择"""
        if self.current_flight_mode == FlightMode.TRACKING:
            return self.config['tracking_safe_distance']
        else:
            return self.config['waypoint_safe_distance']
            
    def _get_turn_angle(self):
        """根据飞行模式选择转向角度"""
        if self.current_flight_mode == FlightMode.TRACKING:
            return self.config['tracking_turn_angle']
        else:
            return self.config['waypoint_turn_angle']
            
    def _detect_obstacle(self):
        """优雅的障碍物检测逻辑"""
        # 声纳数据有效性检查
        if rospy.get_time() - self.last_sonar_time > self.config['sonar_timeout']:
            return False
            
        safe_distance = self._get_safe_distance()
        
        # 紧急情况检测
        if self.sonar_distance < self.config['emergency_distance']:
            return True
            
        # 正常避障检测
        if self.sonar_distance < safe_distance:
            return True
            
        return False
        
    def _plan_avoidance_maneuver(self):
        """优雅的避障机动规划"""
        turn_angle = self._get_turn_angle()
        
        # 智能方向选择（避免重复同向转弯）
        if not hasattr(self, '_last_turn_direction'):
            self._last_turn_direction = 1
            
        # 交替转向策略
        self.avoidance_direction = -self._last_turn_direction
        self._last_turn_direction = self.avoidance_direction
        
        # 转换为角速度指令
        angular_z = math.radians(turn_angle) * self.avoidance_direction
        self.current_turn_angle = turn_angle * self.avoidance_direction
        
        return angular_z
        
    def _execute_avoidance(self, angular_z):
        """执行避障机动 - 优雅的速度指令"""
        avoidance_cmd = Twist()
        
        # 根据距离调整避障强度
        if self.sonar_distance < self.config['emergency_distance']:
            # 紧急避障：停止前进，快速转弯
            avoidance_cmd.linear.x = 0.0
            avoidance_cmd.angular.z = angular_z * 1.5
        else:
            # 正常避障：减速前进，平稳转弯
            avoidance_cmd.linear.x = self.config['avoidance_speed'] * 0.5
            avoidance_cmd.angular.z = angular_z
            
        # 发布高优先级避障指令
        self.avoidance_cmd_pub.publish(avoidance_cmd)
        
        # 更新状态记录
        turn_direction = '右' if self.avoidance_direction > 0 else '左'
        self.last_avoidance_action = f"避障中: {turn_direction}转{abs(self.current_turn_angle):.0f}°"
        
        # 优雅的状态发布
        status_msg = (f"🛡️ 避障中: 距离{self.sonar_distance:.2f}m, "
                     f"模式{self.current_flight_mode.value}, "
                     f"转向{turn_direction}")
        self.avoidance_status_pub.publish(String(data=status_msg))
        
    def _check_recovery_condition(self):
        """检查是否可以恢复正常飞行"""
        return (self.sonar_distance > self.config['recovery_distance'] and 
                rospy.get_time() - self.avoidance_start_time > 2.0)
        
    def _publish_system_status(self):
        """优雅的系统状态发布"""
        self.avoidance_active_pub.publish(
            Bool(data=self.avoidance_state != AvoidanceState.NORMAL)
        )
        self.flight_mode_pub.publish(
            String(data=self.current_flight_mode.value)
        )
        
        
    def run(self):
        """优雅的主控制循环"""
        rate = rospy.Rate(self.config['control_frequency'])
        
        rospy.loginfo("🚁 避障系统开始监控...")
        
        while not rospy.is_shutdown():
            try:
                # === 状态机逻辑 ===
                if self.avoidance_state == AvoidanceState.NORMAL:
                    if self._detect_obstacle():
                        self.avoidance_state = AvoidanceState.DETECTING
                        self.avoidance_start_time = rospy.get_time()
                        self.last_avoidance_action = "检测到障碍物"
                        rospy.loginfo(f"🚨 检测到障碍物: {self.sonar_distance:.2f}m")
                        
                elif self.avoidance_state == AvoidanceState.DETECTING:
                    if self._detect_obstacle():
                        self.avoidance_state = AvoidanceState.AVOIDING
                        angular_z = self._plan_avoidance_maneuver()
                        self.total_avoidances += 1
                        rospy.loginfo(f"🛡️ 开始避障机动: {self.current_flight_mode.value}模式")
                    else:
                        self.avoidance_state = AvoidanceState.NORMAL
                        self.last_avoidance_action = "正常飞行"
                        
                elif self.avoidance_state == AvoidanceState.AVOIDING:
                    if self._check_recovery_condition():
                        self.avoidance_state = AvoidanceState.RECOVERING
                        rospy.loginfo("✅ 障碍物清除，准备恢复飞行")
                    else:
                        # 持续执行避障
                        angular_z = self._plan_avoidance_maneuver()
                        self._execute_avoidance(angular_z)
                        
                elif self.avoidance_state == AvoidanceState.RECOVERING:
                    # 优雅恢复：停止发布避障指令，让原系统重新接管控制权
                    # 不发布任何指令，让waypoint_flight或human_tracker重新获得控制
                    self.avoidance_state = AvoidanceState.NORMAL
                    self.last_avoidance_action = "避障完成，恢复飞行"
                    rospy.loginfo("🎯 避障完成，恢复正常飞行")
                    
                # 更新正常状态显示
                if self.avoidance_state == AvoidanceState.NORMAL:
                    self.last_avoidance_action = "正常飞行"
                
                # 优雅的状态发布
                self._publish_system_status()
                
            except Exception as e:
                rospy.logerr(f"避障系统错误: {e}")
                # 安全恢复
                safe_cmd = Twist()
                self.avoidance_cmd_pub.publish(safe_cmd)
                
            rate.sleep()

if __name__ == '__main__':
    try:
        avoidance = ElegantObstacleAvoidance()
        avoidance.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛡️ 避障系统安全关闭")
    except Exception as e:
        rospy.logerr(f"避障系统启动失败: {e}")
