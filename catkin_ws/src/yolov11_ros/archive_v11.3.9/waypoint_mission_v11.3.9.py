#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
航点飞行节点 - 使用位置控制
功能：专门负责航点飞行控制，直接发布位置目标
与人体跟踪节点通过ROS话题通信，实现优先级控制
"""

import rospy
import math
import numpy as np
import json
import threading
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from mavros_msgs.msg import PositionTarget, State
from geometry_msgs.msg import Twist
import tf.transformations as tf_trans

class WaypointMission:
    def __init__(self):
        rospy.init_node('waypoint_mission', anonymous=True)
        
        # 多机参数
        self.drone_id = rospy.get_param("~drone_id", 0)
        self.num_drones = rospy.get_param("~num_drones", 1)
        self.vehicle_type = rospy.get_param('~vehicle_type', 'typhoon_h480')
        self.vehicle_ns = f'{self.vehicle_type}_{self.drone_id}'
        
        # 飞行状态
        self.mission_active = False
        self.current_position = None
        self.current_yaw = 0.0
        self.mavros_state = None
        
        # 航点相关
        self.waypoints = []
        self.current_waypoint_index = 0
        self.waypoint_reached = False
        self.waypoint_start_time = 0
        
        # 人体跟踪状态
        self.tracking_active = False
        self.last_tracking_control_time = rospy.Time(0)
        self.latest_tracking_cmd = None
        
        # 后台setpoint发送线程
        self.background_setpoint_active = False
        self.background_thread = None
        self.setpoint_lock = threading.Lock()
        
        # 参数配置
        self.waypoint_reached_threshold = rospy.get_param("~waypoint_reached_threshold", 0.5)
        self.position_hold_time = rospy.get_param("~position_hold_time", 0.5)
        
        # ROS发布者 - 直接发布到MAVROS（位置控制）
        self.setpoint_pub = rospy.Publisher(
            f'/{self.vehicle_ns}/mavros/setpoint_raw/local', 
            PositionTarget, queue_size=1
        )
        self.flight_status_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/waypoint_flight/status', 
            String, queue_size=1
        )
        # 发布追踪状态供mission_controller使用
        self.tracking_status_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/tracking_status',
            Bool, queue_size=1
        )
        # 发布追踪控制命令供mission_controller转发
        self.tracking_control_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/tracking_control',
            Twist, queue_size=1
        )
        
        # ROS订阅者
        self.pose_sub = rospy.Subscriber(
            f'/{self.vehicle_ns}/mavros/local_position/pose', 
            PoseStamped, self._pose_callback
        )
        self.state_sub = rospy.Subscriber(
            f'/{self.vehicle_ns}/mavros/state', 
            State, self._state_callback
        )
        # 订阅追踪相关话题（注意：target_tracker发布的话题名称）
        self.tracking_request_sub = rospy.Subscriber(
            f'/drone_{self.drone_id}/request_tracking_mode', 
            Bool, self._tracking_request_callback
        )
        self.tracking_cmd_sub = rospy.Subscriber(
            f'/drone_{self.drone_id}/target_tracker/cmd_vel', 
            Twist, self._tracking_cmd_callback
        )
        # 为了兼容，也订阅human_tracker的话题（如果有的话）
        self.human_tracker_cmd_sub = rospy.Subscriber(
            f'/drone_{self.drone_id}/human_tracker/cmd_vel', 
            Twist, self._tracking_cmd_callback
        )
        self.flight_mode_sub = rospy.Subscriber(
            f'/drone_{self.drone_id}/flight_mode', 
            String, self._flight_mode_callback
        )
        
        # 加载航点
        self._load_waypoints()
        
        rospy.loginfo("="*60)
        rospy.loginfo(f"✈️ 航点任务节点已启动 - 无人机 {self.drone_id}")
        rospy.loginfo("="*60)
        rospy.loginfo(f"   无人机ID: {self.drone_id}")
        rospy.loginfo(f"   命名空间: {self.vehicle_ns}")
        rospy.loginfo(f"   航点数量: {len(self.waypoints)}")
        rospy.loginfo(f"   到达阈值: {self.waypoint_reached_threshold}米")
        rospy.loginfo(f"   使用位置控制模式")
        rospy.loginfo("="*60)
    
    def _load_waypoints(self):
        """加载航点数据"""
        # 从mission_controller获取航点文件路径
        waypoint_file = rospy.get_param(f'/drone_{self.drone_id}/mission_controller/waypoint_file', 
                                       f'waypoints/waypoints_drone_{self.drone_id}.json')
        
        # 获取ROS包路径
        import rospkg
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('yolov11_ros')
        
        # 如果是相对路径，转换为绝对路径
        import os
        if not os.path.isabs(waypoint_file):
            waypoint_file = os.path.join(pkg_path, 'scripts', waypoint_file)
        
        try:
            rospy.loginfo(f"加载航点文件: {waypoint_file}")
            with open(waypoint_file, 'r') as f:
                data = json.load(f)
                self.waypoints = data.get('waypoints', [])
                
            rospy.loginfo(f"✅ 加载了{len(self.waypoints)}个航点")
            # 显示前几个航点
            for i in range(min(5, len(self.waypoints))):
                wp = self.waypoints[i]
                rospy.loginfo(f"   航点{i}: ({wp['x']}, {wp['y']}, {wp['z']})")
                
        except Exception as e:
            rospy.logerr(f"加载航点文件失败: {e}")
            self.waypoints = []
    
    def _pose_callback(self, msg):
        """位置回调函数"""
        self.current_position = msg.pose.position
        q = msg.pose.orientation
        euler = tf_trans.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = euler[2]
    
    def _state_callback(self, msg):
        """MAVROS状态回调"""
        self.mavros_state = msg
    
    def _tracking_request_callback(self, msg):
        """追踪模式请求回调"""
        if msg.data:
            self.tracking_active = True
            self.last_tracking_control_time = rospy.Time.now()  # 更新时间以防止立即超时
            rospy.loginfo(f"🎯 无人机{self.drone_id}: 收到追踪请求，暂停航点任务")
            # 立即发布追踪状态给mission_controller
            tracking_status = Bool()
            tracking_status.data = True
            self.tracking_status_pub.publish(tracking_status)
            rospy.loginfo(f"📡 无人机{self.drone_id}: 已发布追踪状态到mission_controller")
        
    def _tracking_cmd_callback(self, msg):
        """人体跟踪控制指令回调"""
        self.latest_tracking_cmd = msg
        self.last_tracking_control_time = rospy.Time.now()
        # 收到追踪命令时自动激活追踪模式
        if not self.tracking_active:
            self.tracking_active = True
            # 立即发布追踪状态
            self.tracking_status_pub.publish(Bool(data=True))
            rospy.loginfo(f"🎯 无人机{self.drone_id}: 收到追踪命令，激活追踪模式")
        
        # 立即转发追踪命令给mission_controller
        self.tracking_control_pub.publish(msg)
        rospy.loginfo_throttle(2.0, 
            f"📡 无人机{self.drone_id}: 转发追踪命令 - "
            f"vx={msg.linear.x:.2f} vy={msg.linear.y:.2f} vz={msg.linear.z:.2f}")
    
    def _flight_mode_callback(self, msg):
        """飞行模式回调"""
        mode = msg.data
        if mode == "WAYPOINT" and not self.mission_active:
            self.mission_active = True
            rospy.loginfo(f"🚁 无人机{self.drone_id}: 开始航点任务")
        elif mode != "WAYPOINT":
            self.mission_active = False
    
    def _background_setpoint_sender(self):
        """后台线程：持续发送位置保持命令"""
        rate = rospy.Rate(30)  # 30Hz
        
        rospy.loginfo(f"🔄 后台位置保持线程已启动(30Hz) - 无人机 {self.drone_id}")
        
        while not rospy.is_shutdown() and self.background_setpoint_active:
            try:
                # 任务未激活时发送当前位置保持
                if not self.mission_active and self.current_position:
                    cmd = PositionTarget()
                    cmd.header.stamp = rospy.Time.now()
                    cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                    cmd.type_mask = (
                        PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
                        PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                        PositionTarget.IGNORE_YAW_RATE
                    )
                    cmd.position.x = self.current_position.x
                    cmd.position.y = self.current_position.y
                    cmd.position.z = self.current_position.z
                    cmd.yaw = self.current_yaw
                    
                    self.setpoint_pub.publish(cmd)
                    
            except Exception as e:
                rospy.logwarn_throttle(10.0, f"后台setpoint错误: {e}")
            
            rate.sleep()
        
        rospy.loginfo(f"🔄 后台位置保持线程已停止 - 无人机 {self.drone_id}")
    
    def _start_background_setpoint_sender(self):
        """启动后台setpoint发送线程"""
        if not self.background_setpoint_active:
            self.background_setpoint_active = True
            self.background_thread = threading.Thread(
                target=self._background_setpoint_sender,
                daemon=True
            )
            self.background_thread.start()
            rospy.loginfo(f"✅ 后台位置保持发送器已启动 - 无人机 {self.drone_id}")
    
    def _stop_background_setpoint_sender(self):
        """停止后台setpoint发送线程"""
        self.background_setpoint_active = False
        if self.background_thread:
            self.background_thread.join(timeout=2.0)
            rospy.loginfo(f"⏹️ 后台位置保持发送器已停止 - 无人机 {self.drone_id}")
    
    def _check_waypoint_reached(self):
        """检查是否到达航点"""
        if not self.current_position or self.current_waypoint_index >= len(self.waypoints):
            return False
        
        current_wp = self.waypoints[self.current_waypoint_index]
        dx = current_wp['x'] - self.current_position.x
        dy = current_wp['y'] - self.current_position.y
        dz = current_wp['z'] - self.current_position.z
        
        # 使用3D距离判断
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        return distance < self.waypoint_reached_threshold
    
    def _create_position_target(self, x, y, z, yaw=None):
        """创建位置目标消息"""
        cmd = PositionTarget()
        cmd.header.stamp = rospy.Time.now()
        cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        # 使用位置控制
        cmd.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        
        cmd.position.x = x
        cmd.position.y = y
        cmd.position.z = z
        
        if yaw is not None:
            cmd.yaw = yaw
        else:
            # 计算朝向航点的偏航角
            if self.current_position:
                dx = x - self.current_position.x
                dy = y - self.current_position.y
                if abs(dx) > 0.1 or abs(dy) > 0.1:
                    cmd.yaw = math.atan2(dy, dx)
                else:
                    cmd.yaw = self.current_yaw
            else:
                cmd.yaw = 0.0
        
        return cmd
    
    def _control_callback(self, event):
        """控制回调 - 20Hz"""
        current_time = rospy.Time.now()
        
        # 检查是否最近收到了跟踪控制指令（1.5秒内）
        time_since_tracking = (current_time - self.last_tracking_control_time).to_sec()
        actively_tracking = time_since_tracking < 1.5 and self.tracking_active
        
        if actively_tracking:
            # 正在执行跟踪 - 不执行航点控制（追踪命令已在回调中转发）
            self.flight_status_pub.publish(String(data="TRACKING_ACTIVE"))
            rospy.loginfo_throttle(3.0, f"🎯 无人机{self.drone_id}: 正在执行人体跟踪")
        else:
            # 执行航点飞行控制（位置控制）
            if self.tracking_active and time_since_tracking >= 1.5:
                rospy.loginfo("⏰ 跟踪超时，恢复航点飞行")
                self.tracking_active = False
                # 发布追踪状态为false
                self.tracking_status_pub.publish(Bool(data=False))
            
            # 只有在航点任务激活时才执行航点控制
            if not self.mission_active:
                return
                
            # 航点位置控制
            if self.current_waypoint_index >= len(self.waypoints):
                self.current_waypoint_index = 0  # 循环执行
                
            current_wp = self.waypoints[self.current_waypoint_index]
            
            # 检查是否到达航点
            if self._check_waypoint_reached():
                if not self.waypoint_reached:
                    self.waypoint_reached = True
                    self.waypoint_start_time = rospy.Time.now().to_sec()
                    rospy.loginfo(f"📍 到达航点 {self.current_waypoint_index}: ({current_wp['x']:.1f}, {current_wp['y']:.1f}, {current_wp['z']:.1f})")
                
                # 在航点停留
                hover_time_elapsed = rospy.Time.now().to_sec() - self.waypoint_start_time
                if hover_time_elapsed < self.position_hold_time:
                    # 继续发送当前航点位置
                    cmd = self._create_position_target(current_wp['x'], current_wp['y'], current_wp['z'])
                    self.setpoint_pub.publish(cmd)
                else:
                    # 停留时间结束，前往下一个航点
                    self.current_waypoint_index += 1
                    self.waypoint_reached = False
                    if self.current_waypoint_index < len(self.waypoints):
                        next_wp = self.waypoints[self.current_waypoint_index]
                        rospy.loginfo(f"➡️ 前往航点 {self.current_waypoint_index}: ({next_wp['x']:.1f}, {next_wp['y']:.1f}, {next_wp['z']:.1f})")
            else:
                # 发送目标航点位置
                cmd = self._create_position_target(current_wp['x'], current_wp['y'], current_wp['z'])
                self.setpoint_pub.publish(cmd)
                self.flight_status_pub.publish(String(data="WAYPOINT_FLIGHT"))
                
                # 调试信息
                if self.current_position:
                    dx = current_wp['x'] - self.current_position.x
                    dy = current_wp['y'] - self.current_position.y
                    dz = current_wp['z'] - self.current_position.z
                    distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                    
                    rospy.loginfo_throttle(3.0, 
                        f"无人机{self.drone_id} 航点导航: 目标WP{self.current_waypoint_index}({current_wp['x']:.1f},{current_wp['y']:.1f},{current_wp['z']:.1f}) "
                        f"当前({self.current_position.x:.1f},{self.current_position.y:.1f},{self.current_position.z:.1f}) "
                        f"距离:{distance:.1f}m")
    
    def run(self):
        """运行航点任务"""
        # 启动后台setpoint线程
        self._start_background_setpoint_sender()
        
        # 启动控制定时器
        control_timer = rospy.Timer(rospy.Duration(0.05), self._control_callback)  # 20Hz
        
        rospy.loginfo(f"🚀 航点任务节点运行中 - 无人机 {self.drone_id}")
        
        # 保持运行
        rospy.spin()
        
        # 清理
        control_timer.shutdown()
        self._stop_background_setpoint_sender()

if __name__ == '__main__':
    try:
        mission = WaypointMission()
        mission.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("航点任务节点关闭")
    except Exception as e:
        rospy.logerr(f"航点任务节点错误: {e}")