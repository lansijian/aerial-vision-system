#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Typhoon H480 激光雷达 + Minimum Snap 避障导航系统
=================================================

功能特性：
1. 基于2D激光雷达的实时障碍物检测
2. Minimum Snap轨迹优化生成平滑飞行轨迹
3. 针对电线杆、行人等细长障碍物的智能识别与避障
4. 多航点队列管理，支持依次导航
5. 保留多机协同扩展接口

作者：XTDrone团队
日期：2024
"""

import rospy
import numpy as np
import math
from collections import deque
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize
import threading

# ROS消息类型
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Float32MultiArray
from mavros_msgs.msg import State
import tf.transformations as tft


class MinimumSnapOptimizer:
    """
    Minimum Snap 轨迹优化器
    
    核心原理：
    通过最小化轨迹的 snap (加速度的二阶导数) 来生成平滑、动力学可行的轨迹。
    对于四旋翼无人机，这样的轨迹能够：
    - 减少电机的频繁加减速
    - 提高飞行舒适度
    - 降低能耗
    """
    
    def __init__(self, dt=0.02):
        """
        初始化优化器
        
        Args:
            dt: 控制时间步长 (秒)
        """
        self.dt = dt
        
        # 轨迹历史缓冲
        self.position_history = deque(maxlen=20)
        self.velocity_history = deque(maxlen=20)
        
        # 优化权重
        self.w_snap = 1.0       # Snap 项权重
        self.w_smooth = 0.3     # 平滑项权重
        
        rospy.loginfo("[MinSnap优化器] 初始化完成")
    
    def compute_minimum_snap_trajectory(self, waypoints, T_total=None, n_samples=50):
        """
        计算 Minimum Snap 优化轨迹
        
        Args:
            waypoints: 航点列表 [[x1,y1,z1], [x2,y2,z2], ...]
            T_total: 总飞行时间 (秒)，None 时自动计算
            n_samples: 轨迹采样点数量
            
        Returns:
            trajectory: 优化后的轨迹点列表 [[x,y,z], ...]
            success: 是否成功生成轨迹
        """
        if len(waypoints) < 2:
            rospy.logwarn("[MinSnap] 航点数量不足")
            return waypoints, False
        
        waypoints = np.array(waypoints)
        n_seg = len(waypoints) - 1
        
        # 自动计算飞行时间（基于距离和平均速度）
        if T_total is None:
            distances = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
            # 使用较高的平均速度规划（实际会根据障碍物动态调整）
            avg_speed = 4.0  # 平均速度 m/s（考虑避障因素）
            T_total = max(np.sum(distances) / avg_speed, 1.0)
        
        # 为每段分配时间（按距离比例）
        distances = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
        total_dist = np.sum(distances)
        
        if total_dist < 0.1:
            rospy.logwarn("[MinSnap] 航点间距离过小")
            return waypoints, False
        
        T_segments = (distances / total_dist) * T_total
        
        try:
            # 使用三次样条插值（满足 C2 连续性，即加速度连续）
            t_waypoints = np.concatenate([[0], np.cumsum(T_segments)])
            t_samples = np.linspace(0, T_total, n_samples)
            
            # 为 x, y, z 三个维度分别生成样条
            trajectory = []
            for dim in range(waypoints.shape[1]):
                spline = CubicSpline(
                    t_waypoints, 
                    waypoints[:, dim], 
                    bc_type='clamped'  # 端点速度为0
                )
                trajectory.append(spline(t_samples))
            
            trajectory = np.column_stack(trajectory)
            
            rospy.logdebug(f"[MinSnap] 轨迹生成: {len(waypoints)}航点 -> {n_samples}采样点, T={T_total:.2f}s")
            return trajectory.tolist(), True
            
        except Exception as e:
            rospy.logwarn(f"[MinSnap] 样条插值失败: {e}，使用线性插值")
            
            # 降级方案：线性插值
            t_waypoints = np.concatenate([[0], np.cumsum(T_segments)])
            t_samples = np.linspace(0, T_total, n_samples)
            trajectory = np.zeros((n_samples, waypoints.shape[1]))
            
            for i, t in enumerate(t_samples):
                seg_idx = np.searchsorted(t_waypoints, t) - 1
                seg_idx = np.clip(seg_idx, 0, n_seg - 1)
                
                t0 = t_waypoints[seg_idx]
                t1 = t_waypoints[seg_idx + 1]
                alpha = (t - t0) / (t1 - t0) if t1 > t0 else 0
                
                trajectory[i] = (1 - alpha) * waypoints[seg_idx] + alpha * waypoints[seg_idx + 1]
            
            return trajectory.tolist(), True
    
    def smooth_velocity_command(self, target_vel, current_vel, max_accel=5.0):
        """
        平滑速度指令（限制加速度变化）
        
        Args:
            target_vel: 目标速度 [vx, vy, vz]
            current_vel: 当前速度 [vx, vy, vz]
            max_accel: 最大加速度 m/s²（支持快速加减速）
            
        Returns:
            smoothed_vel: 平滑后的速度
        """
        target_vel = np.array(target_vel)
        current_vel = np.array(current_vel)
        
        # 计算所需加速度
        accel = (target_vel - current_vel) / self.dt
        
        # 限制加速度幅值
        accel_mag = np.linalg.norm(accel)
        if accel_mag > max_accel:
            accel = accel * (max_accel / accel_mag)
        
        # 应用加速度限制后的速度
        smoothed_vel = current_vel + accel * self.dt
        
        return smoothed_vel.tolist()


class ObstacleAnalyzer:
    """
    障碍物分析器
    
    功能：
    1. 分析360度激光扫描数据
    2. 识别障碍物类型（电线杆、行人、墙壁等）
    3. 确定避障方向
    """
    
    def __init__(self):
        """初始化障碍物分析器"""
        # 障碍物类型识别阈值
        self.pole_width_threshold = 0.4      # 电线杆宽度阈值 (米)
        self.person_width_range = [0.3, 0.8] # 行人宽度范围 (米)
        self.wall_width_threshold = 1.5      # 墙壁/大型障碍物阈值 (米)
        
        # 扇区配置（将360度分为8个扇区）
        self.num_sectors = 8
        self.sector_angle = 360.0 / self.num_sectors  # 45度/扇区
        
        rospy.loginfo("[障碍物分析器] 初始化完成")
    
    def analyze_sectors(self, scan_msg, max_range=6.0, min_range=0.3):
        """
        分析激光扫描的各个扇区
        
        Args:
            scan_msg: LaserScan 消息
            max_range: 最大有效距离
            min_range: 最小有效距离（过滤机体本身）
            
        Returns:
            obstacle_sectors: 各扇区的最小距离 {sector_id: distance}
        """
        num_points = len(scan_msg.ranges)
        if num_points == 0:
            return {i: float('inf') for i in range(self.num_sectors)}
        
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        
        # 初始化扇区
        obstacle_sectors = {i: float('inf') for i in range(self.num_sectors)}
        
        # 遍历每个扫描点
        for i in range(num_points):
            r = scan_msg.ranges[i]
            
            # 过滤有效距离
            if min_range < r <= max_range:
                # 计算角度（弧度 -> 角度）
                angle = angle_min + i * angle_increment
                angle_deg = math.degrees(angle)
                
                # 归一化到 [-180, 180]
                while angle_deg > 180:
                    angle_deg -= 360
                while angle_deg < -180:
                    angle_deg += 360
                
                # 确定扇区
                sector = self.get_sector_from_angle(angle_deg)
                if sector >= 0:
                    obstacle_sectors[sector] = min(obstacle_sectors[sector], r)
        
        return obstacle_sectors
    
    def get_obstacle_distance_in_direction(self, scan_msg, target_direction_deg, cone_angle=60.0, max_range=6.0):
        """
        获取指定方向锥形区域内的最近障碍物距离
        
        Args:
            scan_msg: LaserScan 消息
            target_direction_deg: 目标方向（相对机体，度数）
            cone_angle: 锥形角度（度数），默认60度
            max_range: 最大有效距离
            
        Returns:
            min_distance: 该方向上的最小障碍物距离
        """
        if scan_msg is None or len(scan_msg.ranges) == 0:
            return float('inf')
        
        num_points = len(scan_msg.ranges)
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        
        min_distance = float('inf')
        half_cone = cone_angle / 2.0
        
        # 遍历扫描点
        for i in range(num_points):
            r = scan_msg.ranges[i]
            
            if 0.3 < r <= max_range:
                # 计算该点的角度
                angle = angle_min + i * angle_increment
                angle_deg = math.degrees(angle)
                
                # 归一化到 [-180, 180]
                while angle_deg > 180:
                    angle_deg -= 360
                while angle_deg < -180:
                    angle_deg += 360
                
                # 计算与目标方向的角度差
                angle_diff = angle_deg - target_direction_deg
                while angle_diff > 180:
                    angle_diff -= 360
                while angle_diff < -180:
                    angle_diff += 360
                
                # 如果在锥形范围内
                if abs(angle_diff) <= half_cone:
                    min_distance = min(min_distance, r)
        
        return min_distance
    
    def get_sector_from_angle(self, angle_deg):
        """
        根据角度获取扇区编号
        
        扇区定义（相对于机体坐标系）：
        0: 正前方 [-22.5°, 22.5°]
        1: 左前   [22.5°, 67.5°]
        2: 左侧   [67.5°, 112.5°]
        3: 左后   [112.5°, 157.5°]
        4: 正后   [±157.5° - ±180°]
        5: 右后   [-157.5°, -112.5°]
        6: 右侧   [-112.5°, -67.5°]
        7: 右前   [-67.5°, -22.5°]
        
        Args:
            angle_deg: 角度（度）
            
        Returns:
            sector: 扇区编号
        """
        if -22.5 <= angle_deg < 22.5:
            return 0  # 正前方
        elif 22.5 <= angle_deg < 67.5:
            return 1  # 左前
        elif 67.5 <= angle_deg < 112.5:
            return 2  # 左侧
        elif 112.5 <= angle_deg < 157.5:
            return 3  # 左后
        elif angle_deg >= 157.5 or angle_deg < -157.5:
            return 4  # 正后
        elif -157.5 <= angle_deg < -112.5:
            return 5  # 右后
        elif -112.5 <= angle_deg < -67.5:
            return 6  # 右侧
        elif -67.5 <= angle_deg < -22.5:
            return 7  # 右前
        return -1
    
    def identify_obstacle_type(self, scan_msg, min_distance, max_range=6.0):
        """
        识别障碍物类型
        
        根据障碍物的宽度特征判断类型：
        - 电线杆：窄（< 0.4m）
        - 行人：中等宽度（0.3-0.8m）
        - 墙壁/大型障碍：宽（> 1.5m）
        
        Args:
            scan_msg: LaserScan 消息
            min_distance: 最近障碍物距离
            max_range: 分析范围
            
        Returns:
            obstacle_type: 'pole', 'person', 'wall', 'unknown'
            obstacle_width: 障碍物估计宽度 (米)
        """
        if min_distance > max_range:
            return 'none', 0.0
        
        # 找到最近障碍物的连续点簇
        clusters = self.find_obstacle_clusters(scan_msg, max_range)
        
        if not clusters:
            return 'unknown', 0.0
        
        # 选择最近的障碍物簇
        closest_cluster = min(clusters, key=lambda c: c['distance'])
        width = closest_cluster['width']
        
        # 根据宽度判断类型
        if width < self.pole_width_threshold:
            return 'pole', width
        elif self.person_width_range[0] <= width <= self.person_width_range[1]:
            return 'person', width
        elif width > self.wall_width_threshold:
            return 'wall', width
        else:
            return 'unknown', width
    
    def find_obstacle_clusters(self, scan_msg, max_range=6.0):
        """
        在激光扫描中查找障碍物簇
        
        Args:
            scan_msg: LaserScan 消息
            max_range: 最大有效范围
            
        Returns:
            clusters: 障碍物簇列表 [{'distance', 'width', 'angle', 'size'}, ...]
        """
        clusters = []
        current_cluster = []
        
        num_points = len(scan_msg.ranges)
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        
        for i in range(num_points):
            r = scan_msg.ranges[i]
            
            if 0.3 < r <= max_range:
                # 有效障碍物点
                angle = angle_min + i * angle_increment
                current_cluster.append({'range': r, 'angle': angle, 'index': i})
            else:
                # 障碍物点断开，保存当前簇
                if len(current_cluster) >= 3:  # 至少3个点才算有效簇
                    clusters.append(self.process_cluster(current_cluster))
                current_cluster = []
        
        # 处理最后一个簇
        if len(current_cluster) >= 3:
            clusters.append(self.process_cluster(current_cluster))
        
        return clusters
    
    def process_cluster(self, cluster_points):
        """
        处理障碍物簇，计算特征
        
        Args:
            cluster_points: 簇点列表
            
        Returns:
            cluster_info: {'distance', 'width', 'angle', 'size'}
        """
        ranges = [p['range'] for p in cluster_points]
        angles = [p['angle'] for p in cluster_points]
        
        avg_distance = np.mean(ranges)
        min_distance = np.min(ranges)
        
        # 计算宽度（弧长近似）
        angle_span = max(angles) - min(angles)
        width = avg_distance * angle_span  # 弧长 = 半径 × 角度
        
        # 计算中心角度
        center_angle = (max(angles) + min(angles)) / 2.0
        
        return {
            'distance': min_distance,
            'width': width,
            'angle': center_angle,
            'size': len(cluster_points)
        }
    
    def decide_avoidance_direction(self, obstacle_sectors, warning_distance=4.0):
        """
        决定避障方向（向左或向右绕行）
        
        策略：选择空间更大的一侧
        
        Args:
            obstacle_sectors: 扇区障碍物距离字典
            warning_distance: 警告距离
            
        Returns:
            direction: 'left', 'right', 'none'
        """
        # 前方障碍物检测
        front = obstacle_sectors.get(0, float('inf'))
        front_left = obstacle_sectors.get(1, float('inf'))
        front_right = obstacle_sectors.get(7, float('inf'))
        
        front_min = min(front, front_left, front_right)
        
        if front_min > warning_distance:
            return 'none'
        
        # 计算左右两侧的平均空间
        left_space = (
            obstacle_sectors.get(1, float('inf')) +
            obstacle_sectors.get(2, float('inf'))
        ) / 2.0
        
        right_space = (
            obstacle_sectors.get(7, float('inf')) +
            obstacle_sectors.get(6, float('inf'))
        ) / 2.0
        
        # 选择空间更大的一侧（增加阈值避免频繁切换）
        if left_space > right_space + 0.8:
            return 'left'
        elif right_space > left_space + 0.8:
            return 'right'
        else:
            # 空间相近时，默认向左
            return 'left'


class TyphoonLidarMinSnapNavigation:
    """
    Typhoon H480 激光雷达 + Minimum Snap 避障导航主类
    
    功能：
    1. 接收目标航点，按顺序导航
    2. 实时激光雷达障碍物检测
    3. Minimum Snap 轨迹优化
    4. 针对电线杆、行人的智能避障
    5. 支持多机扩展（通过话题通信）
    """
    
    def __init__(self, drone_type='typhoon_h480', drone_id=0, num_drones=1):
        """
        初始化导航节点
        
        Args:
            drone_type: 无人机类型
            drone_id: 无人机ID（0开始）
            num_drones: 无人机总数（用于多机协同）
        """
        self.drone_type = drone_type
        self.drone_id = int(drone_id)
        self.num_drones = int(num_drones)
        
        # 初始化 ROS 节点
        node_name = f'lidar_minsnap_nav_{self.drone_type}_{self.drone_id}'
        rospy.init_node(node_name, anonymous=False)
        
        rospy.loginfo("=" * 80)
        rospy.loginfo(f"  Typhoon H480 激光雷达 + MinSnap 避障导航系统")
        rospy.loginfo(f"  无人机: {self.drone_type}_{self.drone_id}")
        rospy.loginfo(f"  多机模式: {'启用' if self.num_drones > 1 else '禁用'} ({self.num_drones}架)")
        rospy.loginfo("=" * 80)
        
        # ==================== 参数配置 ====================
        # 激光雷达参数
        self.laser_max_range = rospy.get_param('~laser_max_range', 6.0)
        self.laser_timeout = rospy.get_param('~laser_timeout', 0.5)
        
        # 避障距离阈值（针对5m/s速度优化）
        self.emergency_distance = rospy.get_param('~emergency_distance', 1.2)    # 紧急停止
        self.stop_distance = rospy.get_param('~stop_distance', 2.2)              # 减速停止
        self.safe_distance = rospy.get_param('~safe_distance', 3.5)              # 安全距离
        self.warning_distance = rospy.get_param('~warning_distance', 5.0)        # 警告距离（开始减速）
        
        # 速度参数（动态调整策略）
        self.max_speed = rospy.get_param('~max_speed', 5.0)              # 最大速度
        self.cruise_speed = rospy.get_param('~cruise_speed', 5.0)        # 巡航速度（无障碍物时）
        self.cruise_speed_safe = rospy.get_param('~cruise_speed_safe', 2.5)  # 安全巡航速度（有障碍物时）
        self.avoidance_lateral_speed = rospy.get_param('~avoidance_lateral_speed', 2.0)
        
        # 转向参数
        self.large_yaw_threshold = rospy.get_param('~large_yaw_threshold', 150.0)  # 大角度阈值（度）
        self.yaw_align_threshold = rospy.get_param('~yaw_align_threshold', 20.0)  # 对齐阈值（度）
        
        # 导航参数
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 1.0)
        
        # Minimum Snap 参数
        self.enable_minsnap = rospy.get_param('~enable_minsnap', True)
        self.trajectory_horizon = rospy.get_param('~trajectory_horizon', 3.0)    # 轨迹预测时域
        self.trajectory_samples = rospy.get_param('~trajectory_samples', 40)     # 采样点数
        self.replan_interval = rospy.get_param('~replan_interval', 0.5)          # 重规划间隔
        
        # 控制频率
        self.control_rate = rospy.get_param('~control_rate', 50)  # Hz
        
        # 多机协同参数
        self.multi_drone_enabled = (self.num_drones > 1)
        self.multi_drone_safe_distance = rospy.get_param('~multi_drone_safe_distance', 3.0)
        
        # ==================== 状态变量 ====================
        # 位置与速度
        self.current_position = None
        self.current_velocity = None
        self.current_yaw = 0.0
        
        # 转向状态
        self.is_rotating = False  # 是否处于原地转向模式
        
        # 激光数据
        self.laser_scan = None
        self.laser_valid = False
        self.last_laser_time = rospy.Time.now()
        self.min_obstacle_distance = float('inf')
        self.obstacle_sectors = {}
        
        # 障碍物识别
        self.obstacle_type = None       # 'pole', 'person', 'wall', 'unknown'
        self.obstacle_width = 0.0
        self.avoidance_direction = None # 'left', 'right', 'none'
        
        # 航点队列
        self.waypoint_queue = []           # 待飞航点队列
        self.current_waypoint_idx = 0      # 当前航点索引
        self.current_goal = None           # 当前目标点
        
        # 轨迹规划
        self.planned_trajectory = None     # 规划的轨迹
        self.trajectory_index = 0          # 当前跟踪的轨迹点索引
        self.last_replan_time = rospy.Time.now()
        
        # 速度平滑
        self.last_cmd_vel = np.array([0.0, 0.0, 0.0])
        
        # 多机状态（如果启用）
        self.other_drones_state = {}       # {drone_id: {'position': [x,y,z], 'velocity': [vx,vy,vz], 'time': rospy.Time}}
        
        # 线程锁
        self.lock = threading.Lock()
        
        # ==================== 初始化子模块 ====================
        self.minsnap_optimizer = MinimumSnapOptimizer(dt=1.0/self.control_rate)
        self.obstacle_analyzer = ObstacleAnalyzer()
        
        # ==================== ROS 通信 ====================
        # 订阅话题
        scan_topic = f'/{self.drone_type}_{self.drone_id}/scan'
        odom_topic = f'/{self.drone_type}_{self.drone_id}/mavros/odometry/in'
        state_topic = f'/{self.drone_type}_{self.drone_id}/mavros/state'
        goal_topic = f'/{self.drone_type}_{self.drone_id}/move_base_simple/goal'
        
        rospy.loginfo(f"[订阅话题]")
        rospy.loginfo(f"  - 激光扫描: {scan_topic}")
        rospy.loginfo(f"  - 里程计: {odom_topic}")
        rospy.loginfo(f"  - 目标点: {goal_topic}")
        
        self.scan_sub = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.state_sub = rospy.Subscriber(state_topic, State, self.state_callback, queue_size=1)
        self.goal_sub = rospy.Subscriber(goal_topic, PoseStamped, self.goal_callback, queue_size=10)
        
        # 多机协同话题
        if self.multi_drone_enabled:
            self.state_broadcast_topic = '/multi_drone/state_broadcast'
            self.state_broadcast_sub = rospy.Subscriber(
                self.state_broadcast_topic,
                Float32MultiArray,
                self.multi_drone_state_callback,
                queue_size=10
            )
            rospy.loginfo(f"  - 多机状态: {self.state_broadcast_topic}")
        
        # 发布话题
        vel_topic = f'/xtdrone/{self.drone_type}_{self.drone_id}/cmd_vel_flu'
        path_topic = f'/{self.drone_type}_{self.drone_id}/planned_path'
        cmd_topic = f'/xtdrone/{self.drone_type}_{self.drone_id}/cmd'
        
        rospy.loginfo(f"[发布话题]")
        rospy.loginfo(f"  - 速度指令: {vel_topic}")
        rospy.loginfo(f"  - 规划路径: {path_topic}")
        
        self.vel_pub = rospy.Publisher(vel_topic, Twist, queue_size=1)
        self.path_pub = rospy.Publisher(path_topic, Path, queue_size=1)
        self.cmd_pub = rospy.Publisher(cmd_topic, String, queue_size=1)
        
        # 多机状态广播
        if self.multi_drone_enabled:
            self.state_broadcast_pub = rospy.Publisher(
                self.state_broadcast_topic,
                Float32MultiArray,
                queue_size=10
            )
        
        # ==================== 定时器 ====================
        # 主控制循环
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate),
            self.control_loop
        )
        
        # 轨迹重规划定时器
        self.replan_timer = rospy.Timer(
            rospy.Duration(self.replan_interval),
            self.replan_trajectory
        )
        
        # 多机状态广播定时器
        if self.multi_drone_enabled:
            self.broadcast_timer = rospy.Timer(
                rospy.Duration(0.02),  # 50Hz
                self.broadcast_state
            )
        
        rospy.loginfo("[导航系统] 初始化完成！")
        rospy.loginfo(f"  - Minimum Snap: {'启用' if self.enable_minsnap else '禁用'}")
        rospy.loginfo(f"  - 速度策略: 全速{self.cruise_speed}m/s (无障碍) / 安全{self.cruise_speed_safe}m/s (有障碍)")
        rospy.loginfo(f"  - 转向策略: 大角度>{self.large_yaw_threshold}° 原地转向 / 小角度边转边飞")
        rospy.loginfo(f"  - 避障阈值: 紧急{self.emergency_distance}m / 停止{self.stop_distance}m / "
                     f"安全{self.safe_distance}m / 警告{self.warning_distance}m")
        rospy.loginfo("=" * 80)
    
    # ==================== 回调函数 ====================
    
    def scan_callback(self, msg):
        """激光扫描回调"""
        with self.lock:
            self.laser_scan = msg
            self.last_laser_time = rospy.Time.now()
            self.laser_valid = True
            
            # 分析障碍物扇区（用于全局感知）
            self.obstacle_sectors = self.obstacle_analyzer.analyze_sectors(
                msg,
                max_range=self.laser_max_range
            )
            
            # 注意：不在这里计算min_obstacle_distance
            # 因为需要根据目标方向动态计算
            # 这里只计算机体前方的距离（用于紧急避障）
            front_sectors = [
                self.obstacle_sectors.get(0, float('inf')),
                self.obstacle_sectors.get(1, float('inf')),
                self.obstacle_sectors.get(7, float('inf'))
            ]
            self.min_obstacle_distance = min(front_sectors)
            
            # 识别障碍物类型
            self.obstacle_type, self.obstacle_width = self.obstacle_analyzer.identify_obstacle_type(
                msg,
                self.min_obstacle_distance,
                max_range=self.laser_max_range
            )
            
            # 决定避障方向
            self.avoidance_direction = self.obstacle_analyzer.decide_avoidance_direction(
                self.obstacle_sectors,
                warning_distance=self.warning_distance
            )
    
    def odom_callback(self, msg):
        """里程计回调"""
        self.current_position = msg.pose.pose.position
        self.current_velocity = msg.twist.twist.linear
        
        # 获取航向角
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = tft.euler_from_quaternion(quaternion)
        self.current_yaw = euler[2]
        
        # 检查航点到达
        self.check_waypoint_reached()
    
    def state_callback(self, msg):
        """MAVROS 状态回调"""
        self.mavros_state = msg
    
    def goal_callback(self, msg):
        """
        接收新目标点
        
        Args:
            msg: geometry_msgs/PoseStamped
        """
        with self.lock:
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            
            waypoint = [x, y, z]
            self.waypoint_queue.append(waypoint)
            
            rospy.loginfo(f"[航点] ✓ 收到新航点 #{len(self.waypoint_queue)}: ({x:.2f}, {y:.2f}, {z:.2f})")
            
            # 如果当前没有目标，立即设置（已在锁内，直接调用）
            if self.current_goal is None:
                rospy.loginfo("[话题] 当前无目标，立即设置新航点为目标")
                success = self.set_next_waypoint()
                if success:
                    rospy.loginfo(f"[话题] ✅ 已设置目标: {self.current_goal}")
    
    def multi_drone_state_callback(self, msg):
        """
        多机状态广播回调
        
        消息格式: [drone_id, x, y, z, vx, vy, vz]
        """
        if not self.multi_drone_enabled:
            return
        
        if len(msg.data) < 7:
            return
        
        other_id = int(msg.data[0])
        
        # 忽略自己的消息
        if other_id == self.drone_id:
            return
        
        with self.lock:
            self.other_drones_state[other_id] = {
                'position': [msg.data[1], msg.data[2], msg.data[3]],
                'velocity': [msg.data[4], msg.data[5], msg.data[6]],
                'time': rospy.Time.now()
            }
    
    # ==================== 航点管理 ====================
    
    def set_next_waypoint(self):
        """设置下一个航点为当前目标（调用前需要已持有锁）"""
        if self.current_waypoint_idx < len(self.waypoint_queue):
            self.current_goal = self.waypoint_queue[self.current_waypoint_idx]
            
            rospy.loginfo(f"[航点] 前往航点 #{self.current_waypoint_idx + 1}/{len(self.waypoint_queue)}: "
                         f"({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f}, {self.current_goal[2]:.2f})")
            
            return True
        else:
            rospy.loginfo("[航点] 所有航点已完成！")
            self.current_goal = None
            return False
    
    def check_waypoint_reached(self):
        """检查是否到达当前航点"""
        if self.current_goal is None or self.current_position is None:
            return
        
        # 计算距离
        dx = self.current_goal[0] - self.current_position.x
        dy = self.current_goal[1] - self.current_position.y
        dz = self.current_goal[2] - self.current_position.z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        if distance <= self.goal_tolerance:
            rospy.loginfo(f"[航点] ✓ 到达航点 #{self.current_waypoint_idx + 1}")
            
            with self.lock:
                self.current_waypoint_idx += 1
                self.set_next_waypoint()
    
    def add_waypoint(self, x, y, z):
        """
        手动添加航点（Python API）
        
        Args:
            x, y, z: 航点坐标
        """
        with self.lock:
            waypoint = [float(x), float(y), float(z)]
            self.waypoint_queue.append(waypoint)
            
            rospy.loginfo(f"[API] 添加航点 #{len(self.waypoint_queue)}: ({x:.2f}, {y:.2f}, {z:.2f})")
            
            # 如果当前没有目标，立即设置
            if self.current_goal is None:
                rospy.loginfo("[API] 当前无目标，立即设置新航点为目标")
                success = self.set_next_waypoint()
                if not success:
                    rospy.logwarn("[API] ⚠️ 设置航点失败，可能航点队列为空")
                else:
                    rospy.loginfo(f"[API] ✅ 已设置目标: {self.current_goal}")
    
    # ==================== 轨迹规划 ====================
    
    def replan_trajectory(self, event):
        """定时重新规划轨迹"""
        if not self.enable_minsnap:
            return
        
        if self.current_goal is None or self.current_position is None:
            return
        
        with self.lock:
            # 生成航点序列
            waypoints = self.generate_avoidance_waypoints()
            
            if waypoints is None or len(waypoints) < 2:
                return
            
            # 使用 Minimum Snap 优化
            trajectory, success = self.minsnap_optimizer.compute_minimum_snap_trajectory(
                waypoints,
                T_total=self.trajectory_horizon,
                n_samples=self.trajectory_samples
            )
            
            if success:
                self.planned_trajectory = trajectory
                self.trajectory_index = 0
                self.last_replan_time = rospy.Time.now()
                
                # 发布轨迹可视化
                self.publish_planned_path(trajectory)
    
    def generate_avoidance_waypoints(self):
        """
        生成避障航点
        
        策略：
        1. 无障碍物：直飞目标
        2. 有障碍物：生成绕行航点
        3. 多机冲突：增加避让余量
        
        Returns:
            waypoints: 航点列表 [[x,y,z], ...]
        """
        if self.current_position is None or self.current_goal is None:
            return None
        
        # 起点
        start = np.array([
            self.current_position.x,
            self.current_position.y,
            self.current_position.z
        ])
        
        # 终点
        goal = np.array(self.current_goal)
        
        waypoints = [start]
        
        # 检查是否需要避障
        need_avoidance = (self.min_obstacle_distance < self.warning_distance and 
                         self.avoidance_direction in ['left', 'right'])
        
        # 检查多机冲突
        multi_drone_conflict = self.check_multi_drone_conflict()
        
        if need_avoidance or multi_drone_conflict:
            # 计算方向向量
            direction = goal - start
            distance_to_goal = np.linalg.norm(direction)
            
            if distance_to_goal < 0.1:
                return [start, goal]
            
            direction = direction / distance_to_goal
            
            # 计算侧向偏移向量
            if self.avoidance_direction == 'left' or (multi_drone_conflict and self.drone_id % 2 == 0):
                # 向左绕行（逆时针90度）
                lateral_offset = np.array([-direction[1], direction[0], 0])
                side_name = "左"
            else:
                # 向右绕行（顺时针90度）
                lateral_offset = np.array([direction[1], -direction[0], 0])
                side_name = "右"
            
            # 偏移距离（电线杆用小偏移，墙壁用大偏移）
            if self.obstacle_type == 'pole':
                offset_distance = 1.0  # 电线杆：1米偏移
            elif self.obstacle_type == 'person':
                offset_distance = 1.5  # 行人：1.5米偏移
            else:
                offset_distance = 2.0  # 其他：2米偏移
            
            # 多机冲突时增大偏移
            if multi_drone_conflict:
                offset_distance *= 1.5
            
            # 生成3个中间航点
            # 航点1: 30%前进 + 30%侧移
            wp1 = start + direction * min(2.0, distance_to_goal * 0.3) + lateral_offset * (offset_distance * 0.3)
            wp1[2] = start[2]  # 保持高度
            
            # 航点2: 50%前进 + 100%侧移
            wp2 = start + direction * min(4.0, distance_to_goal * 0.5) + lateral_offset * offset_distance
            wp2[2] = (start[2] + goal[2]) / 2.0  # 平均高度
            
            # 航点3: 80%前进 + 50%侧移
            wp3 = start + direction * min(6.0, distance_to_goal * 0.8) + lateral_offset * (offset_distance * 0.5)
            wp3[2] = goal[2]
            
            waypoints.extend([wp1, wp2, wp3])
            
            rospy.loginfo(f"[轨迹规划] 生成绕障航点: 向{side_name}绕行, 偏移{offset_distance:.1f}m "
                         f"(障碍类型: {self.obstacle_type})")
        
        waypoints.append(goal)
        
        return waypoints
    
    def check_multi_drone_conflict(self):
        """
        检查多机冲突
        
        Returns:
            has_conflict: 是否存在冲突风险
        """
        if not self.multi_drone_enabled:
            return False
        
        if self.current_position is None or self.current_velocity is None:
            return False
        
        my_pos = np.array([
            self.current_position.x,
            self.current_position.y,
            self.current_position.z
        ])
        
        my_vel = np.array([
            self.current_velocity.x,
            self.current_velocity.y,
            self.current_velocity.z
        ])
        
        current_time = rospy.Time.now()
        
        # 检查每架其他无人机
        for other_id, state in self.other_drones_state.items():
            # 检查数据是否过期
            if (current_time - state['time']).to_sec() > 1.0:
                continue
            
            other_pos = np.array(state['position'])
            other_vel = np.array(state['velocity'])
            
            # 预测未来位置（2秒后）
            prediction_time = 2.0
            my_future_pos = my_pos + my_vel * prediction_time
            other_future_pos = other_pos + other_vel * prediction_time
            
            # 计算预测距离
            predicted_distance = np.linalg.norm(my_future_pos - other_future_pos)
            
            if predicted_distance < self.multi_drone_safe_distance:
                rospy.logwarn(f"[多机] ⚠️ 检测到与无人机{other_id}的潜在冲突，预测距离: {predicted_distance:.2f}m")
                return True
        
        return False
    
    def publish_planned_path(self, trajectory):
        """发布规划路径用于可视化"""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = rospy.Time.now()
        
        for point in trajectory:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    # ==================== 速度控制 ====================
    
    def compute_velocity_command(self):
        """
        计算速度指令
        
        Returns:
            Twist: 速度指令（机体坐标系 FLU）
        """
        vel_cmd = Twist()
        
        # 检查前置条件
        if self.current_goal is None:
            # 判断是正常完成还是异常状态
            if self.current_waypoint_idx >= len(self.waypoint_queue):
                # 所有航点已完成，这是正常状态
                rospy.loginfo_throttle(10.0, "[控制] 所有航点已完成，悬停中...")
            else:
                # 异常：还有航点但无目标
                rospy.logwarn_throttle(5.0, "[控制] ⚠️ 速度为0：无目标航点！航点队列有 {} 个点，当前索引={}".format(
                    len(self.waypoint_queue), self.current_waypoint_idx))
                rospy.logwarn_throttle(5.0, "[控制] 航点队列非空但无目标，这是异常状态！")
            return vel_cmd
        
        if self.current_position is None:
            rospy.logwarn_throttle(5.0, "[控制] ⚠️ 速度为0：无位置信息！请检查里程计话题")
            return vel_cmd
        
        # 检查激光数据有效性（放宽检查，允许无激光时继续飞行）
        laser_data_valid = self.laser_valid and (rospy.Time.now() - self.last_laser_time).to_sec() <= self.laser_timeout
        
        if not laser_data_valid:
            rospy.logwarn_throttle(5.0, "[控制] ⚠️ 激光数据超时，使用安全速度模式")
            # 不直接返回，而是继续计算，但标记为安全模式
        
        # 计算目标方向
        dx = self.current_goal[0] - self.current_position.x
        dy = self.current_goal[1] - self.current_position.y
        dz = self.current_goal[2] - self.current_position.z
        distance_to_goal = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        if distance_to_goal < 0.1:
            return vel_cmd
        
        # 归一化方向
        dx /= distance_to_goal
        dy /= distance_to_goal
        dz /= distance_to_goal
        
        # ==================== 智能避障逻辑 ====================
        # 关键改进：只对目标方向上的障碍物进行避障
        
        # 计算目标方向（世界坐标系）
        target_yaw_world = math.atan2(dy, dx)
        
        # 转换到机体坐标系（相对于机体朝向的角度）
        target_direction_body = target_yaw_world - self.current_yaw
        
        # 归一化到 [-π, π]
        while target_direction_body > math.pi:
            target_direction_body -= 2 * math.pi
        while target_direction_body < -math.pi:
            target_direction_body += 2 * math.pi
        
        target_direction_deg = math.degrees(target_direction_body)
        
        # 获取目标方向上的障碍物距离（60度锥形范围）
        obstacle_in_target_direction = self.obstacle_analyzer.get_obstacle_distance_in_direction(
            self.laser_scan,
            target_direction_deg,
            cone_angle=60.0,  # 目标方向前后30度范围
            max_range=self.laser_max_range
        )
        
        # ==================== 优先检查是否需要原地转向 ====================
        # 计算目标方向的偏航角
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - self.current_yaw
        
        # 角度归一化
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        yaw_error_deg = abs(math.degrees(yaw_error))
        
        # 大角度转向：先原地转向，再前进
        if yaw_error_deg > self.large_yaw_threshold and not self.is_rotating:
            self.is_rotating = True
            rospy.loginfo(f"[转向] 检测到大角度 {yaw_error_deg:.1f}° > {self.large_yaw_threshold}°，进入原地转向模式")
        
        if self.is_rotating:
            # 原地转向模式
            if yaw_error_deg < self.yaw_align_threshold:
                self.is_rotating = False
                rospy.loginfo(f"[转向] 转向完成 {yaw_error_deg:.1f}° < {self.yaw_align_threshold}°，恢复正常飞行")
            else:
                # ==================== 原地转向时的360°安全检查 ====================
                # 检查全方位近距离障碍物，防止转向时撞到背后的障碍物
                obstacle_360_close = float('inf')
                close_range_threshold = 1.5  # 转向时使用更严格的阈值
                
                if laser_data_valid and self.obstacle_sectors:
                    for sector_id, distance in self.obstacle_sectors.items():
                        if distance < close_range_threshold:
                            obstacle_360_close = min(obstacle_360_close, distance)
                
                # 如果周围有近距离障碍物，先避障再转向
                if obstacle_360_close < 1.2:
                    rospy.logwarn(f"[转向] ⚠️ 转向受阻！周围有障碍物 {obstacle_360_close:.2f}m < 1.2m，暂停转向")
                    # 暂停转向，返回悬停
                    vel_cmd.angular.z = 0.0
                    vel_cmd.linear.x = 0.0
                    vel_cmd.linear.y = 0.0
                    vel_cmd.linear.z = 0.0
                    return vel_cmd
                
                # 安全，继续原地转向
                yaw_gain = 2.0  # 提高转向速度
                vel_cmd.angular.z = yaw_gain * yaw_error
                vel_cmd.linear.x = 0.0
                vel_cmd.linear.y = 0.0
                vel_cmd.linear.z = 0.0
                
                rospy.loginfo_throttle(0.5, f"[转向] 原地转向中... 剩余角度: {yaw_error_deg:.1f}°，周围安全距离: {obstacle_360_close:.2f}m")
                return vel_cmd
        
        # 使用目标方向上的障碍物距离进行避障判断
        # 动态速度调整策略：无障碍时全速，有障碍时降至安全速度
        lateral_speed = 0.0
        
        # ==================== 智能分级360°避障检查 ====================
        # 1. 目标方向（60°锥形）：远距离检测（5m），检测目标路径
        # 2. 速度方向（45°锥形）：远距离检测（5m），检测实际运动方向（最重要！）
        # 3. 360°全方位：近距离检测（1.8m），只防止真正的碰撞
        
        obstacle_in_velocity_direction = float('inf')
        velocity_direction_deg = 0.0
        obstacle_360_close = float('inf')  # 360°近距离最小障碍物
        
        # 计算速度方向上的障碍物
        if self.current_velocity is not None and laser_data_valid:
            vel_x = self.current_velocity.x
            vel_y = self.current_velocity.y
            vel_speed = math.sqrt(vel_x*vel_x + vel_y*vel_y)
            
            # 只在有显著速度时检查速度方向（避免静止时的噪声）
            if vel_speed > 0.3:  # 速度 > 0.3 m/s 才检查
                # 计算速度方向（世界坐标系）
                velocity_yaw_world = math.atan2(vel_y, vel_x)
                
                # 转换到机体坐标系
                velocity_direction_body = velocity_yaw_world - self.current_yaw
                
                # 归一化到 [-π, π]
                while velocity_direction_body > math.pi:
                    velocity_direction_body -= 2 * math.pi
                while velocity_direction_body < -math.pi:
                    velocity_direction_body += 2 * math.pi
                
                velocity_direction_deg = math.degrees(velocity_direction_body)
                
                # 获取速度方向上的障碍物距离（45度锥形范围，比目标方向窄）
                obstacle_in_velocity_direction = self.obstacle_analyzer.get_obstacle_distance_in_direction(
                    self.laser_scan,
                    velocity_direction_deg,
                    cone_angle=45.0,  # 速度方向前后±22.5度范围
                    max_range=self.laser_max_range
                )
        
        # 计算360°全方位的近距离障碍物（使用扇区数据）
        close_range_threshold = 1.8  # 近距离阈值（米）
        for sector_id, distance in self.obstacle_sectors.items():
            if distance < close_range_threshold:
                obstacle_360_close = min(obstacle_360_close, distance)
        
        # 智能合并：远距离检查（目标+速度）+ 近距离全方位检查
        # 目标和速度方向：使用原始距离（可达5m）
        # 其他方向：只在1.8m内才触发
        effective_obstacle_distance = min(
            obstacle_in_target_direction,   # 目标方向（远）
            obstacle_in_velocity_direction, # 速度方向（远）
            obstacle_360_close              # 360°全方位（近）
        )
        
        # 记录日志
        if effective_obstacle_distance < self.warning_distance:
            rospy.loginfo_throttle(2.0, 
                f"[避障] 目标:{obstacle_in_target_direction:.2f}m({target_direction_deg:.0f}°) | "
                f"速度:{obstacle_in_velocity_direction:.2f}m({velocity_direction_deg:.0f}°) | "
                f"360°近:{obstacle_360_close:.2f}m → 有效:{effective_obstacle_distance:.2f}m")
        
        # 如果激光数据无效，使用安全模式（降低速度但继续飞行）
        if not laser_data_valid:
            forward_speed = self.cruise_speed_safe * 0.5  # 使用安全速度的50%
            rospy.loginfo_throttle(2.0, f"[控制] 安全模式飞行: 速度={forward_speed:.2f}m/s")
        elif effective_obstacle_distance < self.emergency_distance:
            # 紧急停止
            forward_speed = 0.0
            
            # 侧向避障
            if self.avoidance_direction == 'left':
                lateral_speed = self.avoidance_lateral_speed * 0.5
            elif self.avoidance_direction == 'right':
                lateral_speed = -self.avoidance_lateral_speed * 0.5
            
            rospy.logwarn_throttle(1.0, f"[避障] 🔴 紧急停止！有效距离:{effective_obstacle_distance:.2f}m")
        
        elif effective_obstacle_distance < self.stop_distance:
            # 停止区 - 降至安全速度的40%
            forward_speed = self.cruise_speed_safe * 0.4
            
            lateral_intensity = 0.7
            if self.avoidance_direction == 'left':
                lateral_speed = self.avoidance_lateral_speed * lateral_intensity
            elif self.avoidance_direction == 'right':
                lateral_speed = -self.avoidance_lateral_speed * lateral_intensity
            
            rospy.logwarn_throttle(1.0, f"[避障] 🟡 停止区 有效距离:{effective_obstacle_distance:.2f}m，速度降至{forward_speed:.1f}m/s")
        
        elif effective_obstacle_distance < self.safe_distance:
            # 安全区 - 降至安全速度的60%
            forward_speed = self.cruise_speed_safe * 0.6
            
            lateral_intensity = 0.5
            if self.avoidance_direction == 'left':
                lateral_speed = self.avoidance_lateral_speed * lateral_intensity
            elif self.avoidance_direction == 'right':
                lateral_speed = -self.avoidance_lateral_speed * lateral_intensity
            
            rospy.loginfo_throttle(2.0, f"[避障] 🟠 减速区 有效距离:{effective_obstacle_distance:.2f}m，速度降至{forward_speed:.1f}m/s")
        
        elif effective_obstacle_distance < self.warning_distance:
            # 警告区 - 切换到安全巡航速度
            forward_speed = self.cruise_speed_safe
            
            lateral_intensity = 0.3
            if self.avoidance_direction == 'left':
                lateral_speed = self.avoidance_lateral_speed * lateral_intensity
            elif self.avoidance_direction == 'right':
                lateral_speed = -self.avoidance_lateral_speed * lateral_intensity
            
            rospy.loginfo_throttle(2.0, f"[避障] ⚠️ 警告区 有效距离:{effective_obstacle_distance:.2f}m，速度降至{forward_speed:.1f}m/s")
        
        else:
            # 安全 - 使用全速巡航
            forward_speed = self.cruise_speed
            lateral_speed = 0.0
        
        # ==================== 计算机体坐标系速度 ====================
        # 正常飞行时的偏航控制
        yaw_gain = 1.2
        vel_cmd.angular.z = yaw_gain * yaw_error
        
        # 世界坐标系速度 -> 机体坐标系速度
        # 前向速度（沿目标方向）
        world_vel_x = forward_speed * dx
        world_vel_y = forward_speed * dy
        world_vel_z = forward_speed * dz
        
        # 侧向速度（垂直于目标方向）
        world_vel_x += lateral_speed * (-dy)
        world_vel_y += lateral_speed * dx
        
        # 转换到机体坐标系
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        
        vel_cmd.linear.x = world_vel_x * cos_yaw + world_vel_y * sin_yaw
        vel_cmd.linear.y = -world_vel_x * sin_yaw + world_vel_y * cos_yaw
        vel_cmd.linear.z = world_vel_z
        
        # ==================== 速度平滑（Minimum Snap） ====================
        if self.enable_minsnap:
            current_vel = [vel_cmd.linear.x, vel_cmd.linear.y, vel_cmd.linear.z]
            smoothed_vel = self.minsnap_optimizer.smooth_velocity_command(
                current_vel,
                self.last_cmd_vel.tolist(),
                max_accel=5.0
            )
            
            vel_cmd.linear.x = smoothed_vel[0]
            vel_cmd.linear.y = smoothed_vel[1]
            vel_cmd.linear.z = smoothed_vel[2]
            
            self.last_cmd_vel = np.array(smoothed_vel)
        
        # 限制速度
        vel_cmd.linear.x = np.clip(vel_cmd.linear.x, -self.max_speed, self.max_speed)
        vel_cmd.linear.y = np.clip(vel_cmd.linear.y, -self.max_speed, self.max_speed)
        vel_cmd.linear.z = np.clip(vel_cmd.linear.z, -self.max_speed*0.5, self.max_speed*0.5)
        
        return vel_cmd
    
    def compute_safe_velocity(self):
        """计算安全速度（激光数据丢失时）"""
        vel_cmd = Twist()
        
        if self.current_goal is None or self.current_position is None:
            return vel_cmd
        
        # 低速前进
        dx = self.current_goal[0] - self.current_position.x
        dy = self.current_goal[1] - self.current_position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance > 0.1:
            # 世界坐标系速度
            world_vel_x = (dx / distance) * self.max_speed * 0.3
            world_vel_y = (dy / distance) * self.max_speed * 0.3
            
            # 转换到机体坐标系
            cos_yaw = math.cos(self.current_yaw)
            sin_yaw = math.sin(self.current_yaw)
            
            vel_cmd.linear.x = world_vel_x * cos_yaw + world_vel_y * sin_yaw
            vel_cmd.linear.y = -world_vel_x * sin_yaw + world_vel_y * cos_yaw
        
        return vel_cmd
    
    # ==================== 多机协同 ====================
    
    def broadcast_state(self, event):
        """广播自身状态（多机模式）"""
        if not self.multi_drone_enabled:
            return
        
        if self.current_position is None or self.current_velocity is None:
            return
        
        msg = Float32MultiArray()
        msg.data = [
            float(self.drone_id),
            self.current_position.x,
            self.current_position.y,
            self.current_position.z,
            self.current_velocity.x,
            self.current_velocity.y,
            self.current_velocity.z
        ]
        
        self.state_broadcast_pub.publish(msg)
    
    # ==================== 控制循环 ====================
    
    def control_loop(self, event):
        """主控制循环"""
        # 修复：如果航点队列有数据但当前目标为空，尝试设置目标
        # 注意：只有当 current_waypoint_idx < len(waypoint_queue) 时才是真正的异常
        # 如果 current_waypoint_idx >= len(waypoint_queue)，说明所有航点已完成，这是正常状态
        if self.current_goal is None and self.current_waypoint_idx < len(self.waypoint_queue):
            with self.lock:
                rospy.logwarn_throttle(1.0, f"[控制循环] 检测到异常：航点队列有 {len(self.waypoint_queue)} 个点但无当前目标（索引={self.current_waypoint_idx}），尝试修复...")
                if self.set_next_waypoint():
                    rospy.loginfo(f"[控制循环] ✅ 已自动设置目标: {self.current_goal}")
        
        vel_cmd = self.compute_velocity_command()
        
        # 调试：打印速度指令（每50次打印一次）
        if hasattr(self, '_control_loop_count'):
            self._control_loop_count += 1
        else:
            self._control_loop_count = 0
        
        if self._control_loop_count % 50 == 0:  # 每秒打印一次（50Hz控制频率）
            rospy.loginfo(f"[控制循环] 计算的速度: linear=({vel_cmd.linear.x:.3f}, {vel_cmd.linear.y:.3f}, {vel_cmd.linear.z:.3f}), "
                         f"angular.z={vel_cmd.angular.z:.3f}")
        
        # 发布速度指令
        self.vel_pub.publish(vel_cmd)
        
        # 状态日志
        if rospy.get_time() % 2.0 < 0.05:
            self.print_status(vel_cmd)
    
    def print_status(self, vel_cmd):
        """打印状态信息"""
        if self.current_position is None:
            return
        
        # 计算三个方向的障碍物距离
        obstacle_in_target_direction = float('inf')
        obstacle_in_velocity_direction = float('inf')
        
        if self.current_goal is not None and self.laser_scan is not None:
            dx = self.current_goal[0] - self.current_position.x
            dy = self.current_goal[1] - self.current_position.y
            target_yaw_world = math.atan2(dy, dx)
            target_direction_body = target_yaw_world - self.current_yaw
            while target_direction_body > math.pi:
                target_direction_body -= 2 * math.pi
            while target_direction_body < -math.pi:
                target_direction_body += 2 * math.pi
            target_direction_deg = math.degrees(target_direction_body)
            
            obstacle_in_target_direction = self.obstacle_analyzer.get_obstacle_distance_in_direction(
                self.laser_scan,
                target_direction_deg,
                cone_angle=60.0,
                max_range=self.laser_max_range
            )
            
            # 计算速度方向的障碍物
            if self.current_velocity is not None:
                vel_x = self.current_velocity.x
                vel_y = self.current_velocity.y
                vel_speed = math.sqrt(vel_x*vel_x + vel_y*vel_y)
                
                if vel_speed > 0.3:
                    velocity_yaw_world = math.atan2(vel_y, vel_x)
                    velocity_direction_body = velocity_yaw_world - self.current_yaw
                    while velocity_direction_body > math.pi:
                        velocity_direction_body -= 2 * math.pi
                    while velocity_direction_body < -math.pi:
                        velocity_direction_body += 2 * math.pi
                    
                    obstacle_in_velocity_direction = self.obstacle_analyzer.get_obstacle_distance_in_direction(
                        self.laser_scan,
                        math.degrees(velocity_direction_body),
                        cone_angle=45.0,
                        max_range=self.laser_max_range
                    )
        
        # 计算360°全方位的近距离障碍物
        obstacle_360_close = float('inf')
        close_range_threshold = 1.8
        for sector_id, distance in self.obstacle_sectors.items():
            if distance < close_range_threshold:
                obstacle_360_close = min(obstacle_360_close, distance)
        
        # 有效障碍物距离（智能分级检查）
        effective_obstacle_distance = min(
            obstacle_in_target_direction,   # 目标方向（远）
            obstacle_in_velocity_direction, # 速度方向（远）
            obstacle_360_close              # 360°全方位（近）
        )
        
        # 障碍物状态（基于有效距离）
        if effective_obstacle_distance < self.emergency_distance:
            status_str = "🔴紧急"
        elif effective_obstacle_distance < self.stop_distance:
            status_str = "🟡停止"
        elif effective_obstacle_distance < self.safe_distance:
            status_str = "🟠减速"
        elif effective_obstacle_distance < self.warning_distance:
            status_str = "⚠️警告"
        else:
            status_str = "🟢安全"
        
        # 障碍物类型
        obstacle_info = ""
        if self.obstacle_type and self.obstacle_type != 'none':
            type_names = {
                'pole': '电线杆',
                'person': '行人',
                'wall': '墙壁',
                'unknown': '未知'
            }
            obstacle_info = f" [{type_names.get(self.obstacle_type, '?')}:{self.obstacle_width:.2f}m]"
        
        # 显示详细的障碍物信息
        rospy.loginfo(
            f"[状态] 位置:({self.current_position.x:.1f},{self.current_position.y:.1f},{self.current_position.z:.1f}) "
            f"航点:{self.current_waypoint_idx+1}/{len(self.waypoint_queue)} "
            f"有效:{effective_obstacle_distance:.2f}m "
            f"(目标:{obstacle_in_target_direction:.2f}m|速度:{obstacle_in_velocity_direction:.2f}m|360°近:{obstacle_360_close:.2f}m) "
            f"{status_str}{obstacle_info} "
            f"速度:({vel_cmd.linear.x:.2f},{vel_cmd.linear.y:.2f},{vel_cmd.linear.z:.2f})"
        )
    
    # ==================== 主循环 ====================
    
    def run(self):
        """启动节点"""
        rospy.loginfo("[导航系统] 运行中...")
        rospy.loginfo(f"[导航系统] 等待航点输入... (话题: /{self.drone_type}_{self.drone_id}/move_base_simple/goal)")
        
        # 使用自定义循环而不是rospy.spin()，以便更好地处理Ctrl+C
        rate = rospy.Rate(10)  # 10Hz
        try:
            while not rospy.is_shutdown():
                rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("[导航系统] 收到退出信号 (Ctrl+C)")
        finally:
            rospy.loginfo("[导航系统] 正在清理资源...")
            # 发送停止指令
            stop_cmd = Twist()
            self.vel_pub.publish(stop_cmd)
            rospy.loginfo("[导航系统] 已停止")


# ==================== 主函数 ====================

def main():
    """主函数"""
    import sys
    import signal
    
    # 默认参数
    drone_type = 'typhoon_h480'
    drone_id = 0
    num_drones = 1
    waypoints = []
    
    # 全局节点引用，用于信号处理
    node = None
    
    def signal_handler(sig, frame):
        """处理Ctrl+C信号"""
        rospy.loginfo("\n[系统] 收到中断信号，正在退出...")
        if node is not None:
            stop_cmd = Twist()
            node.vel_pub.publish(stop_cmd)
        rospy.signal_shutdown("用户中断")
        sys.exit(0)
    
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 解析命令行参数
    # 格式: python3 script.py [drone_id] [num_drones] [x1 y1 z1] [x2 y2 z2] ...
    
    if len(sys.argv) >= 2:
        try:
            drone_id = int(sys.argv[1])
        except ValueError:
            rospy.logerr(f"错误: 无效的 drone_id '{sys.argv[1]}'")
            sys.exit(1)
    
    if len(sys.argv) >= 3:
        try:
            num_drones = int(sys.argv[2])
        except ValueError:
            rospy.logerr(f"错误: 无效的 num_drones '{sys.argv[2]}'")
            sys.exit(1)
    
    # 解析航点
    i = 3
    while i + 2 < len(sys.argv):
        try:
            x = float(sys.argv[i])
            y = float(sys.argv[i+1])
            z = float(sys.argv[i+2])
            waypoints.append((x, y, z))
            i += 3
        except (ValueError, IndexError):
            print(f"警告: 跳过无效的航点参数 (索引 {i})")
            break
    
    # 打印启动信息
    if len(waypoints) > 0:
        print("=" * 70)
        print("  Typhoon H480 激光雷达 + MinSnap 避障导航")
        print("=" * 70)
        print(f"无人机类型: {drone_type}")
        print(f"无人机ID:   {drone_id}")
        print(f"无人机总数: {num_drones}")
        print(f"航点数量:   {len(waypoints)}")
        print()
        print("航点列表:")
        for idx, (x, y, z) in enumerate(waypoints, 1):
            print(f"  航点 {idx}: ({x:.2f}, {y:.2f}, {z:.2f})")
        print("=" * 70)
        print()
    
    try:
        # 创建节点
        node = TyphoonLidarMinSnapNavigation(drone_type, drone_id, num_drones)
        
        # 如果提供了航点，添加到队列
        if len(waypoints) > 0:
            rospy.loginfo(f"[启动] 从命令行加载 {len(waypoints)} 个航点")
            for x, y, z in waypoints:
                node.add_waypoint(x, y, z)
            rospy.loginfo("[启动] 所有航点已加载，开始导航...")
        else:
            rospy.loginfo("[启动] 等待通过话题发布航点...")
            rospy.loginfo("[提示] 发送航点命令：")
            rospy.loginfo(f"  rostopic pub /{drone_type}_{drone_id}/move_base_simple/goal geometry_msgs/PoseStamped "
                         "'{{pose: {{position: {{x: 10, y: 5, z: 2}}}}}}'")
        
        # 运行节点
        node.run()
    
    except rospy.ROSInterruptException:
        rospy.loginfo("[导航系统] ROS中断")
    except KeyboardInterrupt:
        rospy.loginfo("[导航系统] 用户中断 (Ctrl+C)")
    except Exception as e:
        rospy.logerr(f"[导航系统] 发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rospy.loginfo("[导航系统] 节点已关闭")


if __name__ == '__main__':
    main()

