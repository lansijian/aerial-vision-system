#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
云台控制节点 - 增强版，支持多机同时启动
功能：30Hz持续发送云台控制指令，保持俯仰角-45°
改进：
1. 增加服务等待机制，确保服务就绪后再调用
2. 增加重试机制，提高配置成功率
3. 增强异常处理和日志输出
4. 支持6台无人机同时启动而不冲突
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import MountControl
from mavros_msgs.srv import MountConfigure
from gazebo_msgs.srv import GetLinkState
import sys
import std_msgs.msg

if __name__ == "__main__":
    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    vehicle_name = vehicle_type + '_' + vehicle_id
    
    # 初始化节点
    rospy.init_node('gimbal_control_' + vehicle_name)
    rospy.loginfo(f"[{vehicle_name}] Gimbal control node starting...")
    
    # 云台参数
    gimbal_pitch_ = -45
    gimbal_yaw_ = 0.0
    gimbal_roll_ = 0.0
    
    # ========== 第1步: 等待MAVROS云台配置服务 ==========
    mount_config_service = vehicle_name + '/mavros/mount_control/configure'
    rospy.loginfo(f"[{vehicle_name}] Waiting for service: {mount_config_service}")
    try:
        rospy.wait_for_service(mount_config_service, timeout=30.0)
        rospy.loginfo(f"[{vehicle_name}] Service {mount_config_service} is ready")
    except rospy.ROSException as e:
        rospy.logerr(f"[{vehicle_name}] Service {mount_config_service} not available: {e}")
        sys.exit(1)
    
    # ========== 第2步: 等待Gazebo链接状态服务 ==========
    gazebo_service = 'gazebo/get_link_state'
    rospy.loginfo(f"[{vehicle_name}] Waiting for Gazebo service: {gazebo_service}")
    try:
        rospy.wait_for_service(gazebo_service, timeout=30.0)
        rospy.loginfo(f"[{vehicle_name}] Gazebo service is ready")
    except rospy.ROSException as e:
        rospy.logwarn(f"[{vehicle_name}] Gazebo service not available, will continue without cam_pose: {e}")
    
    # ========== 第3步: 创建发布者和服务代理 ==========
    mountCnt = rospy.Publisher(vehicle_name + '/mavros/mount_control/command', MountControl, queue_size=10)
    mountConfig = rospy.ServiceProxy(mount_config_service, MountConfigure)
    cam_pose_pub = rospy.Publisher('/xtdrone/' + vehicle_name + '/cam_pose', PoseStamped, queue_size=10)
    gazeboLinkstate = rospy.ServiceProxy(gazebo_service, GetLinkState)
    
    # 等待发布者连接（重要：确保订阅者建立）
    rospy.sleep(0.5)
    
    # ========== 第4步: 配置云台模式（增加重试机制） ==========
    srvheader = std_msgs.msg.Header()
    srvheader.stamp = rospy.Time.now()
    srvheader.frame_id = "map"
    
    max_retries = 5
    config_success = False
    
    for attempt in range(max_retries):
        try:
            rospy.loginfo(f"[{vehicle_name}] Configuring gimbal (attempt {attempt + 1}/{max_retries})...")
            mountConfig(header=srvheader, mode=2, stabilize_roll=0, stabilize_yaw=0, stabilize_pitch=0)
            rospy.loginfo(f"[{vehicle_name}] ✓ Gimbal configured successfully (pitch={gimbal_pitch_}°)")
            config_success = True
            break
        except Exception as e:
            rospy.logwarn(f"[{vehicle_name}] Gimbal config attempt {attempt + 1} failed: {e}")
            if attempt < max_retries - 1:
                rospy.sleep(0.5)  # 等待后重试
            else:
                rospy.logerr(f"[{vehicle_name}] ✗ Failed to configure gimbal after {max_retries} attempts")
    
    if not config_success:
        rospy.logerr(f"[{vehicle_name}] Gimbal configuration failed, but will continue sending control commands")
    
    # ========== 第5步: 强制发送控制指令确保配置生效 ==========
    # 关键：配置服务调用成功后，立即发送多次控制指令确保MAVROS真正应用配置
    rospy.loginfo(f"[{vehicle_name}] Sending initial control commands to ensure gimbal takes effect...")
    initial_rate = rospy.Rate(30)  # 30Hz
    for i in range(30):  # 发送1秒的控制指令（30次）
        msg = MountControl()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.mode = 2
        msg.pitch = gimbal_pitch_
        msg.roll = gimbal_roll_
        msg.yaw = gimbal_yaw_
        mountCnt.publish(msg)
        initial_rate.sleep()
    
    rospy.loginfo(f"[{vehicle_name}] ✓✓ Initial control commands sent, gimbal should be at {gimbal_pitch_}°")
    
    # ========== 第6步: 主循环 - 持续发送云台控制指令 ==========
    rate = rospy.Rate(30)  # 30Hz
    cam_pose = PoseStamped()
    loop_count = 0
    gazebo_error_count = 0
    max_gazebo_errors = 10  # 允许的最大Gazebo错误次数
    
    rospy.loginfo(f"[{vehicle_name}] Starting main control loop at 30Hz...")
    
    while not rospy.is_shutdown():
        # 发送云台控制指令
        msg = MountControl()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.mode = 2
        msg.pitch = gimbal_pitch_
        msg.roll = gimbal_roll_
        msg.yaw = gimbal_yaw_
        mountCnt.publish(msg)
        
        # 获取相机位姿（用于视觉伺服）
        try:
            response = gazeboLinkstate(vehicle_name + '::cgo3_camera_link', 'ground_plane::link')
            cam_pose.header.stamp = rospy.Time.now()
            cam_pose.pose = response.link_state.pose
            cam_pose_pub.publish(cam_pose)
            
            # 每300次循环打印一次状态（10秒一次）
            if loop_count % 300 == 0:
                rospy.loginfo(f"[{vehicle_name}] Gimbal running normally (pitch={gimbal_pitch_}°)")
            
            gazebo_error_count = 0  # 重置错误计数
            
        except Exception as e:
            gazebo_error_count += 1
            if gazebo_error_count <= max_gazebo_errors:
                # 只在前几次报错时打印警告
                if gazebo_error_count % 5 == 1:
                    rospy.logwarn(f"[{vehicle_name}] Gazebo link state error ({gazebo_error_count}): {e}")
            # 超过最大错误次数后静默处理，避免日志刷屏
        
        loop_count += 1
        rate.sleep()
    
    rospy.loginfo(f"[{vehicle_name}] Gimbal control node shutting down")


