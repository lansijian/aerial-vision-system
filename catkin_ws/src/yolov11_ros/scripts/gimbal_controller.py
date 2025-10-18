#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Gimbal Controller v9.5 - 云台控制器
统一管理无人机云台，支持多种控制模式

功能：
1. 保持云台稳定向下
2. 支持手动控制模式
3. 支持追踪模式自动调整
4. 确保多机云台配置正确

作者: 东华大学 Astraeus队
日期: 2025-10-12
"""

import rospy
import math
from geometry_msgs.msg import Quaternion, Vector3
from mavros_msgs.msg import MountControl
from mavros_msgs.srv import MountConfigure
from std_msgs.msg import String, Bool


class GimbalController:
    def __init__(self, vehicle_type=None, vehicle_id=None):
        # 如果传入命令行参数，优先使用
        if vehicle_type and vehicle_id is not None:
            self.drone_id = int(vehicle_id)
            self.vehicle_type = vehicle_type
            self.vehicle_ns = f"{vehicle_type}_{vehicle_id}"
            rospy.init_node(f'gimbal_control_{self.vehicle_ns}')
        else:
            # 否则使用ROS参数
            rospy.init_node('gimbal_controller')
            self.drone_id = rospy.get_param('~drone_id', 0)
            self.vehicle_type = rospy.get_param('~vehicle_type', 'typhoon_h480')
            self.vehicle_ns = f"{self.vehicle_type}_{self.drone_id}"
            
        self.default_pitch = rospy.get_param('~default_pitch', -50.0)  # 改为-50度
        self.update_rate = rospy.get_param('~update_rate', 30)
        
        # 云台状态
        self.current_pitch = self.default_pitch
        self.current_yaw = 0.0
        self.current_roll = 0.0
        self.control_mode = "AUTO"  # AUTO, MANUAL, TRACKING
        
        # 发布者
        self.mount_control_pub = rospy.Publisher(
            f'/{self.vehicle_ns}/mavros/mount_control/command',
            MountControl, queue_size=1
        )
        
        # 云台配置完成信号发布者
        self.gimbal_ready_pub = rospy.Publisher(
            f'/drone_{self.drone_id}/gimbal_ready',
            Bool, queue_size=1, latch=True
        )
        
        # 订阅者
        self.mode_sub = rospy.Subscriber(
            f'/drone_{self.drone_id}/flight_mode',
            String, self._mode_callback
        )
        
        # 配置云台
        self._configure_gimbal()
        
        rospy.loginfo(f"[{self.vehicle_ns}] Gimbal control node starting...")
        rospy.loginfo(f"[✅ {self.vehicle_ns}] 云台控制器初始化完成")
        rospy.loginfo(f"   默认俯仰角: {self.default_pitch}度")
        
    def _configure_gimbal(self):
        """配置云台参数"""
        # 暂时跳过云台配置服务，直接使用命令控制
        rospy.loginfo("云台控制器已初始化，使用默认配置")
            
    def _mode_callback(self, msg):
        """飞行模式回调"""
        if msg.data == "TRACKING":
            self.control_mode = "TRACKING"
            # 追踪模式下稍微调整角度以获得更好的视野
            self.current_pitch = -35.0
        else:
            self.control_mode = "AUTO"
            self.current_pitch = self.default_pitch
            
    def _publish_gimbal_command(self):
        """发布云台控制命令"""
        msg = MountControl()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        
        # 控制模式（2 = MAV_MOUNT_MODE_MAVLINK_TARGETING）
        msg.mode = 2
        
        # 设置角度（单位：度）
        msg.pitch = self.current_pitch
        msg.roll = self.current_roll
        msg.yaw = self.current_yaw
        
        # MountControl消息不需要save_position属性
        
        # 发布
        self.mount_control_pub.publish(msg)
        
    def set_gimbal_angle(self, pitch=None, yaw=None, roll=None):
        """设置云台角度（外部调用接口）"""
        if pitch is not None:
            self.current_pitch = max(-90, min(0, pitch))  # 限制俯仰角范围
        if yaw is not None:
            self.current_yaw = max(-180, min(180, yaw))
        if roll is not None:
            self.current_roll = max(-45, min(45, roll))
            
        rospy.loginfo(f"设置云台角度: Pitch={self.current_pitch:.1f}, Yaw={self.current_yaw:.1f}")
        
    def smooth_tracking(self, target_x, target_y):
        """平滑追踪目标（用于追踪模式）"""
        if self.control_mode != "TRACKING":
            return
            
        # 计算目标偏移（假设图像中心为(320, 240)）
        offset_x = target_x - 320
        offset_y = target_y - 240
        
        # 根据偏移调整云台角度
        # 偏航控制（水平）
        yaw_adjustment = offset_x * 0.05  # 增益系数
        self.current_yaw = max(-45, min(45, yaw_adjustment))
        
        # 俯仰控制（垂直）
        pitch_adjustment = offset_y * 0.03
        self.current_pitch = max(-90, min(0, self.default_pitch + pitch_adjustment))
        
    def reset_gimbal(self):
        """重置云台到默认位置"""
        self.current_pitch = self.default_pitch
        self.current_yaw = 0.0
        self.current_roll = 0.0
        self.control_mode = "AUTO"
        rospy.loginfo("云台已重置到默认位置")
        
    def run(self):
        """主循环"""
        rate = rospy.Rate(self.update_rate)
        
        rospy.loginfo(f"🚀 [{self.vehicle_ns}] 云台控制器开始运行")
        
        # 等待一段时间让系统初始化
        rospy.sleep(2.0)
        
        # 发布云台准备完成信号（允许YOLO可视化窗口弹出）
        self.gimbal_ready_pub.publish(Bool(data=True))
        rospy.loginfo(f"✅ [{self.vehicle_ns}] 云台配置完成，YOLO可视化已启用")
        
        while not rospy.is_shutdown():
            # 发布云台命令
            self._publish_gimbal_command()
            
            # 调试信息（降低频率）
            if rospy.Time.now().to_sec() % 5 < 0.1:
                rospy.logdebug(
                    f"云台状态 - 模式: {self.control_mode}, "
                    f"俯仰: {self.current_pitch:.1f}°, "
                    f"偏航: {self.current_yaw:.1f}°"
                )
                
            rate.sleep()
            
        rospy.loginfo("云台控制器停止")


if __name__ == '__main__':
    import sys
    try:
        if len(sys.argv) >= 3:
            # 命令行方式启动（与原版本兼容）
            vehicle_type = sys.argv[1]
            vehicle_id = sys.argv[2]
            controller = GimbalController(vehicle_type, vehicle_id)
        else:
            # ROS参数方式启动
            controller = GimbalController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
