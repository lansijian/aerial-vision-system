#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Score Calculator v9.5 - 多机协同记分系统
基于官方score_cal.py，适配双机协同，优化UI显示

功能：
1. 订阅多架无人机的ActorInfo消息
2. 验证目标检测（15秒持续检测）
3. 计算比赛得分
4. 与Gazebo交互删除已确认目标
5. 美观的UI界面

作者: 东华大学 Astraeus队
日期: 2025-10-12
"""

import rospy
import sys
import time
import cv2
import numpy as np
from std_msgs.msg import Int16, String
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
from gazebo_msgs.srv import DeleteModel, GetModelState
from gazebo_msgs.msg import ModelStates


class ScoreCalculator:
    def __init__(self, vehicle_type=None):
        rospy.init_node('score_calculator')
        
        # 如果传入vehicle_type，存储以备后用
        self.vehicle_type = vehicle_type if vehicle_type else 'typhoon_h480'
        
        # 参数
        self.actor_num = rospy.get_param('~actor_num', 6)
        self.err_threshold = rospy.get_param('~err_threshold', 1.0)
        self.detection_time = rospy.get_param('~detection_time', 15.0)
        self.timeout_sec = rospy.get_param('~timeout_sec', 600)
        self.num_drones = rospy.get_param('~num_drones', 2)
        
        # Actor配置（根据官方规则）
        self.actor_id_dict = {
            'green': [0],
            'blue': [1],
            'brown': [2],
            'white': [3],
            'red': [4, 5]
        }
        
        # 传感器成本（根据官方规则）
        self.sensor_cost = {
            'mono_cam': 500,      # 单目相机
            'stereo_cam': 1000,   # 双目相机
            'laser1d': 200,       # 1D激光
            'laser2d': 5000,      # 2D激光
            'laser3d': 20000,     # 3D激光
            'gimbal': 200         # 云台
        }
        
        # 使用的传感器
        self.sensors_used = {
            'mono_cam': 1,
            'stereo_cam': 0,
            'laser1d': 0,
            'laser2d': 0,
            'laser3d': 0,
            'gimbal': 1
        }
        
        # 计算传感器总成本
        self.total_sensor_cost = sum(
            self.sensors_used[sensor] * cost 
            for sensor, cost in self.sensor_cost.items()
        )
        
        # 状态变量
        self.left_actors = list(range(self.actor_num))
        self.actors_pos = [None] * self.actor_num
        self.count_flag = [False] * self.actor_num
        self.find_time = [0.0] * self.actor_num
        self.topic_arrive_time = [0.0] * self.actor_num
        self.target_finish = 0
        self.start_time = rospy.Time.now()
        
        # 计算初始分数（与原版一致）
        self.score = (2 + self.target_finish) * 60 - self.total_sensor_cost * 3e-3
        rospy.loginfo(f"初始分数: {self.score:.1f}")
        rospy.loginfo(f"传感器成本计算: mono_cam(500) + gimbal(200) = {self.total_sensor_cost}")
        rospy.loginfo(f"分数计算: (2+0)*60 - {self.total_sensor_cost}*0.003 = {self.score:.1f}")
        
        # 多机协同记录
        self.drone_detections = {}  # {actor_id: {drone_id: last_detection_time}}
        
        # 记录每个目标的详细信息
        self.actor_info = {}
        for i in range(6):
            self.actor_info[i] = {
                'color': '',
                'ground_truth_pos': None,
                'detected_pos': None,
                'detection_progress': 0.0,  # 0-100%
                'is_detecting': False,
                'detection_start_time': None
            }
            
        # 设置颜色映射
        for color, ids in self.actor_id_dict.items():
            for id in ids:
                self.actor_info[id]['color'] = color
        
        # 服务客户端
        self.del_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        
        # 发布者
        self.score_pub = rospy.Publisher("/score", Int16, queue_size=1)
        self.time_usage_pub = rospy.Publisher("/time_usage", Int16, queue_size=1)
        self.left_actors_pub = rospy.Publisher("/left_actors", String, queue_size=1)
        
        # 订阅者 - 订阅所有颜色的actor信息
        self.actor_subs = {}
        for color in ['blue', 'green', 'white', 'brown', 'red', 'red1']:
            if color == 'red1':
                # 特殊处理red1
                self.actor_subs[color] = rospy.Subscriber(
                    f"/actor_{color}_info", ActorInfo,
                    self._actor_info_callback_red1
                )
            else:
                self.actor_subs[color] = rospy.Subscriber(
                    f"/actor_{color}_info", ActorInfo,
                    self._actor_info_callback,
                    callback_args=color
                )
                
        # 模型状态订阅（用于获取actor真实位置）
        self.model_states_sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates,
            self._model_states_callback
        )
        
        # OpenCV窗口（显示得分）
        self._create_display_window()
        
        rospy.loginfo("✅ 记分系统初始化完成")
        rospy.loginfo(f"   目标数量: {self.actor_num}")
        rospy.loginfo(f"   传感器成本: {self.total_sensor_cost}")
        rospy.loginfo(f"   初始分数: {self.score:.1f}")
        
    def _create_display_window(self):
        """创建显示窗口（优化版UI）"""
        # 固定窗口大小（增加宽度以显示坐标信息）
        self.window_width = 1400
        self.window_height = 600
        
        # 创建窗口（固定大小，不可调整）
        self.window_name = "RoboCup Multi-UAV Competition Scoring System"
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        
        # 预设颜色
        self.bg_color = (245, 245, 245)  # 浅灰色背景
        self.primary_color = (50, 50, 50)  # 深灰色文字
        self.accent_color = (0, 120, 255)  # 橙色强调色
        self.success_color = (0, 200, 0)  # 绿色
        self.danger_color = (0, 0, 200)  # 红色
        self.coord_color = (100, 100, 100)  # 坐标文字颜色
        
    def _model_states_callback(self, msg):
        """更新actor真实位置"""
        for i, name in enumerate(msg.name):
            if name.startswith('actor_'):
                try:
                    actor_id = int(name.split('_')[1])
                    if actor_id < self.actor_num:
                        self.actors_pos[actor_id] = msg.pose[i].position
                        # 更新地面真实位置
                        self.actor_info[actor_id]['ground_truth_pos'] = (
                            msg.pose[i].position.x, 
                            msg.pose[i].position.y
                        )
                except:
                    pass
                    
    def _actor_info_callback(self, msg, color):
        """处理ActorInfo消息（通用）"""
        actor_ids = self.actor_id_dict.get(color, [])
        
        for actor_id in actor_ids:
            if actor_id not in self.left_actors:
                continue
                
            # 更新检测位置
            self.actor_info[actor_id]['detected_pos'] = (msg.x, msg.y)
            self._process_detection(actor_id, msg)
            
    def _actor_info_callback_red1(self, msg):
        """处理红色恐怖分子的特殊情况"""
        # red1对应actor_4或actor_5
        for actor_id in [4, 5]:
            if actor_id not in self.left_actors:
                continue
                
            # 更新检测位置
            self.actor_info[actor_id]['detected_pos'] = (msg.x, msg.y)
            self._process_detection(actor_id, msg)
            
    def _process_detection(self, actor_id, msg):
        """处理单个检测"""
        if self.actors_pos[actor_id] is None:
            return
            
        # 检查位置匹配
        distance_sq = (
            (msg.x - self.actors_pos[actor_id].x) ** 2 +
            (msg.y - self.actors_pos[actor_id].y) ** 2
        )
        
        topic_arrive_interval = rospy.Time.now().to_sec() - self.topic_arrive_time[actor_id]
        self.topic_arrive_time[actor_id] = rospy.Time.now().to_sec()
        
        if distance_sq < self.err_threshold ** 2 and topic_arrive_interval < 1:
            if not self.count_flag[actor_id]:
                # 首次检测到
                self.count_flag[actor_id] = True
                self.find_time[actor_id] = rospy.Time.now().to_sec()
                self.actor_info[actor_id]['is_detecting'] = True
                self.actor_info[actor_id]['detection_start_time'] = rospy.Time.now().to_sec()
                rospy.loginfo(f"🎯 发现目标 actor_{actor_id} ({msg.cls})")
                
            else:
                # 更新检测进度
                time_detected = rospy.Time.now().to_sec() - self.find_time[actor_id]
                progress = min(100, (time_detected / self.detection_time) * 100)
                self.actor_info[actor_id]['detection_progress'] = progress
                
                if time_detected >= self.detection_time:
                    # 持续检测15秒，确认目标
                    self._confirm_target(actor_id)
        else:
            # 目标丢失，重置计数
            self.count_flag[actor_id] = False
            self.actor_info[actor_id]['is_detecting'] = False
            self.actor_info[actor_id]['detection_progress'] = 0.0
            
    def _confirm_target(self, actor_id):
        """确认并删除目标"""
        try:
            # 删除模型
            self.del_model(f'actor_{actor_id}')
            self.left_actors.remove(actor_id)
            
            time_usage = (rospy.Time.now() - self.start_time).to_sec()
            rospy.loginfo(f"✅ actor_{actor_id} 已确认并删除")
            rospy.loginfo(f"⏱️ 用时: {time_usage:.1f}秒")
            
            self.target_finish = self.actor_num - len(self.left_actors)
            
            # 计算得分
            self._calculate_score()
            
        except Exception as e:
            rospy.logerr(f"删除actor_{actor_id}失败: {e}")
            
    def _calculate_score(self):
        """计算得分（与原版一致）"""
        time_usage = (rospy.Time.now() - self.start_time).to_sec()
        
        if self.target_finish == self.actor_num:
            # 完成所有目标
            self.score = (1200 - time_usage) - self.total_sensor_cost * 3e-3
            rospy.loginfo(f"🏆 任务完成！最终得分: {self.score:.2f}")
            rospy.loginfo("按Ctrl+C退出程序")
        else:
            # 部分完成
            self.score = (2 + self.target_finish) * 60 - self.total_sensor_cost * 3e-3
            
        rospy.loginfo(f"📊 当前得分: {self.score:.2f}")
        
    def _check_drone_positions(self):
        """检查无人机高度限制"""
        for i in range(self.num_drones):
            try:
                uav_state = self.get_model_state(f'typhoon_h480_{i}', 'ground_plane')
                if uav_state.pose.position.z > 6.5:
                    rospy.logwarn(f"⚠️ 警告: 无人机{i}高度超过6米!")
                    # 可以选择结束任务或降低高度
            except:
                pass
                
    def _update_display(self):
        """更新显示（优化版UI）"""
        # 创建画布
        canvas = np.ones((self.window_height, self.window_width, 3), dtype=np.uint8)
        canvas[:] = self.bg_color
        
        # 计算时间
        time_usage = (rospy.Time.now() - self.start_time).to_sec()
        
        # 绘制标题栏
        cv2.rectangle(canvas, (0, 0), (self.window_width, 80), self.accent_color, -1)
        cv2.putText(canvas, "Multi-UAV Search Competition", (20, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(canvas, "Donghua University - Astraeus Team", (20, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        
        # 主要信息区域（大字体）
        y_offset = 120
        # 分数
        cv2.putText(canvas, f"Score: {self.score:.1f}", (50, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, self.primary_color, 2, cv2.LINE_AA)
        # 时间
        cv2.putText(canvas, f"Time: {int(time_usage)}s", (350, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, self.primary_color, 2, cv2.LINE_AA)
        # 剩余目标
        cv2.putText(canvas, f"Left: {len(self.left_actors)}/6", (600, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, self.primary_color, 2, cv2.LINE_AA)
        # 完成进度
        cv2.putText(canvas, f"Done: {self.target_finish}/6", (800, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, self.success_color, 2, cv2.LINE_AA)
        
        # 目标详情标题
        y_offset = 180
        cv2.line(canvas, (50, y_offset), (self.window_width-50, y_offset), self.primary_color, 2)
        y_offset += 30
        cv2.putText(canvas, "Target Details", (50, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, self.primary_color, 2, cv2.LINE_AA)
        
        # 表头
        y_offset += 40
        headers = ["ID", "Color", "Status", "Ground Truth (X,Y)", "Detected (X,Y)", "Error", "Progress"]
        x_positions = [50, 120, 250, 400, 600, 800, 950]
        for header, x in zip(headers, x_positions):
            font_size = 0.5 if len(header) > 10 else 0.6
            cv2.putText(canvas, header, (x, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, font_size, self.primary_color, 2, cv2.LINE_AA)
        
        # 目标信息
        y_offset += 30
        for i in range(6):
            y = y_offset + i * 45  # 增加行间距
            info = self.actor_info[i]
            
            # ID
            cv2.putText(canvas, str(i), (60, y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.primary_color, 2, cv2.LINE_AA)
            
            # 颜色（用对应颜色显示）
            color_bgr = {
                'blue': (255, 100, 0),
                'green': (0, 255, 0),
                'red': (0, 0, 255),
                'white': (200, 200, 200),
                'brown': (42, 42, 165)
            }.get(info['color'], (100, 100, 100))
            cv2.putText(canvas, info['color'].upper()[:5], (120, y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2, cv2.LINE_AA)
            
            # 状态
            if i not in self.left_actors:
                cv2.putText(canvas, "ELIMINATED", (250, y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.success_color, 2, cv2.LINE_AA)
                # 真实坐标（如果有）
                if info['ground_truth_pos']:
                    coord_text = f"({info['ground_truth_pos'][0]:.1f},{info['ground_truth_pos'][1]:.1f})"
                    cv2.putText(canvas, coord_text, (400, y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.coord_color, 1, cv2.LINE_AA)
                # 进度条 - 完成
                cv2.rectangle(canvas, (950, y-15), (1100, y-5), self.success_color, -1)
                cv2.putText(canvas, "100%", (1120, y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.success_color, 1, cv2.LINE_AA)
            else:
                if info['is_detecting']:
                    cv2.putText(canvas, "TRACKING", (250, y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.accent_color, 2, cv2.LINE_AA)
                else:
                    cv2.putText(canvas, "SEARCHING", (250, y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.primary_color, 1, cv2.LINE_AA)
                
                # 真实坐标
                if info['ground_truth_pos']:
                    coord_text = f"({info['ground_truth_pos'][0]:.1f},{info['ground_truth_pos'][1]:.1f})"
                    cv2.putText(canvas, coord_text, (400, y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.coord_color, 1, cv2.LINE_AA)
                else:
                    cv2.putText(canvas, "--", (400, y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.coord_color, 1, cv2.LINE_AA)
                
                # 检测坐标
                if info['detected_pos']:
                    coord_text = f"({info['detected_pos'][0]:.1f},{info['detected_pos'][1]:.1f})"
                    cv2.putText(canvas, coord_text, (600, y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.accent_color, 1, cv2.LINE_AA)
                else:
                    cv2.putText(canvas, "--", (600, y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.coord_color, 1, cv2.LINE_AA)
                
                # 误差计算
                if info['ground_truth_pos'] and info['detected_pos']:
                    error = np.sqrt((info['ground_truth_pos'][0] - info['detected_pos'][0])**2 + 
                                   (info['ground_truth_pos'][1] - info['detected_pos'][1])**2)
                    error_color = self.success_color if error < self.err_threshold else self.danger_color
                    cv2.putText(canvas, f"{error:.2f}m", (800, y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, error_color, 1, cv2.LINE_AA)
                else:
                    cv2.putText(canvas, "--", (800, y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.coord_color, 1, cv2.LINE_AA)
                
                # 进度条
                progress = info['detection_progress']
                bar_width = int(150 * progress / 100)
                # 背景
                cv2.rectangle(canvas, (950, y-15), (1100, y-5), (200, 200, 200), -1)
                # 进度
                if bar_width > 0:
                    color = self.success_color if progress > 80 else self.accent_color
                    cv2.rectangle(canvas, (950, y-15), (950 + bar_width, y-5), color, -1)
                # 百分比
                cv2.putText(canvas, f"{int(progress)}%", (1120, y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.primary_color, 1, cv2.LINE_AA)
        
        # 底部状态栏
        cv2.rectangle(canvas, (0, 550), (self.window_width, self.window_height), (230, 230, 230), -1)
        status_text = f"System Running | FPS: 30 | Drones: {self.num_drones} | Sensor Cost: {self.total_sensor_cost} | Error Threshold: {self.err_threshold}m"
        cv2.putText(canvas, status_text, (20, 580), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.primary_color, 1, cv2.LINE_AA)
        
        # 显示
        cv2.imshow(self.window_name, canvas)
        cv2.waitKey(1)
        
    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz
        
        rospy.loginfo("🚀 记分系统开始运行")
        rospy.sleep(1.0)  # 等待系统初始化
        
        self.start_time = rospy.Time.now()
        
        # 初始化分数
        self._calculate_score()
        
        while not rospy.is_shutdown():
            # 检查超时
            time_usage = (rospy.Time.now() - self.start_time).to_sec()
            if time_usage > self.timeout_sec:
                rospy.logwarn(f"⏰ 超时！最终得分: {self.score:.2f}")
                break
                
            # 检查无人机高度
            self._check_drone_positions()
            
            # 发布状态
            self.score_pub.publish(int(self.score))
            self.time_usage_pub.publish(int(time_usage))
            self.left_actors_pub.publish(str(self.left_actors))
            
            # 更新显示
            self._update_display()
            
            rate.sleep()
            
        # 清理
        cv2.destroyAllWindows()
        rospy.loginfo("记分系统停止")


if __name__ == '__main__':
    import sys
    try:
        # 兼容命令行参数（如果有）
        vehicle_type = sys.argv[1] if len(sys.argv) > 1 else None
        calculator = ScoreCalculator(vehicle_type)
        calculator.run()
    except rospy.ROSInterruptException:
        pass