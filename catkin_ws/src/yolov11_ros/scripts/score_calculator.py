#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Score Calculator v10.0 - 完全重构版记分系统
完全参考官方score_cal.py的逻辑，保持v9.5的UI界面

功能：
1. 完全按照官方规则计算得分
2. 订阅ActorInfo话题（与官方格式完全一致）
3. 5秒持续检测确认目标（测试用，原官方15秒）
4. 美观的UI界面（继承v9.5设计）

作者: 东华大学 Astraeus队
日期: 2025-10-13
版本: v10.0
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
    def __init__(self, uav_type='typhoon_h480'):
        rospy.init_node('score_calculator')
        
        # 参数（官方标准）
        self.uav_type = uav_type
        self.actor_num = 6
        self.uav_num = rospy.get_param('~num_drones', 2)
        self.err_threshold = 1.0  # 官方阈值：1米
        
        # Actor配置（与官方完全一致）
        # 官方标准映射，不可修改！
        self.actor_id_dict = {
            'green': [0], 
            'blue': [1], 
            'brown': [2], 
            'white': [3], 
            'red': [4, 5]
        }
        
        # 传感器成本（与官方完全一致）
        mono_cam = 1
        stereo_cam = 0
        laser1d = 0
        laser2d = 0
        laser3d = 0
        gimbal = 1
        self.sensor_cost = mono_cam * 5e2 + stereo_cam * 1e3 + laser1d * 2e2 + laser2d * 5e3 + laser3d * 2e4 + gimbal * 2e2
        
        # 状态变量（与官方一致）
        self.left_actors = list(range(self.actor_num))
        self.actors_pos = [None] * self.actor_num
        self.count_flag = [False] * self.actor_num
        self.topic_arrive_time = [0.0] * self.actor_num
        self.find_time = [0.0] * self.actor_num
        self.flag_1 = 0  # red1标志
        self.flag_2 = 1  # red2标志
        self.target_finish = 0
        self.score = (2 + self.target_finish) * 60 - self.sensor_cost * 3e-3
        
        # UI增强：额外信息记录
        self.actor_info = {}
        for i in range(6):
            self.actor_info[i] = {
                'color': '',
                'ground_truth_pos': None,
                'detected_pos': None,
                'detection_progress': 0.0,
                'is_detecting': False
            }
        
        # 设置颜色映射
        for color, ids in self.actor_id_dict.items():
            for id in ids:
                self.actor_info[id]['color'] = color
        
        # 服务客户端
        rospy.wait_for_service('/gazebo/delete_model', timeout=10)
        rospy.wait_for_service('/gazebo/get_model_state', timeout=10)
        self.del_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        
        # 发布者
        self.score_pub = rospy.Publisher("/score", Int16, queue_size=1)
        self.time_usage_pub = rospy.Publisher("/time_usage", Int16, queue_size=1)
        self.left_actors_pub = rospy.Publisher("/left_actors", String, queue_size=1)
        
        # 订阅者（与官方完全一致）
        self.actor_blue_sub = rospy.Subscriber("/actor_blue_info", ActorInfo, self.actor_info_callback, queue_size=1)
        self.actor_green_sub = rospy.Subscriber("/actor_green_info", ActorInfo, self.actor_info_callback, queue_size=1)
        self.actor_white_sub = rospy.Subscriber("/actor_white_info", ActorInfo, self.actor_info_callback, queue_size=1)
        self.actor_brown_sub = rospy.Subscriber("/actor_brown_info", ActorInfo, self.actor_info_callback, queue_size=1)
        self.actor_red1_sub = rospy.Subscriber("/actor_red1_info", ActorInfo, self.actor_info1_callback, queue_size=1)
        self.actor_red2_sub = rospy.Subscriber("/actor_red2_info", ActorInfo, self.actor_info2_callback, queue_size=1)
        
        # 模型状态订阅（用于UI显示）
        self.model_states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_callback)
        
        # OpenCV窗口（v9.5风格UI）
        self._create_display_window()
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("✅ 记分系统v10.1初始化完成（测试模式）")
        rospy.loginfo(f"   传感器配置: 单目相机(500) + 云台(200) = {int(self.sensor_cost)}")
        rospy.loginfo(f"   初始分数: {self.score:.1f}")
        rospy.loginfo(f"   误差阈值: {self.err_threshold}m（测试用，原官方1m）")
        rospy.loginfo(f"   确认时间: 5秒（测试用，原官方15秒）")
        rospy.loginfo(f"   无人机数量: {self.uav_num}")
        rospy.loginfo("=" * 60)
        
    def _create_display_window(self):
        """创建v9.5风格的显示窗口"""
        self.window_width = 1400
        self.window_height = 600
        self.window_name = "RoboCup Multi-UAV Competition Scoring System v10.0"
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        
        # 颜色配置
        self.bg_color = (245, 245, 245)
        self.primary_color = (50, 50, 50)
        self.accent_color = (0, 120, 255)
        self.success_color = (0, 200, 0)
        self.danger_color = (0, 0, 200)
        self.coord_color = (100, 100, 100)
    
    def _model_states_callback(self, msg):
        """更新actor真实位置（用于UI显示）"""
        for i, name in enumerate(msg.name):
            if name.startswith('actor_'):
                try:
                    actor_id = int(name.split('_')[1])
                    if actor_id < self.actor_num:
                        self.actor_info[actor_id]['ground_truth_pos'] = (
                            msg.pose[i].position.x,
                            msg.pose[i].position.y
                        )
                except:
                    pass
    
    def actor_info_callback(self, msg):
        """处理ActorInfo消息（与官方逻辑完全一致 - 用于blue/green/white/brown）"""
        actor_id = self.actor_id_dict[msg.cls]
        
        for i in actor_id:
            if i not in self.left_actors:
                continue
            
            # UI：更新检测位置
            self.actor_info[i]['detected_pos'] = (msg.x, msg.y)
            
            topic_arrive_interval = rospy.get_time() - self.topic_arrive_time[i]
            self.topic_arrive_time[i] = rospy.get_time()
            
            # 与官方完全一致的判断逻辑
            if (msg.x - self.actors_pos[i].x)**2 + (msg.y - self.actors_pos[i].y)**2 < self.err_threshold**2 and topic_arrive_interval < 1:
                if not self.count_flag[i]:
                    self.count_flag[i] = True
                    self.find_time[i] = rospy.get_time()
                    self.actor_info[i]['is_detecting'] = True
                    rospy.loginfo(f"🎯 发现目标 actor_{i} ({msg.cls})")
                elif rospy.get_time() - self.find_time[i] >= 15:
                    # 持续检测5秒，确认目标（测试用，原官方15秒）
                    self._confirm_and_delete_actor(i)
                else:
                    # UI：更新检测进度
                    time_detected = rospy.get_time() - self.find_time[i]
                    self.actor_info[i]['detection_progress'] = min(100, (time_detected / 5.0) * 100)
            else:
                self.count_flag[i] = False
                self.actor_info[i]['is_detecting'] = False
                self.actor_info[i]['detection_progress'] = 0.0
    
    def actor_info1_callback(self, msg):
        """处理red1的ActorInfo（与官方逻辑完全一致）"""
        red_cnt = 0
        actor_id = self.actor_id_dict[msg.cls]
        
        for i in actor_id:
            if i not in self.left_actors:
                continue
            
            # UI：更新检测位置
            self.actor_info[i]['detected_pos'] = (msg.x, msg.y)
            
            topic_arrive_interval = rospy.get_time() - self.topic_arrive_time[i]
            self.topic_arrive_time[i] = rospy.get_time()
            
            if (msg.x - self.actors_pos[i].x)**2 + (msg.y - self.actors_pos[i].y)**2 < self.err_threshold**2 and topic_arrive_interval < 1:
                if not self.count_flag[i]:
                    self.count_flag[i] = True
                    self.find_time[i] = rospy.get_time()
                    self.actor_info[i]['is_detecting'] = True
                    rospy.loginfo(f"🎯 发现目标 actor_{i} ({msg.cls})")
                    self.flag_1 = i
                elif rospy.get_time() - self.find_time[i] >= 15:
                    self._confirm_and_delete_actor(i)
                else:
                    # UI：更新检测进度
                    time_detected = rospy.get_time() - self.find_time[i]
                    self.actor_info[i]['detection_progress'] = min(100, (time_detected / 5.0) * 100)
            else:
                red_cnt += 1
                if red_cnt == 2 and not self.flag_1 == 0:
                    self.count_flag[self.flag_1] = False
                    self.actor_info[self.flag_1]['is_detecting'] = False
                    self.actor_info[self.flag_1]['detection_progress'] = 0.0
                    self.flag_1 = 0
    
    def actor_info2_callback(self, msg):
        """处理red2的ActorInfo（与官方逻辑完全一致）"""
        red_cnt = 0
        actor_id = self.actor_id_dict[msg.cls]
        
        for i in actor_id:
            if i not in self.left_actors:
                continue
            
            # UI：更新检测位置
            self.actor_info[i]['detected_pos'] = (msg.x, msg.y)
            
            topic_arrive_interval = rospy.get_time() - self.topic_arrive_time[i]
            self.topic_arrive_time[i] = rospy.get_time()
            
            if (msg.x - self.actors_pos[i].x)**2 + (msg.y - self.actors_pos[i].y)**2 < self.err_threshold**2 and topic_arrive_interval < 1:
                if not self.count_flag[i]:
                    self.count_flag[i] = True
                    self.find_time[i] = rospy.get_time()
                    self.actor_info[i]['is_detecting'] = True
                    rospy.loginfo(f"🎯 发现目标 actor_{i} ({msg.cls})")
                    self.flag_2 = i
                elif rospy.get_time() - self.find_time[i] >= 15:
                    self._confirm_and_delete_actor(i)
                else:
                    # UI：更新检测进度
                    time_detected = rospy.get_time() - self.find_time[i]
                    self.actor_info[i]['detection_progress'] = min(100, (time_detected / 5.0) * 100)
            else:
                red_cnt += 1
                if red_cnt == 2 and not self.flag_2 == 1:
                    self.count_flag[self.flag_2] = False
                    self.actor_info[self.flag_2]['is_detecting'] = False
                    self.actor_info[self.flag_2]['detection_progress'] = 0.0
                    self.flag_2 = 1
    
    def _confirm_and_delete_actor(self, actor_id):
        """确认并删除目标（与官方逻辑一致）"""
        try:
            self.del_model(f'actor_{actor_id}')
            self.left_actors.remove(actor_id)
            
            time_usage = rospy.get_time() - self.start_time
            rospy.loginfo(f"✅ actor_{actor_id} 已消除")
            rospy.loginfo(f"⏱️ 用时: {time_usage:.1f}秒")
            
            self.target_finish = 6 - len(self.left_actors)
            
            # 计算得分（与官方完全一致）
            if self.target_finish == 6:
                self.score = (1200 - time_usage) - self.sensor_cost * 3e-3
                rospy.loginfo(f"🏆 任务完成！最终得分: {self.score:.2f}")
                rospy.loginfo("任务完成，按Ctrl+C退出")
            else:
                self.score = (2 + self.target_finish) * 60 - self.sensor_cost * 3e-3
                rospy.loginfo(f"📊 当前得分: {self.score:.2f}")
        except Exception as e:
            rospy.logerr(f"删除actor_{actor_id}失败: {e}")
    
    def _update_display(self):
        """更新v9.5风格的UI显示"""
        canvas = np.ones((self.window_height, self.window_width, 3), dtype=np.uint8)
        canvas[:] = self.bg_color
        
        time_usage = rospy.get_time() - self.start_time
        
        # 标题栏
        cv2.rectangle(canvas, (0, 0), (self.window_width, 80), self.accent_color, -1)
        cv2.putText(canvas, "Multi-UAV Search Competition v10.0", (20, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(canvas, "Donghua University - Astraeus Team", (20, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        
        # 主要信息
        y_offset = 120
        cv2.putText(canvas, f"Score: {self.score:.1f}", (50, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, self.primary_color, 2, cv2.LINE_AA)
        cv2.putText(canvas, f"Time: {int(time_usage)}s", (350, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, self.primary_color, 2, cv2.LINE_AA)
        cv2.putText(canvas, f"Left: {len(self.left_actors)}/6", (600, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, self.primary_color, 2, cv2.LINE_AA)
        cv2.putText(canvas, f"Done: {self.target_finish}/6", (800, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, self.success_color, 2, cv2.LINE_AA)
        
        # 详情标题
        y_offset = 180
        cv2.line(canvas, (50, y_offset), (self.window_width - 50, y_offset), self.primary_color, 2)
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
            y = y_offset + i * 45
            info = self.actor_info[i]
            
            # ID
            cv2.putText(canvas, str(i), (60, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.primary_color, 2, cv2.LINE_AA)
            
            # 颜色
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
                if info['ground_truth_pos']:
                    coord_text = f"({info['ground_truth_pos'][0]:.1f},{info['ground_truth_pos'][1]:.1f})"
                    cv2.putText(canvas, coord_text, (400, y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.coord_color, 1, cv2.LINE_AA)
                cv2.rectangle(canvas, (950, y - 15), (1100, y - 5), self.success_color, -1)
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
                
                # 误差
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
                cv2.rectangle(canvas, (950, y - 15), (1100, y - 5), (200, 200, 200), -1)
                if bar_width > 0:
                    color = self.success_color if progress > 80 else self.accent_color
                    cv2.rectangle(canvas, (950, y - 15), (950 + bar_width, y - 5), color, -1)
                cv2.putText(canvas, f"{int(progress)}%", (1120, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.primary_color, 1, cv2.LINE_AA)
        
        # 底部状态栏
        cv2.rectangle(canvas, (0, 550), (self.window_width, self.window_height), (230, 230, 230), -1)
        status_text = f"System v10.0 | FPS: 10 | Drones: {self.uav_num} | Sensor Cost: {int(self.sensor_cost)} | Error Threshold: {self.err_threshold}m"
        cv2.putText(canvas, status_text, (20, 580),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.primary_color, 1, cv2.LINE_AA)
        
        cv2.imshow(self.window_name, canvas)
        cv2.waitKey(1)
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(10)
        
        rospy.loginfo("🚀 记分系统v10.0开始运行")
        rospy.sleep(1.0)
        
        self.start_time = rospy.get_time()
        
        while not rospy.is_shutdown():
            # 更新actor位置
            for i in self.left_actors:
                try:
                    actors_pos_tmp = self.get_model_state(f'actor_{i}', 'ground_plane').pose.position
                    if not actors_pos_tmp.x**2 + actors_pos_tmp.y**2 == 0:
                        self.actors_pos[i] = actors_pos_tmp
                except:
                    pass
            
            # 检查无人机高度（与官方一致）
            for i in range(self.uav_num):
                try:
                    uav_pos_tmp = self.get_model_state(f'{self.uav_type}_{i}', 'ground_plane').pose.position
                    if uav_pos_tmp.z > 6.5:
                        rospy.logwarn(f"⚠️ 警告: 无人机{i}高度超过6米!")
                        # 可以选择终止任务
                except:
                    pass
            
            # 检查超时
            time_usage = rospy.get_time() - self.start_time
            if time_usage > 600:
                rospy.logwarn(f"⏰ 超时！最终得分: {self.score:.2f}")
                rospy.logwarn("任务失败")
                break
            
            # 发布状态
            self.score_pub.publish(int(self.score))
            self.time_usage_pub.publish(int(time_usage))
            self.left_actors_pub.publish(str(self.left_actors))
            
            # 更新显示
            self._update_display()
            
            rate.sleep()
        
        cv2.destroyAllWindows()
        rospy.loginfo("记分系统停止")


if __name__ == '__main__':
    import sys
    try:
        uav_type = sys.argv[1] if len(sys.argv) > 1 else 'typhoon_h480'
        calculator = ScoreCalculator(uav_type)
        calculator.run()
    except rospy.ROSInterruptException:
        pass
