#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RoboCup记分系统 - 完全按照官方XTDrone score_cal.py
仅增加UI进度显示
"""

import rospy
import sys
from std_msgs.msg import Int16, String
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
from gazebo_msgs.srv import DeleteModel, GetModelState
import time
import cv2
import numpy as np
import math

# 官方参数（完全一致）
uav_type = "typhoon_h480"
actor_num = 6
err_threshold = 1  # 官方：1米
detection_time = 15  # 官方：15秒
actor_id_dict = {'green': [0], 'blue': [1], 'brown': [2], 'white': [3], 'red': [4, 5]}

# 传感器成本（官方）
mono_cam = 1
stereo_cam = 0
laser1d = 0
laser2d = 0
laser3d = 0
gimbal = 1
sensor_cost = mono_cam * 5e2 + stereo_cam * 1e3 + laser1d * 2e2 + laser2d * 5e3 + laser3d * 2e4 + gimbal * 2e2

# 全局变量（官方）
left_actors = []
actors_pos = [None] * actor_num
count_flag = [False] * actor_num
topic_arrive_time = [0.0] * actor_num
find_time = [0.0] * actor_num
flag_1 = 0
flag_2 = 1
target_finish = 0
score = 0
start_time = 0
time_usage = 0

# ✅ 新增：记录无人机发布的检测坐标
detected_positions = {}  # {actor_id: {'x': float, 'y': float, 'timestamp': float}}

def actor_info_callback(msg):
    """官方回调函数 - 增加检测坐标记录"""
    global target_finish, start_time, score, count_flag, left_actors, actors_pos, find_time, topic_arrive_time, detected_positions
    actor_id = actor_id_dict[msg.cls]
    
    # ✅ 记录检测到的坐标
    for i in actor_id:
        detected_positions[i] = {
            'x': msg.x,
            'y': msg.y,
            'timestamp': rospy.get_time()
        }
    
    # 官方逻辑（不变）
    for i in actor_id:
        if i not in left_actors:
            continue
        topic_arrive_interval = rospy.get_time() - topic_arrive_time[i]
        topic_arrive_time[i] = rospy.get_time()
        if (msg.x - actors_pos[i].x)**2 + (msg.y - actors_pos[i].y)**2 < err_threshold**2 and topic_arrive_interval < 1:
            if not count_flag[i]:
                count_flag[i] = True
                find_time[i] = rospy.get_time()
                print("find actor_" + str(i))
            elif rospy.get_time() - find_time[i] >= detection_time:
                del_model('actor_' + str(i))
                left_actors.remove(i)
                print('actor_' + str(i) + ' is OK')
                print('Time usage:', time_usage)
                target_finish = 6 - len(left_actors)
                # calculate score
                if target_finish == 6:
                    score = (1200 - time_usage) - sensor_cost * 3e-3
                    print('score:', score)
                    print("Mission finished")
                    while True:
                        pass
                else:
                    score = (2 + target_finish) * 60 - sensor_cost * 3e-3
                    print('score:', score)
                    break
        else:
            count_flag[i] = False

def actor_info1_callback(msg):
    """官方红色1回调 - 增加检测坐标记录"""
    global target_finish, start_time, score, count_flag, left_actors, actors_pos, find_time, topic_arrive_time, flag_1, flag_2, detected_positions
    red_cnt = 0
    actor_id = actor_id_dict[msg.cls]
    
    # ✅ 记录检测到的坐标
    for i in actor_id:
        detected_positions[i] = {
            'x': msg.x,
            'y': msg.y,
            'timestamp': rospy.get_time()
        }
    
    # 官方逻辑（不变）
    for i in actor_id:
        if i not in left_actors:
            continue
        topic_arrive_interval = rospy.get_time() - topic_arrive_time[i]
        topic_arrive_time[i] = rospy.get_time()
        if (msg.x - actors_pos[i].x)**2 + (msg.y - actors_pos[i].y)**2 < err_threshold**2 and topic_arrive_interval < 1:
            if not count_flag[i]:
                count_flag[i] = True
                find_time[i] = rospy.get_time()
                print("find actor_" + str(i))
                flag_1 = i
            elif rospy.get_time() - find_time[i] >= detection_time:
                del_model('actor_' + str(i))
                left_actors.remove(i)
                print('actor_' + str(i) + ' is OK')
                print('Time usage:', time_usage)
                target_finish = 6 - len(left_actors)
                # calculate score
                if target_finish == 6:
                    score = (1200 - time_usage) - sensor_cost * 3e-3
                    print('score:', score)
                    print("Mission finished")
                    while True:
                        pass
                else:
                    score = (2 + target_finish) * 60 - sensor_cost * 3e-3
                    print('score:', score)
                    break
        else:
            red_cnt += 1
            if red_cnt == 2 and not flag_1 == 0:
                count_flag[flag_1] = False
                flag_1 = 0

def actor_info2_callback(msg):
    """官方红色2回调 - 增加检测坐标记录"""
    global target_finish, start_time, score, count_flag, left_actors, actors_pos, find_time, topic_arrive_time, flag_1, flag_2, detected_positions
    red_cnt = 0
    actor_id = actor_id_dict[msg.cls]
    
    # ✅ 记录检测到的坐标
    for i in actor_id:
        detected_positions[i] = {
            'x': msg.x,
            'y': msg.y,
            'timestamp': rospy.get_time()
        }
    
    # 官方逻辑（不变）
    for i in actor_id:
        if i not in left_actors:
            continue
        topic_arrive_interval = rospy.get_time() - topic_arrive_time[i]
        topic_arrive_time[i] = rospy.get_time()
        if (msg.x - actors_pos[i].x)**2 + (msg.y - actors_pos[i].y)**2 < err_threshold**2 and topic_arrive_interval < 1:
            if not count_flag[i]:
                count_flag[i] = True
                find_time[i] = rospy.get_time()
                print("find actor_" + str(i))
                flag_2 = i
            elif rospy.get_time() - find_time[i] >= detection_time:
                del_model('actor_' + str(i))
                left_actors.remove(i)
                print('actor_' + str(i) + ' is OK')
                print('Time usage:', time_usage)
                target_finish = 6 - len(left_actors)
                # calculate score
                if target_finish == 6:
                    score = (1200 - time_usage) - sensor_cost * 3e-3
                    print('score:', score)
                    print("Mission finished")
                    while True:
                        pass
                else:
                    score = (2 + target_finish) * 60 - sensor_cost * 3e-3
                    print('score:', score)
                    break
        else:
            red_cnt += 1
            if red_cnt == 2 and not flag_2 == 1:
                count_flag[flag_2] = False
                flag_2 = 1

def draw_ui():
    """绘制UI（增加坐标对比和误差显示）"""
    canvas = np.ones((900, 1400, 3), dtype=np.uint8) * 255  # 扩大画布
    
    # 标题
    cv2.putText(canvas, "RoboCup Score System - Coordinate Debug", (50, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 2)
    
    # 状态栏
    cv2.putText(canvas, f"Score: {int(score)}", (50, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 150, 0), 2)
    cv2.putText(canvas, f"Time: {int(time_usage)}s/600s", (300, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 200), 2)
    cv2.putText(canvas, f"Left: {len(left_actors)}/6", (600, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 0, 0), 2)
    
    # 表头
    y = 120
    cv2.putText(canvas, "ID", (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    cv2.putText(canvas, "Status", (100, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    cv2.putText(canvas, "Real Pos (x,y)", (280, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    cv2.putText(canvas, "Detected Pos (x,y)", (520, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    cv2.putText(canvas, "Error", (780, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    cv2.putText(canvas, "Progress", (900, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    cv2.line(canvas, (20, y+10), (1380, y+10), (150, 150, 150), 2)
    
    # Actor列表
    y = 160
    colors = ['GREEN', 'BLUE', 'BROWN', 'WHITE', 'RED1', 'RED2']
    for i in range(6):
        # 状态
        if i not in left_actors:
            status = "DONE"
            color = (0, 200, 0)
            progress = 1.0
        elif count_flag[i]:
            elapsed = rospy.get_time() - find_time[i]
            progress = elapsed / detection_time
            status = f"TRACK {elapsed:.1f}s"
            color = (0, 150, 200)
        else:
            status = "SEARCH"
            progress = 0.0
            color = (100, 100, 100)
        
        # ID和颜色
        cv2.putText(canvas, f"{i}", (40, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        cv2.putText(canvas, f"[{colors[i]}]", (100, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        cv2.putText(canvas, status, (200, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # 真实坐标
        if actors_pos[i] is not None:
            real_x, real_y = actors_pos[i].x, actors_pos[i].y
            cv2.putText(canvas, f"({real_x:.2f}, {real_y:.2f})", (280, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 100, 0), 1)
        else:
            cv2.putText(canvas, "N/A", (320, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
        
        # 检测坐标（从最近接收到的ActorInfo）
        detected_info = detected_positions.get(i, None)
        if detected_info:
            det_x, det_y = detected_info['x'], detected_info['y']
            cv2.putText(canvas, f"({det_x:.2f}, {det_y:.2f})", (520, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 150), 1)
            
            # 计算误差
            if actors_pos[i] is not None:
                error = math.sqrt((det_x - real_x)**2 + (det_y - real_y)**2)
                error_color = (0, 200, 0) if error < err_threshold else (0, 0, 200)
                cv2.putText(canvas, f"{error:.2f}m", (780, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, error_color, 2)
        else:
            cv2.putText(canvas, "No data", (540, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
            cv2.putText(canvas, "-", (800, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
        
        # 进度条
        if progress > 0:
            bar_w = int(400 * min(progress, 1.0))
            cv2.rectangle(canvas, (900, y - 15), (900 + bar_w, y + 5), color, -1)
        cv2.rectangle(canvas, (900, y - 15), (1300, y + 5), (200, 200, 200), 2)
        
        y += 120
    
    # 说明
    y_info = 840
    cv2.putText(canvas, "Legend:", (50, y_info),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.putText(canvas, "Real Pos = Gazebo ground truth", (50, y_info+25),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 100, 0), 1)
    cv2.putText(canvas, "Detected Pos = Published by drone", (50, y_info+50),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 150), 1)
    cv2.putText(canvas, f"Error Threshold = {err_threshold}m (GREEN=OK, RED=BAD)", (50, y_info+75),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
    
    return canvas

if __name__ == "__main__":
    # 官方初始化（一字不改）
    left_actors = list(range(actor_num))
    rospy.init_node('score_cal')
    time.sleep(1)
    start_time = rospy.get_time()
    del_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    score_pub = rospy.Publisher("/score", Int16, queue_size=1)
    time_usage_pub = rospy.Publisher("/time_usage", Int16, queue_size=1)
    left_actors_pub = rospy.Publisher("/left_actors", String, queue_size=1)
    
    # 官方订阅（一字不改）
    actor_blue_sub = rospy.Subscriber("/actor_blue_info", ActorInfo, actor_info_callback, queue_size=1)
    actor_green_sub = rospy.Subscriber("/actor_green_info", ActorInfo, actor_info_callback, queue_size=1)
    actor_white_sub = rospy.Subscriber("/actor_white_info", ActorInfo, actor_info_callback, queue_size=1)
    actor_brown_sub = rospy.Subscriber("/actor_brown_info", ActorInfo, actor_info_callback, queue_size=1)
    actor_red1_sub = rospy.Subscriber("/actor_red1_info", ActorInfo, actor_info1_callback, queue_size=1)
    actor_red2_sub = rospy.Subscriber("/actor_red2_info", ActorInfo, actor_info2_callback, queue_size=1)
    
    score = (2 + target_finish) * 60 - sensor_cost * 3e-3
    rate = rospy.Rate(10)
    
    print("="*60)
    print("官方记分系统已启动")
    print(f"误差阈值: {err_threshold}m (官方)")
    print(f"检测时间: {detection_time}s (官方)")
    print("="*60)
    
    # ✅ 获取实际启动的无人机数量
    num_drones = rospy.get_param('/num_drones', 2)
    rospy.loginfo(f"检测到 {num_drones} 架无人机")
    
    while not rospy.is_shutdown():
        # 官方主循环（修改为检查实际数量的无人机）
        for i in range(num_drones):  # ✅ 改为实际数量
            try:
                uav_pos_tmp = get_model_state(uav_type + '_' + str(i), 'ground_plane').pose.position
                if uav_pos_tmp.z > 6.5:
                    print("Warning: higher than 6 meter")
                    sys.exit(0)
            except:
                pass  # 如果模型不存在，跳过
        
        for i in left_actors:
            actors_pos_tmp = get_model_state('actor_' + str(i), 'ground_plane').pose.position
            if not actors_pos_tmp.x**2 + actors_pos_tmp.y**2 == 0:
                actors_pos[i] = actors_pos_tmp
        
        time_usage = rospy.get_time() - start_time
        if time_usage > 600:
            print('score:', score)
            print("Time out, mission failed")
            while True:
                pass
        
        # 官方发布
        score_pub.publish(int(score))
        time_usage_pub.publish(int(time_usage))
        left_actors_pub.publish(str(left_actors))
        
        # UI（仅此处是增强，不改官方逻辑）
        try:
            canvas = draw_ui()
            cv2.imshow("RoboCup Score System", canvas)
            cv2.waitKey(1)
        except:
            pass
        
        rate.sleep()
