#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
目标分配协调器（简化版）- 避免多架无人机重复追踪同一目标
版本: v1.0.0-simple
作者: 东华大学 Astraeus队
日期: 2025-10-17

功能：
1. 收集所有无人机的锁定状态
2. 广播锁定颜色列表
3. 红色目标允许多机追踪
"""

import rospy
from std_msgs.msg import String
import threading


class TargetCoordinator:
    """目标分配协调器"""
    
    def __init__(self):
        rospy.init_node('target_coordinator')
        
        # 参数配置
        self.num_drones = rospy.get_param('~num_drones', 6)
        self.heartbeat_timeout = rospy.get_param('~heartbeat_timeout', 3.0)  # 心跳超时时间（秒）
        self.status_publish_rate = rospy.get_param('~status_publish_rate', 2.0)  # 状态发布频率（Hz）
        
        # 目标锁定状态
        # 格式: {color: {'drone_id': int, 'last_heartbeat': rospy.Time}}
        self.target_locks = {}
        self.lock_mutex = threading.Lock()
        
        # 允许多机追踪的颜色列表
        self.multi_track_colors = ['red']  # 红色允许多机追踪
        
        # 所有可能的目标颜色
        self.all_colors = ['blue', 'green', 'white', 'brown', 'red']
        
        # 订阅者
        self.request_sub = rospy.Subscriber(
            '/target_coordinator/request',
            TargetRequest,
            self._handle_request,
            queue_size=10
        )
        
        self.heartbeat_sub = rospy.Subscriber(
            '/target_coordinator/heartbeat',
            TargetHeartbeat,
            self._handle_heartbeat,
            queue_size=10
        )
        
        # 发布者
        self.response_pub = rospy.Publisher(
            '/target_coordinator/response',
            TargetResponse,
            queue_size=10
        )
        
        self.status_pub = rospy.Publisher(
            '/target_coordinator/status',
            CoordinatorStatus,
            queue_size=1
        )
        
        rospy.loginfo("=" * 80)
        rospy.loginfo("✅ 目标分配协调器 v1.0.0 初始化完成")
        rospy.loginfo(f"   📊 管理无人机数量: {self.num_drones}")
        rospy.loginfo(f"   ⏱️  心跳超时时间: {self.heartbeat_timeout}秒")
        rospy.loginfo(f"   🔴 允许多机追踪: {', '.join(self.multi_track_colors)}")
        rospy.loginfo(f"   🎯 目标颜色列表: {', '.join(self.all_colors)}")
        rospy.loginfo("=" * 80)
    
    def _handle_request(self, msg):
        """
        处理追踪请求
        
        Args:
            msg (TargetRequest): 包含 drone_id, target_color
        """
        drone_id = msg.drone_id
        target_color = msg.target_color
        
        with self.lock_mutex:
            # 检查目标颜色是否有效
            if target_color not in self.all_colors:
                rospy.logwarn(f"⚠️ 无人机{drone_id}请求追踪无效颜色: {target_color}")
                self._send_response(drone_id, target_color, approved=False, 
                                   reason=f"无效颜色: {target_color}")
                return
            
            # 红色目标：始终允许追踪
            if target_color in self.multi_track_colors:
                rospy.loginfo(f"✅ 无人机{drone_id}请求追踪{target_color}目标 → 批准（允许多机追踪）")
                self._send_response(drone_id, target_color, approved=True, 
                                   reason="红色目标允许多机追踪")
                
                # 记录锁定（即使允许多机，也记录用于状态监控）
                if target_color not in self.target_locks:
                    self.target_locks[target_color] = []
                
                # 检查是否已在追踪列表中
                already_tracking = False
                for lock in self.target_locks[target_color]:
                    if lock['drone_id'] == drone_id:
                        lock['last_heartbeat'] = rospy.Time.now()
                        already_tracking = True
                        break
                
                if not already_tracking:
                    self.target_locks[target_color].append({
                        'drone_id': drone_id,
                        'last_heartbeat': rospy.Time.now()
                    })
                return
            
            # 非红色目标：检查是否已被其他无人机锁定
            if target_color in self.target_locks:
                # 检查锁定是否超时
                lock_info = self.target_locks[target_color]
                time_since_heartbeat = (rospy.Time.now() - lock_info['last_heartbeat']).to_sec()
                
                if time_since_heartbeat > self.heartbeat_timeout:
                    # 锁定超时，释放并允许新请求
                    old_drone = lock_info['drone_id']
                    rospy.logwarn(f"⏱️ {target_color}目标锁定超时（无人机{old_drone}），自动释放")
                    del self.target_locks[target_color]
                    
                    # 批准新请求
                    rospy.loginfo(f"✅ 无人机{drone_id}请求追踪{target_color}目标 → 批准（超时释放）")
                    self._send_response(drone_id, target_color, approved=True, 
                                       reason=f"前一锁定超时，已释放")
                    self.target_locks[target_color] = {
                        'drone_id': drone_id,
                        'last_heartbeat': rospy.Time.now()
                    }
                else:
                    # 目标已被锁定，拒绝请求
                    locked_by = lock_info['drone_id']
                    
                    # 如果是同一架无人机重复请求，更新心跳
                    if locked_by == drone_id:
                        rospy.loginfo(f"🔄 无人机{drone_id}重复请求{target_color}目标 → 批准（已锁定）")
                        self._send_response(drone_id, target_color, approved=True, 
                                           reason="已锁定该目标")
                        lock_info['last_heartbeat'] = rospy.Time.now()
                    else:
                        rospy.loginfo(f"❌ 无人机{drone_id}请求追踪{target_color}目标 → 拒绝（已被无人机{locked_by}锁定）")
                        self._send_response(drone_id, target_color, approved=False, 
                                           reason=f"目标已被无人机{locked_by}锁定")
            else:
                # 目标未被锁定，批准请求
                rospy.loginfo(f"✅ 无人机{drone_id}请求追踪{target_color}目标 → 批准（新目标）")
                self._send_response(drone_id, target_color, approved=True, 
                                   reason="目标未被锁定")
                self.target_locks[target_color] = {
                    'drone_id': drone_id,
                    'last_heartbeat': rospy.Time.now()
                }
    
    def _handle_heartbeat(self, msg):
        """
        处理心跳消息，更新锁定状态
        
        Args:
            msg (TargetHeartbeat): 包含 drone_id, target_color, is_tracking
        """
        drone_id = msg.drone_id
        target_color = msg.target_color
        is_tracking = msg.is_tracking
        
        with self.lock_mutex:
            if not is_tracking:
                # 无人机停止追踪，释放锁定
                if target_color in self.target_locks:
                    # 红色目标：从列表中移除该无人机
                    if target_color in self.multi_track_colors:
                        self.target_locks[target_color] = [
                            lock for lock in self.target_locks[target_color] 
                            if lock['drone_id'] != drone_id
                        ]
                        if not self.target_locks[target_color]:
                            del self.target_locks[target_color]
                        rospy.loginfo(f"🔓 无人机{drone_id}停止追踪{target_color}目标，释放锁定")
                    else:
                        # 非红色目标：检查是否是锁定者
                        if self.target_locks[target_color]['drone_id'] == drone_id:
                            del self.target_locks[target_color]
                            rospy.loginfo(f"🔓 无人机{drone_id}停止追踪{target_color}目标，释放锁定")
                return
            
            # 更新心跳时间
            if target_color in self.target_locks:
                # 红色目标：更新对应无人机的心跳
                if target_color in self.multi_track_colors:
                    for lock in self.target_locks[target_color]:
                        if lock['drone_id'] == drone_id:
                            lock['last_heartbeat'] = rospy.Time.now()
                            rospy.loginfo_throttle(5.0, 
                                f"💓 无人机{drone_id}追踪{target_color}目标心跳更新")
                            break
                else:
                    # 非红色目标：检查是否是锁定者
                    if self.target_locks[target_color]['drone_id'] == drone_id:
                        self.target_locks[target_color]['last_heartbeat'] = rospy.Time.now()
                        rospy.loginfo_throttle(5.0, 
                            f"💓 无人机{drone_id}追踪{target_color}目标心跳更新")
    
    def _send_response(self, drone_id, target_color, approved, reason):
        """
        发送追踪请求响应
        
        Args:
            drone_id (int): 无人机ID
            target_color (str): 目标颜色
            approved (bool): 是否批准
            reason (str): 原因说明
        """
        response = TargetResponse()
        response.drone_id = drone_id
        response.target_color = target_color
        response.approved = approved
        response.reason = reason
        response.timestamp = rospy.Time.now()
        
        self.response_pub.publish(response)
    
    def _check_timeouts(self):
        """检查并清理超时的锁定"""
        with self.lock_mutex:
            colors_to_remove = []
            
            for color, lock_info in self.target_locks.items():
                # 红色目标：检查每个无人机的心跳
                if color in self.multi_track_colors:
                    locks_to_remove = []
                    for i, lock in enumerate(lock_info):
                        time_since_heartbeat = (rospy.Time.now() - lock['last_heartbeat']).to_sec()
                        if time_since_heartbeat > self.heartbeat_timeout:
                            rospy.logwarn(f"⏱️ 无人机{lock['drone_id']}追踪{color}目标超时，释放锁定")
                            locks_to_remove.append(i)
                    
                    # 从后往前删除，避免索引错乱
                    for i in reversed(locks_to_remove):
                        lock_info.pop(i)
                    
                    if not lock_info:
                        colors_to_remove.append(color)
                else:
                    # 非红色目标：检查单一锁定
                    time_since_heartbeat = (rospy.Time.now() - lock_info['last_heartbeat']).to_sec()
                    if time_since_heartbeat > self.heartbeat_timeout:
                        rospy.logwarn(f"⏱️ 无人机{lock_info['drone_id']}追踪{color}目标超时，释放锁定")
                        colors_to_remove.append(color)
            
            # 删除超时的锁定
            for color in colors_to_remove:
                del self.target_locks[color]
    
    def _publish_status(self):
        """发布协调器状态"""
        with self.lock_mutex:
            status = CoordinatorStatus()
            status.timestamp = rospy.Time.now()
            status.num_drones = self.num_drones
            
            # 统计锁定信息
            for color, lock_info in self.target_locks.items():
                if color in self.multi_track_colors:
                    # 红色目标：多个无人机
                    drone_ids = [lock['drone_id'] for lock in lock_info]
                    status.locked_colors.append(color)
                    status.locked_by_drones.append(str(drone_ids))
                else:
                    # 非红色目标：单个无人机
                    status.locked_colors.append(color)
                    status.locked_by_drones.append(str(lock_info['drone_id']))
            
            self.status_pub.publish(status)
            
            # 日志输出
            if self.target_locks:
                lock_summary = []
                for color, lock_info in self.target_locks.items():
                    if color in self.multi_track_colors:
                        drone_ids = [lock['drone_id'] for lock in lock_info]
                        lock_summary.append(f"{color}→{drone_ids}")
                    else:
                        lock_summary.append(f"{color}→UAV{lock_info['drone_id']}")
                rospy.loginfo(f"📊 当前锁定: {', '.join(lock_summary)}")
            else:
                rospy.loginfo_throttle(10.0, "📊 当前无目标锁定")
    
    def run(self):
        """主循环"""
        rospy.loginfo("🚀 目标分配协调器开始运行")
        
        # 定时器：检查超时
        timeout_timer = rospy.Timer(
            rospy.Duration(1.0),  # 每秒检查一次
            lambda event: self._check_timeouts()
        )
        
        # 定时器：发布状态
        status_timer = rospy.Timer(
            rospy.Duration(1.0 / self.status_publish_rate),
            lambda event: self._publish_status()
        )
        
        rospy.spin()
        
        timeout_timer.shutdown()
        status_timer.shutdown()
        rospy.loginfo("✅ 协调器停止")


if __name__ == '__main__':
    try:
        coordinator = TargetCoordinator()
        coordinator.run()
    except rospy.ROSInterruptException:
        pass
