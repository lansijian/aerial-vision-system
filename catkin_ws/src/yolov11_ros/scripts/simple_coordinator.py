#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
简化版目标分配协调器 - 使用String消息
版本: v1.0.0-simple
作者: 东华大学 Astraeus队
日期: 2025-10-17
"""

import rospy
from std_msgs.msg import String

class SimpleCoordinator:
    """简化版协调器：收集并广播锁定状态"""
    
    def __init__(self):
        rospy.init_node('simple_coordinator')
        
        self.num_drones = rospy.get_param('~num_drones', 6)
        
        # 存储每架无人机锁定的颜色
        # 格式: {drone_id: color}
        self.drone_locks = {}
        
        # 订阅每架无人机的锁定状态
        self.lock_subs = []
        for i in range(self.num_drones):
            sub = rospy.Subscriber(
                f'/drone_{i}/locked_color',
                String,
                self._update_lock,
                callback_args=i,
                queue_size=10
            )
            self.lock_subs.append(sub)
        
        # 发布全局锁定状态
        self.status_pub = rospy.Publisher(
            '/coordinator/locked_colors',
            String,
            queue_size=10
        )
        
        # 定时发布状态
        self.timer = rospy.Timer(rospy.Duration(0.5), self._publish_status)
        
        rospy.loginfo("=" * 80)
        rospy.loginfo("✅ 简化版协调器初始化完成")
        rospy.loginfo(f"   📊 管理无人机数量: {self.num_drones}")
        rospy.loginfo("=" * 80)
    
    def _update_lock(self, msg, drone_id):
        """更新无人机的锁定状态"""
        if msg.data:
            self.drone_locks[drone_id] = msg.data
            rospy.loginfo(f"🔒 无人机{drone_id} 锁定: {msg.data}")
        else:
            if drone_id in self.drone_locks:
                color = self.drone_locks[drone_id]
                del self.drone_locks[drone_id]
                rospy.loginfo(f"🔓 无人机{drone_id} 释放: {color}")
    
    def _publish_status(self, event):
        """发布全局锁定状态"""
        # 格式: "drone_0:blue,drone_1:green"
        locks = [f"drone_{did}:{color}" for did, color in self.drone_locks.items()]
        status_str = ','.join(locks)
        self.status_pub.publish(String(data=status_str))
    
    def run(self):
        """运行协调器"""
        rospy.loginfo("🚀 简化版协调器开始运行")
        rospy.spin()

if __name__ == '__main__':
    try:
        coordinator = SimpleCoordinator()
        coordinator.run()
    except rospy.ROSInterruptException:
        pass
