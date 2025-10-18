#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Actor真值位置发布器
从Gazebo获取actor的真实位置并发布到ROS话题
用于验证、对比和可视化
"""

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from gazebo_msgs.srv import GetModelState, GetLinkState
from gazebo_msgs.msg import ModelStates, LinkStates
import sys
import tf2_ros

class ActorGroundTruthPublisher:
    def __init__(self, actor_name='actor', publish_rate=30, use_service=True):
        self.actor_name = actor_name
        self.publish_rate = publish_rate
        self.use_service = use_service  # True=用服务, False=用话题订阅
        
        # 当前位置
        self.current_position = None
        self.current_orientation = None
        
        print("=" * 80)
        print("  Actor真值位置发布器")
        print("=" * 80)
        print("  Actor名称: %s" % actor_name)
        print("  发布频率: %d Hz" % publish_rate)
        print("  获取方式: %s" % ("服务调用" if use_service else "话题订阅"))
        print("=" * 80)
        print()
        
        # 发布器
        self.point_pub = rospy.Publisher(
            '/actor/ground_truth/position',
            PointStamped,
            queue_size=10
        )
        
        self.pose_pub = rospy.Publisher(
            '/actor/ground_truth/pose',
            PoseStamped,
            queue_size=10
        )
        
        # TF广播器（可选，用于RViz可视化）
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        if use_service:
            # 方法1: 使用服务获取（推荐，更精确）
            self.setup_service()
        else:
            # 方法2: 订阅话题获取（更高效）
            self.setup_subscriber()
        
        print("[就绪] 开始发布actor位置...")
        print()
    
    def setup_service(self):
        """使用Gazebo服务获取位置"""
        try:
            # 等待服务可用
            rospy.wait_for_service('/gazebo/get_model_state', timeout=5.0)
            self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            
            # 尝试使用link state（更精确）
            try:
                rospy.wait_for_service('/gazebo/get_link_state', timeout=2.0)
                self.get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
                self.use_link_state = True
                print("[提示] 使用GetLinkState获取位置（更精确）")
            except:
                self.use_link_state = False
                print("[提示] 使用GetModelState获取位置")
            
        except rospy.ROSException:
            rospy.logerr("[错误] Gazebo服务不可用，请确保仿真正在运行")
            sys.exit(1)
    
    def setup_subscriber(self):
        """订阅Gazebo话题获取位置"""
        print("[提示] 订阅ModelStates话题")
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)
    
    def model_states_callback(self, msg):
        """ModelStates话题回调"""
        try:
            # 在列表中找到actor
            index = msg.name.index(self.actor_name)
            self.current_position = msg.pose[index].position
            self.current_orientation = msg.pose[index].orientation
        except ValueError:
            rospy.logwarn_throttle(5.0, "[警告] 未找到actor '%s'" % self.actor_name)
    
    def get_actor_pose_from_service(self):
        """通过服务获取actor位姿"""
        try:
            if self.use_link_state:
                # 使用link state
                link_name = self.actor_name + '::actor_pose'
                response = self.get_link_state(link_name, '')
                return response.link_state.pose
            else:
                # 使用model state
                response = self.get_model_state(self.actor_name, '')
                return response.pose
                
        except Exception as e:
            rospy.logwarn_throttle(5.0, "[警告] 获取位置失败: %s" % str(e))
            return None
    
    def publish_actor_pose(self):
        """发布actor位置"""
        # 获取位姿
        if self.use_service:
            pose = self.get_actor_pose_from_service()
            if pose is None:
                return
            position = pose.position
            orientation = pose.orientation
        else:
            if self.current_position is None:
                return
            position = self.current_position
            orientation = self.current_orientation
        
        # 创建时间戳
        stamp = rospy.Time.now()
        
        # 发布PointStamped
        point_msg = PointStamped()
        point_msg.header.stamp = stamp
        point_msg.header.frame_id = "map"
        point_msg.point = position
        self.point_pub.publish(point_msg)
        
        # 发布PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position = position
        pose_msg.pose.orientation = orientation
        self.pose_pub.publish(pose_msg)
        
        # 发布TF（用于RViz可视化）
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = "actor_ground_truth"
        tf_msg.transform.translation.x = position.x
        tf_msg.transform.translation.y = position.y
        tf_msg.transform.translation.z = position.z
        tf_msg.transform.rotation = orientation
        self.tf_broadcaster.sendTransform(tf_msg)
        
        # 日志输出（降低频率）
        rospy.loginfo_throttle(1.0, 
            "[Actor真值] ENU: (%.2f, %.2f, %.2f)" % 
            (position.x, position.y, position.z))
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(self.publish_rate)
        
        print("发布话题:")
        print("  - /actor/ground_truth/position (PointStamped)")
        print("  - /actor/ground_truth/pose (PoseStamped)")
        print("  - TF: map -> actor_ground_truth")
        print()
        print("按 Ctrl+C 停止...")
        print()
        
        while not rospy.is_shutdown():
            self.publish_actor_pose()
            rate.sleep()


def main():
    # 解析参数
    import argparse
    parser = argparse.ArgumentParser(description='发布Actor真值位置')
    parser.add_argument('--name', type=str, default='actor',
                       help='Actor名称（默认: actor）')
    parser.add_argument('--rate', type=int, default=30,
                       help='发布频率 Hz（默认: 30）')
    parser.add_argument('--method', type=str, choices=['service', 'topic'], default='service',
                       help='获取方式: service或topic（默认: service）')
    
    # 解析（忽略rosrun传来的参数）
    args, unknown = parser.parse_known_args()
    
    # 初始化节点
    rospy.init_node('actor_ground_truth_publisher')
    
    # 创建发布器
    use_service = (args.method == 'service')
    publisher = ActorGroundTruthPublisher(
        actor_name=args.name,
        publish_rate=args.rate,
        use_service=use_service
    )
    
    try:
        publisher.run()
    except rospy.ROSInterruptException:
        print()
        print("=" * 80)
        print("  已停止发布")
        print("=" * 80)


if __name__ == "__main__":
    main()

