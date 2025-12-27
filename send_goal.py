#!/usr/bin/env python3
"""
Script để gửi goal cho robot trong Gazebo world
Sử dụng: python3 send_goal.py <x> <y>
Ví dụ: python3 send_goal.py 2.0 2.0
"""
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

def send_goal(x, y):
    rclpy.init()
    node = Node('goal_sender')
    pub = node.create_publisher(PoseStamped, '/goal', 10)
    
    # Đợi cho subscribers kết nối
    import time
    time.sleep(1)
    
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x = float(x)
    goal.pose.position.y = float(y)
    goal.pose.position.z = 0.0
    goal.pose.orientation.w = 1.0
    
    pub.publish(goal)
    node.get_logger().info(f'Goal sent: x={x}, y={y}')
    
    # Đợi một chút để message được gửi
    time.sleep(0.5)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python3 send_goal.py <x> <y>")
        print("Example: python3 send_goal.py 2.0 2.0")
        sys.exit(1)
    
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    send_goal(x, y)
