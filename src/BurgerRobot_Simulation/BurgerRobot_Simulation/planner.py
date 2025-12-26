import math
from typing import Optional
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist

class Planner(Node):
    def __init__(self):
        super().__init__('simple_planner')
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('max_linear_speed', 0.22)
        self.declare_parameter('max_angular_speed', 2.0)

        self.goal_tol = float(self.get_parameter('goal_tolerance').value)
        self.max_lin = float(self.get_parameter('max_linear_speed').value)
        self.max_ang = float(self.get_parameter('max_angular_speed').value)

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.on_map, 5)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal', self.on_goal, 5)
        self.pose_sub = self.create_subscription(PoseStamped, '/pose', self.on_pose, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 1)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_plan', 10)

        self.map: Optional[OccupancyGrid] = None
        self.goal: Optional[PoseStamped] = None
        self.pose: Optional[PoseStamped] = None
        
        self.get_logger().info('Simple planner initialized. Waiting for goal...')

    def on_map(self, msg: OccupancyGrid):
        if self.map is None:
            self.get_logger().info('Map received from Gazebo')

        self.map = msg

    def on_goal(self, msg: PoseStamped):
        self.goal = msg
        self.get_logger().info(f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self.plan()

    def on_pose(self, msg: PoseStamped):
        self.pose = msg
        self.plan()

    def plan(self):
        """Tạo velocity command đơn giản hướng về goal"""
        if self.pose is None or self.goal is None:
            return

        # Tính khoảng cách và hướng đến goal
        dx = self.goal.pose.position.x - self.pose.pose.position.x
        dy = self.goal.pose.position.y - self.pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Nếu đã đến goal thì dừng
        if distance < self.goal_tol:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info('Goal reached!', throttle_duration_sec=2.0)
            return

        # Tính góc cần quay (goal angle)
        goal_angle = math.atan2(dy, dx)
        
        # Lấy góc hiện tại của robot từ quaternion
        q = self.pose.pose.orientation
        # Chuyển quaternion thành yaw angle (simplified for 2D)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Tính sai số góc
        angle_error = goal_angle - current_yaw
        # Normalize về [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2.0 * math.pi
        while angle_error < -math.pi:
            angle_error += 2.0 * math.pi
        
        # Tạo lệnh điều khiển
        cmd = Twist()
        
        # Nếu góc sai lệch quá lớn (> 90 độ), dừng lại và chỉ quay
        if abs(angle_error) > 1.57:  # ~90 degrees
            cmd.linear.x = 0.0
            cmd.angular.z = max(-self.max_ang, min(self.max_ang, 2.5 * angle_error))
        # Nếu góc sai lệch vừa phải, đi chậm và quay
        elif abs(angle_error) > 0.3:  # ~17 degrees
            cmd.linear.x = 0.1
            cmd.angular.z = max(-self.max_ang, min(self.max_ang, 2.0 * angle_error))
        # Nếu đã gần đúng hướng, đi nhanh
        else:
            # Giảm tốc khi gần đích
            if distance < 1.0:
                cmd.linear.x = max(0.05, min(self.max_lin, distance * 0.2))
            else:
                cmd.linear.x = self.max_lin
            cmd.angular.z = max(-self.max_ang, min(self.max_ang, 1.5 * angle_error))
        
        self.cmd_pub.publish(cmd)
        
        # Publish path visualization (đường thẳng đến goal)
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        ps1 = PoseStamped()
        ps1.header.frame_id = 'map'
        ps1.pose = self.pose.pose
        
        ps2 = PoseStamped()
        ps2.header.frame_id = 'map'
        ps2.pose = self.goal.pose
        
        path.poses = [ps1, ps2]
        self.path_pub.publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()