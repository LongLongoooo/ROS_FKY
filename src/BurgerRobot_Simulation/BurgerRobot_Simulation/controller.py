import math
from typing import Optional, List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TwistStamped

class Controller(Node):
    def __init__(self):
        super().__init__('vfh_controller')
        self.declare_parameter('sector_deg', 5.0)
        self.declare_parameter('threshold', 0.5)
        self.declare_parameter('max_lin', 0.20)
        self.declare_parameter('max_ang', 2.0)

        self.sector_deg = float(self.get_parameter('sector_deg').value)
        self.threshold = float(self.get_parameter('threshold').value)
        self.max_lin = float(self.get_parameter('max_lin').value)
        self.max_ang = float(self.get_parameter('max_ang').value)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.plan_sub = self.create_subscription(Twist, '/cmd_vel_plan', self.on_plan, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.last_plan: Optional[Twist] = None
        self.last_scan: Optional[LaserScan] = None

    def on_plan(self, msg: Twist):
        self.last_plan = msg

    def on_scan(self, scan: LaserScan):
        self.last_scan = scan
        self.step()

    def step(self):
        # Chỉ gửi lệnh khi có plan từ planner
        if self.last_scan is None or self.last_plan is None:
            return
        
        # Nếu plan yêu cầu dừng (linear=0 và angular=0), dừng luôn
        if abs(self.last_plan.linear.x) < 0.001 and abs(self.last_plan.angular.z) < 0.001:
            cmd_stamped = TwistStamped()
            cmd_stamped.header.stamp = self.get_clock().now().to_msg()
            cmd_stamped.header.frame_id = 'base_link'
            self.cmd_pub.publish(cmd_stamped)
            return
            
        # Build histogram of occupancy
        scan = self.last_scan
        n_sectors = max(1, int((scan.angle_max - scan.angle_min) * 180.0 / math.pi / self.sector_deg))
        hist = [0.0] * n_sectors
        for i, r in enumerate(scan.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            ang = scan.angle_min + i * scan.angle_increment
            idx = int(((ang - scan.angle_min) / (scan.angle_max - scan.angle_min)) * n_sectors)
            idx = max(0, min(n_sectors-1, idx))
            hist[idx] += max(0.0, 1.0 - r / max(scan.range_max, 1e-3))

        # Find valleys (sectors below threshold)
        valleys = [i for i, v in enumerate(hist) if v < self.threshold]
        cmd = Twist()
        if not valleys:
            # Nếu không có đường đi an toàn, dừng lại
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            # Choose valley nearest to straight ahead (0 rad)
            center_idx = int((0.0 - scan.angle_min) / (scan.angle_max - scan.angle_min) * n_sectors)
            target_idx = min(valleys, key=lambda i: abs(i - center_idx))
            target_angle = scan.angle_min + (target_idx + 0.5) * (scan.angle_max - scan.angle_min) / n_sectors
            # Sử dụng tốc độ từ planner
            cmd.linear.x = self.last_plan.linear.x
            # Kết hợp hướng từ planner và VFH để tránh vật cản
            cmd.angular.z = max(-self.max_ang, min(self.max_ang, self.last_plan.angular.z + target_angle * 0.5))
        
        # Publish TwistStamped cho Gazebo
        cmd_stamped = TwistStamped()
        cmd_stamped.header.stamp = self.get_clock().now().to_msg()
        cmd_stamped.header.frame_id = 'base_link'
        cmd_stamped.twist = cmd
        self.cmd_pub.publish(cmd_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()