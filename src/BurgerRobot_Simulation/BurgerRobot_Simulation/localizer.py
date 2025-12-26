import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

class Localizer(Node):
    def __init__(self):
        super().__init__('ekf_localizer')
        self.declare_parameter('Q_yaw', 0.01)
        self.declare_parameter('R_odom_pos', 0.05)
        self.declare_parameter('R_odom_yaw', 0.05)

        self.Q_yaw = float(self.get_parameter('Q_yaw').value)
        self.Rp = float(self.get_parameter('R_odom_pos').value)
        self.Ry = float(self.get_parameter('R_odom_yaw').value)

        self.x = np.array([0.0, 0.0, 0.0])  # x,y,theta
        self.P = np.diag([0.1, 0.1, 0.1])

        self.last_t = None
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.on_imu, 20)
        self.pose_pub = self.create_publisher(PoseStamped, '/pose', 10)

        self.yaw_rate = 0.0
        self.vx = 0.0

    def on_imu(self, msg: Imu):
        # Use yaw rate from IMU
        self.yaw_rate = msg.angular_velocity.z

    def on_odom(self, msg: Odometry):
        # Predict with odom linear velocity + imu yaw rate
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_t is None:
            self.last_t = t
            return
        dt = max(1e-3, t - self.last_t)
        self.last_t = t

        vx = msg.twist.twist.linear.x
        self.vx = vx
        theta = self.x[2]
        F = np.eye(3)
        B = np.array([[math.cos(theta)*dt, 0.0],
                      [math.sin(theta)*dt, 0.0],
                      [0.0, dt]])
        u = np.array([vx, self.yaw_rate])
        self.x = self.x + B @ u
        Q = np.diag([0.001, 0.001, self.Q_yaw])
        self.P = F @ self.P @ F.T + Q

        # Update from odom pose
        z = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, self.yaw_from_quat(msg.pose.pose.orientation)])
        H = np.eye(3)
        R = np.diag([self.Rp, self.Rp, self.Ry])
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ H) @ self.P

        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = float(self.x[0])
        ps.pose.position.y = float(self.x[1])
        ps.pose.orientation = self.quat_from_yaw(float(self.x[2]))
        self.pose_pub.publish(ps)

    def yaw_from_quat(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))

    def quat_from_yaw(self, yaw):
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.z = math.sin(yaw/2.0)
        q.w = math.cos(yaw/2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = Localizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()