import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class GoalMarker(Node):
    def __init__(self):
        super().__init__('goal_marker')
        self.declare_parameter('tolerance', 0.15)
        self.tol = float(self.get_parameter('tolerance').value)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal', self.on_goal, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/pose', self.on_pose, 10)
        self.marker_pub = self.create_publisher(Marker, '/goal_marker', 1)
        self.goal = None
        self.pose = None

    def on_goal(self, msg: PoseStamped):
        self.goal = msg
        self.publish_marker()

    def on_pose(self, msg: PoseStamped):
        self.pose = msg
        self.publish_marker()

    def publish_marker(self):
        if self.goal is None:
            return
        m = Marker()
        m.header.frame_id = 'map'
        m.type = Marker.CUBE
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.01
        m.pose.position.x = self.goal.pose.position.x
        m.pose.position.y = self.goal.pose.position.y
        # Red if far, green if reached
        reached = False
        if self.pose is not None:
            dx = self.pose.pose.position.x - self.goal.pose.position.x
            dy = self.pose.pose.position.y - self.goal.pose.position.y
            reached = (dx*dx + dy*dy) ** 0.5 < self.tol
        if reached:
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.8
        else:
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.8
        self.marker_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = GoalMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()