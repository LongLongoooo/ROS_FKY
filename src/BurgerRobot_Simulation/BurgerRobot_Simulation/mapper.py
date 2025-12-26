import math
from typing import Optional, Tuple, List

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

try:
    import cv2
except Exception:
    cv2 = None

class Mapper(Node):
    def __init__(self):
        super().__init__('voronoi_mapper')
        self.declare_parameter('robot_radius_m', 0.09)  # Burger ~0.09 m radius
        self.declare_parameter('voronoi_thresh', 0.4)   # threshold on normalized DT ridges
        self.declare_parameter('publish_every_n_maps', 1)

        self.robot_r = float(self.get_parameter('robot_radius_m').value)
        self.voronoi_thresh = float(self.get_parameter('voronoi_thresh').value)
        self.every_n = int(self.get_parameter('publish_every_n_maps').value)

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.on_map, 5)
        self.skel_pub = self.create_publisher(Marker, '/voronoi_skeleton', 1)
        self.last_seq = 0

    def on_map(self, msg: OccupancyGrid):
        self.last_seq += 1
        if self.last_seq % self.every_n != 0:
            return
        if cv2 is None:
            self.get_logger().warn('OpenCV not available; skipping Voronoi build.')
            return

        w, h = msg.info.width, msg.info.height
        res = msg.info.resolution
        origin = msg.info.origin.position

        grid = np.array(msg.data, dtype=np.int16).reshape(h, w)
        # Free=0 or <50, Occupied=100
        occ = (grid >= 50).astype(np.uint8)

        # Inflate obstacles (configuration space) via dilation
        inflate_px = max(1, int(self.robot_r / res))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*inflate_px+1, 2*inflate_px+1))
        occ_inflated = cv2.dilate(occ, kernel)

        free = (occ_inflated == 0).astype(np.uint8) * 255
        # Distance transform on free
        dt = cv2.distanceTransform(free, cv2.DIST_L2, 3)
        dt_norm = dt / (np.max(dt) + 1e-6)

        # Ridge extraction: local maxima via simple non-max suppression
        dt_blur = cv2.GaussianBlur(dt_norm, (5, 5), 0)
        gx = cv2.Sobel(dt_blur, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(dt_blur, cv2.CV_32F, 0, 1, ksize=3)
        grad = np.sqrt(gx**2 + gy**2)
        ridges = (dt_blur > self.voronoi_thresh) & (grad < 0.02) & (free > 0)

        # Downsample ridge points for Marker
        ys, xs = np.where(ridges)
        step = max(1, int(0.05 / res))  # ~5cm sampling
        pts = [(int(x), int(y)) for x, y in zip(xs, ys)][::step]

        # Publish skeleton as LINE_LIST
        m = Marker()
        m.header.frame_id = 'map'
        m.type = Marker.POINTS
        m.scale.x = 0.02
        m.scale.y = 0.02
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.8, 1.0, 1.0
        for x, y in pts:
            p = Point()
            p.x = origin.x + (x + 0.5) * res
            p.y = origin.y + (y + 0.5) * res
            m.points.append(p)
        self.skel_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = Mapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()