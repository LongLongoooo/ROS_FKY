import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class SensorBridge(Node):
    def __init__(self):
        super().__init__('sensor_bridge')
        # Parameters
        self.declare_parameter('max_range', 3.5)
        self.declare_parameter('input_scan_topic', '/scan')
        self.declare_parameter('output_scan_topic', '/scan_filtered')

        self.max_range = float(self.get_parameter('max_range').value)
        self.input_topic = str(self.get_parameter('input_scan_topic').value)
        self.output_topic = str(self.get_parameter('output_scan_topic').value)

        # Use SensorData QoS for LiDAR topics
        self.pub = self.create_publisher(LaserScan, self.output_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(LaserScan, self.input_topic, self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(
            f'SensorBridge initialized (input: {self.input_topic}, output: {self.output_topic}, max_range: {self.max_range})'
        )

    def on_scan(self, scan: LaserScan):
        # Range clipping + NaN/Inf handling suitable for LiDAR
        out = LaserScan()
        out.header = scan.header
        out.angle_min = scan.angle_min
        out.angle_max = scan.angle_max
        out.angle_increment = scan.angle_increment
        out.time_increment = scan.time_increment
        out.scan_time = scan.scan_time
        out.range_min = scan.range_min
        # Reflect clipping in reported max range
        out.range_max = min(scan.range_max, self.max_range)

        rmin = out.range_min
        rmax = out.range_max

        clipped = []
        for r in scan.ranges:
            if math.isnan(r) or math.isinf(r):
                # Treat invalid readings as "no hit" at max range
                clipped.append(rmax)
            else:
                # Clamp to [range_min, max_range]
                if r < rmin:
                    clipped.append(rmin)
                elif r > rmax:
                    clipped.append(rmax)
                else:
                    clipped.append(r)

        out.ranges = clipped
        out.intensities = scan.intensities
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
