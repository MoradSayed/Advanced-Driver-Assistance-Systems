from rclpy.node import Node
from controller import Lidar
from sensor_msgs.msg import LaserScan

class LidarDev(Lidar):
    _model = "Sick LMS 291"

    def __init__(self, node:Node, TIME_STEP):
        super().__init__("Sick LMS 291")
        self.enable(TIME_STEP)

        self.lnum_points = self.getHorizontalResolution()
        self.lfov = self.getFov()
        self.lminrange = self.getMinRange()
        self.lmaxrange = self.getMaxRange()

        self.node = node
        self.scan_pub = node.create_publisher(LaserScan, '/av_lidar', 10)

        self.scan_msg = LaserScan()
        self.scan_msg.angle_min = -self.lfov / 2  # Leftmost angle
        self.scan_msg.angle_max = self.lfov / 2   # Rightmost angle
        self.scan_msg.angle_increment = self.lfov / self.lnum_points  # Angle between beams
        self.scan_msg.range_min = self.lminrange  # LiDAR min range
        self.scan_msg.range_max = self.lmaxrange  # LiDAR max range

    def process_data(self):
        self.scan_msg.header.stamp = self.node.get_clock().now().to_msg()
        self.scan_msg.header.frame_id = self.getName()
        self.scan_msg.ranges = list(self.getRangeImage())
        
        self.scan_pub.publish(self.scan_msg)
