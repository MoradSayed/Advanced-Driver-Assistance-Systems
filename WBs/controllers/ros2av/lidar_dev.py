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

    def process_data(self):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.node.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.getName()
        
        scan_msg.angle_min = -self.lfov / 2  # Leftmost angle
        scan_msg.angle_max = self.lfov / 2   # Rightmost angle
        scan_msg.angle_increment = self.lfov / self.lnum_points  # Angle between beams
        scan_msg.range_min = self.lminrange  # LiDAR min range
        scan_msg.range_max = self.lmaxrange  # LiDAR max range

        scan_msg.ranges = list(self.getRangeImage())

        self.scan_pub.publish(scan_msg)


