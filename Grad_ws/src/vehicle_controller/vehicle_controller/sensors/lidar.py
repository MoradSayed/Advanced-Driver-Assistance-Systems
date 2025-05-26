from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..manager import Manager
import numpy as np

from sensor_msgs.msg import LaserScan

class LidarSensor():
    
    def __init__(self, manager: "Manager", fov:int = 10):
        self.fov = fov
        self.manager = manager

        self.start_index = None
        self.end_index = None

        self.x = self.init_cahce
        manager.node.create_subscription(LaserScan, "/av_lidar", lambda msg: self.x(msg), 10)
        
    def init_cahce(self, scan_data: LaserScan):
        ranges = np.array(scan_data.ranges, dtype=np.float32)

        angle_increment = scan_data.angle_increment
        middle_index = len(ranges) // 2
        num_angles = int(self.fov * (np.pi/180) / angle_increment)  # 10 degrees 

        self.start_index = max(0, middle_index - num_angles)
        self.end_index = min(len(ranges) - 1, middle_index + num_angles)

        self.x = self.process_lidar
        self.process_lidar(scan_data=scan_data)

    def process_lidar(self, scan_data: LaserScan):
        ranges = np.array(scan_data.ranges, dtype=np.float32)
        front_distances = ranges[self.start_index:self.end_index+1]
        valid_distances = front_distances[(front_distances > 0) & (front_distances < np.inf)]
        
        if valid_distances.size > 0:
            min_distance = np.min(valid_distances)
            self.manager.node.get_logger().info(f"min_dist: {min_distance}")
