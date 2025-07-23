from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..manager import Manager
from typing import Callable

import numpy as np
from sensor_msgs.msg import LaserScan

class LidarSensor():
    
    def __init__(self, manager: "Manager", callback: Callable, fov:int = None):
        self.fov = fov
        self.manager = manager
        self.callback = callback

        self.start_index = None
        self.end_index = None

        #? Initial method to cache contant values
        self._process_callback = self.cache_constants #? is replaced with `process_lidar` after the first call
        manager.node.create_subscription(LaserScan, "/av_lidar", lambda msg: self._process_callback(msg), 10)

    def cache_constants(self, scan_data: LaserScan):
        ranges = np.array(scan_data.ranges, dtype=np.float32)
        if self.fov:
            #? Method 1: Angle constraints
            angle_increment = scan_data.angle_increment
            middle_index = len(ranges) // 2
            num_angles = int(self.fov * (np.pi/180) / angle_increment)

            self.start_index = max(0, middle_index - num_angles)
            self.end_index = min(len(ranges) - 1, middle_index + num_angles)

            self._process_callback = self.process_lidar_angle
            # self.process_lidar_angle(scan_data=scan_data)
        else:
            #? Method 2: Area constraints (Rectangle Shape)
            self.rad_angles = scan_data.angle_min + np.arange(len(ranges)) * scan_data.angle_increment
            self.half_width = 2.8 / 2
            self.max_range = scan_data.range_max

            self._process_callback = self.process_lidar_rect
            # self.process_lidar_rect(scan_data=scan_data)

    def process_lidar_angle(self, scan_data: LaserScan):
        ranges = np.array(scan_data.ranges, dtype=np.float32)
        front_distances = ranges[self.start_index:self.end_index+1]
        valid_distances = front_distances[(front_distances > 0) & (front_distances < np.inf)]
        
        if valid_distances.size > 0:
            min_distance = np.min(valid_distances)
        else:
            min_distance = float('inf')
        print("\r\033[K"+f"min_dist: {min_distance:.2f}", end="")
        self.callback(min_distance)

    def process_lidar_rect(self, scan_data: LaserScan):   #! Make sure angles are in the correct order.
        ranges = np.array(scan_data.ranges)
        # print(self.rad_angles)

        # Convert to Cartesian coordinates
        x = ranges * np.cos(self.rad_angles)
        y = ranges * np.sin(self.rad_angles)

        valid = (np.abs(y) <= self.half_width) & (x >= 0) & (x <= self.max_range)
        masked_ranges = np.where(valid, ranges, float('inf'))   # Apply mask: keep valid values, set others to inf

        min_distance = np.min(masked_ranges)
        # print("\r\033[K"+f"min_dist: {min_distance:.2f}", end="") #TODO: find another way to display this value
        self.callback(min_distance)
