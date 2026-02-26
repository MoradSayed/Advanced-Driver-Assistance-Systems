from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..manager import Manager

import time, math
from typing import Callable
from example_interfaces.msg import Float64MultiArray

class WheelSpeed:
    def __init__(self, manager: "Manager", callback: Callable, wheel_radius: float):
        self.man = manager
        self.callback = callback
        manager.node.create_subscription(Float64MultiArray, "/wheel_speed", self.process_sensor_data, 10)

        self.prev_left = 0.0
        self.prev_right = 0.0

        self.wheel_radius = wheel_radius
        self.total_distance = 0.0

        self.last_time = None
        self.current_time = None
        self.speed_in_kmh = None

    def process_sensor_data(self, msg):
        curr_left, curr_right = msg.data    # Angle in rad.         # [(2*pi)/num_of_teeth] to convert from teeth counted to angle in rad.
        self.current_time = self.man.adas.current_time
        if self.last_time is None:
            self.last_time = self.current_time - 0.01

        delta_left = curr_left - self.prev_left
        delta_right = curr_right - self.prev_right

        avg_rotation = (delta_left + delta_right) / 2.0

        #? Distance calculation
        distance_step = self.wheel_radius * avg_rotation
        self.total_distance += abs(distance_step)

        #? Speed calculation
        self.speed_in_kmh = (distance_step / ((self.current_time) - self.last_time-0.000001)) * 3.6
        self.man.node.get_logger().info(f"Speed in km/h: {self.speed_in_kmh: .2f}")

        # print(f"{curr_left} & {curr_right} >> {delta_left} | {delta_right} >> {distance_step} / {self.total_distance}")
        self.prev_left = curr_left
        self.prev_right = curr_right
        self.last_time = self.current_time

        if self.callback is not None:
            self.callback(self.speed_in_kmh)

    def get_total_distance(self):
        return self.total_distance
    
    def get_speed_kmh(self):
        return self.speed_in_kmh
