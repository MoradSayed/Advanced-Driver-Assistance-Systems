from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .manager import Manager

from typing import Callable
from example_interfaces.msg import Float64MultiArray

class WheelSpeed:
    def __init__(self, manager: "Manager", callback: Callable, wheel_radius: float):
        self.callback = callback
        manager.node.create_subscription(Float64MultiArray, "/wheel_speed", self.process_sensor_data, 10)

        self.prev_left = 0.0
        self.prev_right = 0.0

        self.wheel_radius = wheel_radius
        self.total_distance = 0.0

    def process_sensor_data(self, msg):
        curr_left, curr_right = msg.data

        delta_left = curr_left - self.prev_left
        delta_right = curr_right - self.prev_right

        avg_rotation = (delta_left + delta_right) / 2.0
        distance_step = self.wheel_radius * avg_rotation
        self.total_distance += abs(distance_step)

        # print(f"{curr_left} & {curr_right} >> {delta_left} | {delta_right} >> {distance_step} / {self.total_distance}")
        self.prev_left = curr_left
        self.prev_right = curr_right

        self.callback(self.total_distance)

    def get_total_distance(self):
        return self.total_distance
