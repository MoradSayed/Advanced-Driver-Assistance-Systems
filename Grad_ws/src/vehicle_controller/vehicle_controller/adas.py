from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .manager import Manager

from std_msgs.msg import Float64

from .sensors.Wheel_speed import WheelSpeed
from .sensors.lidar import LidarSensor
from .plotter import Plot
from .closed_loop.pid_controller import PID

class ADAS:
    def __init__(self, manager: "Manager"):
        self.manager = manager
        self.is_acc = False

        # self.ws_sensor = WheelSpeed(manager, self.process, 0.374)   # (tire radius = 0.374) obtained from the BmwX5Wheel.proto. https://github.com/cyberbotics/webots/blob/9b5ed70644d66a2b405a039a521899b511102611/projects/vehicles/protos/bmw/BmwX5Wheel.proto
        
        self.current_time = 0.0
        manager.node.create_subscription(
            Float64, "/wbts_time",
            lambda t: setattr(self, "current_time", t.data),
            10) #? to be replaced with real time in practical trials

        safe_distance = 15
        # self.graph = Plot(plot_point = Target_distance, timer_creator = manager.node.create_timer)
        self.accel_controller = PID(0.72, 0.0011, 0.0085, safe_distance, (0, 0.6))  #TODO: Tune the controller
        self.lidar = LidarSensor(manager, callback=self.process)

    def toggle_acc(self, state):
        self.is_acc = state
        print(f"ACC is {'on' if state else 'off'}")

    def process(self, distance):
        if self.is_acc:
            if distance == float('inf'):
                pedal = 0.6
            else:
                pedal = 0.6 - self.accel_controller.compute(distance, self.current_time)
            self.manager.speed_manager(float(pedal))
            # self.graph.add_point(distance_traveled, self.current_time)
