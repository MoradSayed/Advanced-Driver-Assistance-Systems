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
        self.speed_pub = manager.speed_manager
        self.brake_pub = manager.brake_manager
        self.steer_pub = manager.steer_manager

        self.actual_speed = lambda: manager.vdriver.act_spd
        
        self.ws_sensor = WheelSpeed(manager, self.process, 0.374)   # (tire radius = 0.374) obtained from the BmwX5Wheel.proto. https://github.com/cyberbotics/webots/blob/9b5ed70644d66a2b405a039a521899b511102611/projects/vehicles/protos/bmw/BmwX5Wheel.proto
        
        self.current_time = 0.0
        manager.node.create_subscription(Float64, "/wbts_time", 
                                         lambda t: setattr(self, "current_time", t.data), 
                                         10)    #? to be replaced with real time in practical trials

        Target_distance = 200
        # self.graph = Plot(plot_point = Target_distance, timer_creator = manager.node.create_timer)
        # self.accel_controller = PID(2, 0.0011, 0.0085, Target_distance, (0, 60))
        
        self.lidar = LidarSensor(manager)

    def process(self, distance_traveled):
        # pedal = self.accel_controller.compute(distance_traveled, self.current_time)
        # self.speed_pub(float(pedal))
        # self.graph.add_point(distance_traveled, self.current_time)
        pass
