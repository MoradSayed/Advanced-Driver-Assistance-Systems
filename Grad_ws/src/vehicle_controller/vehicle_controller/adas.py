from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .manager import Manager
from std_msgs.msg import Float64

from .plotter import Plot
from .pid_controller import PID

class ADAS:
    def __init__(self, manager: "Manager"):
        self.speed_pub = manager.speed_manager
        self.brake_pub = manager.brake_manager
        self.steer_pub = manager.steer_manager

        self.actual_speed = lambda: manager.vdriver.act_spd
        
        manager.node.create_subscription(Float64, "/odom", lambda dist: self.process(dist.data), 10)
        
        self.current_time = 0.0
        manager.node.create_subscription(Float64, "/wbts_time", 
                                         lambda t: setattr(self, "current_time", t.data), 
                                         10)    #? to be replaced with real time in practical trials

        Target_distance = 200
        self.graph = Plot(plot_point = Target_distance, timer_creator = manager.node.create_timer)
        self.accel_controller = PID(2, 0.0011, 0.0085, Target_distance, (0, 60))

    def process(self, distance_traveled):
        pedal = self.accel_controller.compute(distance_traveled, self.current_time)
        self.speed_pub(float(pedal))
        self.graph.add_point(distance_traveled, self.current_time)
