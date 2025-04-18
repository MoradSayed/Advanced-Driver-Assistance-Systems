from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .manager import Manager
from std_msgs.msg import Float64

from .plotter import Plot

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
                                         10)

        self.distance_controller = PID(2, 0.0011, 0.0085, 200, lambda: manager.vdriver.act_spd, self)
        self.distance_controller.setLims(0, 60)

    def process(self, distance_traveled):
        pedal = self.distance_controller.compute(distance_traveled, self.current_time)
        self.speed_pub(float(pedal))

class PID:
    def __init__(self, KP, KI, KD, target=0, spd=None, adas=None):
        self.speed = spd    #!####
        self.adas = adas
        
        self.kp = KP
        self.ki = KI
        self.kd = KD

        self.sp = target
        
        self.error_last = 0
        self.integral_error = 0
        
        self.saturation_max = None
        self.saturation_min = None

        self.last_time = None

    def compute(self, pos, time_now):
        if self.last_time:
            dt = time_now - self.last_time
            if dt == 0:
                dt = 0.0001
        else:
            dt = 0.1

        error = self.sp - pos   # compute the error
        derivative_error = (error - self.error_last) / dt   # find the derivative of the error (how the error changes with time)
        self.integral_error += error * dt   # error build up over time

        output = self.kp*error + self.ki*self.integral_error + self.kd*derivative_error
        self.error_last = error

        if output > self.saturation_max and self.saturation_max is not None:
            output = self.saturation_max
        elif output < self.saturation_min and self.saturation_min is not None:
            output = self.saturation_min

        self.last_time = time_now
        self.plot_g.add_point(pos, time_now)         #!####
        return output

    def setLims(self, min, max):
        self.saturation_max = max
        self.saturation_min = min

        self.plot_g = Plot(plot_point = 30, brake=self.adas.brake_pub)             #!####
