from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .manager import Manager

from std_msgs.msg import Float64, Float32
import matplotlib.pyplot as plt

from .sensors.Wheel_speed import WheelSpeed
from .sensors.lidar import LidarSensor
from .plotter import Plot
from .closed_loop.pid_controller import PID
from .closed_loop.fuzzy_pid import Fuzzy

class ADAS:
    def __init__(self, manager: "Manager"):
        self.manager = manager
        self.is_acc = True
        self.acc_speed = 1  # 0.6

        # self.ws_sensor = WheelSpeed(manager, self.process, 0.374)   # (tire radius = 0.374) obtained from the BmwX5Wheel.proto. https://github.com/cyberbotics/webots/blob/9b5ed70644d66a2b405a039a521899b511102611/projects/vehicles/protos/bmw/BmwX5Wheel.proto

        self.current_time = 0.0
        manager.node.create_subscription(
            Float64, "/wbts_time",
            lambda t: setattr(self, "current_time", t.data),
            10) #? to be replaced with real time in practical trials
        self.lead_velocity = 0
        manager.node.create_subscription(
            Float32, "/lead_vel", 
            lambda v: setattr(self, "lead_velocity", v.data),
            10)
        self.ego_velocity = 0
        manager.node.create_subscription(
            Float64, "/get_vel", 
            lambda v: setattr(self, "ego_velocity", v.data), 
            10)
        
        self.safe_distance = 15
        self.graph = Plot(target= 120, is_time=True, timer_creator = manager.node.create_timer, callback=self.plot_graphs)

        #? Pure PID
        # self.fuzzy_pid = Fuzzy()
        self.accel_controller = PID(0.25, 0.00194, 0.36985, 
                                    self.safe_distance, 
                                    (0, self.acc_speed), 
                                    )# self.fuzzy_pid)   #? 0.25, 0.00194, 0.36985

        self.lidar = LidarSensor(manager, callback=self.process)

    def toggle_acc(self, state):
        self.is_acc = state
        print(f"ACC is {'on' if state else 'off'}")

    def process(self, distance):    #TODO: Add the pid reset 
        if self.is_acc:
            if distance == float('inf'):
                pedal = self.acc_speed
            else:
                #? Pure PID
                pedal = self.accel_controller.compute(
                    round(distance, 2), 
                    self.current_time, 
                    max(80 - self.safe_distance, self.safe_distance),
                    )

            self.manager.speed_manager(float(pedal))

            self.graph.add_point(distance, self.current_time, self.ego_velocity, self.lead_velocity)

    def plot_graphs(self):
        """ t, y, z, a >> """
        self.graph.plot(self.distance_time)
        self.graph.plot(self.vel_time, title = "Velocity x Velocity", ylabel="Velocity (km/h)")

    def distance_time(self):
        plt.plot(self.graph.t_values, self.graph.y_values, marker='o', linestyle='-', color='green', label='Measured Distance')

    def vel_time(self):
        plt.plot(self.graph.t_values, self.graph.z_values, marker='o', linestyle='-', color='green', label='Ego_V(t)')
        plt.plot(self.graph.t_values, self.graph.a_values, linestyle='--', color='blue', label='Lead_V(t)')