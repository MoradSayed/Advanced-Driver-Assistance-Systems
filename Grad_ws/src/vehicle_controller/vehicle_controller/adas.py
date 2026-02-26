from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .manager import Manager

from std_msgs.msg import Float64, Float32, Bool
import matplotlib.pyplot as plt, time

from .sensors.Wheel_speed import WheelSpeed
from .sensors.lidar import LidarSensor
from .plotter import Plot
from .closed_loop.pid_controller import PID
from .closed_loop.fuzzy_pid import Fuzzy

class ADAS:
    def __init__(self, manager: "Manager", set_speed, set_level, is_sim: bool):
        self.manager = manager
        self.acc_active = True
        self.max_pedal_range = 0.5  #? max is 1.0, but set to 0.5 for safety purposes

        self.timeout_counter = 0
        self.data = [] #? collects data to be saved as json on keyboard inturrept
        self.pedal = 0.0

        self.set_speed = set_speed
        self.set_level = set_level
        self.is_sim = is_sim

        if is_sim:
            self.sim_time = 0.0
            
            #? wheel speed for simulation 
            self.speed_sensor = WheelSpeed(manager, None, 0.374)   # (tire radius = 0.374) obtained from the BmwX5Wheel.proto. https://github.com/cyberbotics/webots/blob/9b5ed70644d66a2b405a039a521899b511102611/projects/vehicles/protos/bmw/BmwX5Wheel.proto
            
            #? publishers for pausing, realtime simulation and fast simulation
            self.pause_pub=manager.node.create_publisher(Bool, "/pause", 10)
            self.rt_pub=manager.node.create_publisher(Bool, "/realtime", 10)
            self.fast_pub=manager.node.create_publisher(Bool, "/fast", 10)

            manager.node.create_subscription(
                Float64, "/wbts_time",
                lambda t: setattr(self, "sim_time", t.data),
                10) #? to be replaced with real time in practical trials
            self.lead_velocity = 0
            manager.node.create_subscription(
                Float32, "/lead_vel", 
                lambda v: setattr(self, "lead_velocity", v.data),
                10)
        
            LidarSensor(manager, callback=self.process)
        else:
            self.t0 = time.time()
            
            self._wss_meas_speed = 0.0
            self.meas_vel_sub = manager.node.create_subscription(Float32, "/ego_velocity", lambda speed: setattr(self, "_wss_meas_speed", speed.data), 10)

            self.radar_sub = manager.node.create_subscription(Float32, "/radar_min", lambda msg: self.process(msg.data), 10)
            
            self.vehicle_rel_vel = 0
            manager.node.create_subscription(Float32, "/radar_rel_vel", lambda v: setattr(self, "vehicle_rel_vel", v.data), 10)
        
        if is_sim:
            self.graph = Plot(target= 120, is_time=True, timer_creator = manager.node.create_timer, callback=self.plot_graphs)

        #? Cruise Control
        self.cruise_controller= PID(0.315, 0.0, 0.2, 
                                    -self.set_speed, 
                                    (-1.0, self.max_pedal_range), 
                                    )

        #? Adaptive Cruise Control - Pure PID
        # self.fuzzy_pid = Fuzzy()
        self.accel_controller = PID(0.25, 0.00194, 0.36985, 
                                    self.safe_distance, 
                                    (-1.0, self.max_pedal_range),
                                    )# self.fuzzy_pid)   #? 0.25, 0.00194, 0.36985

    def toggle_acc(self, state):
        self.acc_active = state
        print(f"ACC is {'on' if state else 'off'}")
        if state:
            self.meas_vel_sub = self.manager.node.create_subscription(
                Float32, "/ego_velocity", 
                lambda speed: setattr(self, "_wss_meas_speed", speed.data), 
                10)
            self.radar_sub = self.manager.node.create_subscription(
                Float32, "/radar_min", 
                lambda msg: self.process(msg.data),
                10)
        else:
            self.manager.node.destroy_subscription(self.radar_sub)
            self.manager.node.destroy_subscription(self.meas_vel_sub)
            self.cruise_controller.reset()
            self.accel_controller.reset()

    def process(self, distance):    #TODO: Add the pid reset on switching between CC and ACC
        distance *= 1
        self.manager.node.get_logger().info(f"distance: {distance}")
        if self.acc_active:
            
            if distance == float('inf'):
                if self.timeout_counter < 2:
                    self.timeout_counter += 1
                else:
                    #// self.pedal = 0.15
                    #? Use cruise control
                    self.pedal = self.cruise_controller.compute(
                        -self.ego_speed,
                        self.current_time,
                        None, None          #? Required if u are gonna use fuzzy
                    )
                    # self.manager.node.get_logger().info(f"CC: Pedal -> {self.pedal}, req_speed: {self.set_speed}")
            
            else:   
                if self.timeout_counter != 0:
                    self.timeout_counter = 0
                #? Adaptive cruise control - Pure PID
                self.pedal = self.accel_controller.compute(
                    round(distance, 2), 
                    self.current_time, 
                    max(80 - self.safe_distance, self.safe_distance),
                    )

            self.manager.node.get_logger().info(f"pedal: {self.pedal}")
            if self.pedal >= 0:
                self.manager.speed_manager(float(self.pedal))
                self.manager.brake_manager(float(0.0))
            else:
                self.manager.brake_manager(float(self.pedal))
                self.manager.speed_manager(float(0.0))
            # self.graph.add_point(distance, self.current_time, self.ego_speed, self.lead_velocity)
            self.data.append({
               "timestamp": self.current_time,
               "distance": distance,
               "rel_vel": self.vehicle_rel_vel
            })


    @property
    def ego_speed(self):
        if self.is_sim:
            return self.speed_sensor.speed_in_kmh
        else:
            return self._wss_meas_speed

    @property
    def safe_distance(self):    #? in meters
        return self.set_level * self.set_speed / 3.6   # Simplified, for more accurate version `self.set_speed` should be set to `self.ego_speed`. But such a change wouldn't be as simple when using with PID (due to varying value).

    @property
    def current_time(self):
        if self.is_sim:
            return self.sim_time
        else:
            return time.time() - self.t0

    def plot_graphs(self):
        """ t, y, z, a >> """
        self.pause_pub.publish(Bool(data=True))
        self.graph.plot(self.distance_time)
        self.graph.plot(self.vel_time, title = "Velocity x Velocity", ylabel="Velocity (km/h)")

    def distance_time(self):
        plt.plot(self.graph.t_values, self.graph.y_values, marker='o', linestyle='-', color='green', label='Measured Distance')

    def vel_time(self):
        plt.plot(self.graph.t_values, self.graph.z_values, marker='o', linestyle='-', color='green', label='Ego_V(t)')
        plt.plot(self.graph.t_values, self.graph.a_values, linestyle='--', color='blue', label='Lead_V(t)')