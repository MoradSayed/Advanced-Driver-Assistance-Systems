import matplotlib.pyplot as plt
from typing import Callable, Optional
from rclpy.timer import Timer
from rclpy.node import Node

class Plot:
    def __init__(self, target, is_time: bool, timer_creator: Node.create_timer, callback):
        self.target = target
        self.is_time = is_time
        self.create_timer = timer_creator
        self.timer_instance: Timer = None
        self.p_callback = callback
        
        self.t_values = []
        self.y_values = []
        self.z_values: Optional[list] = None  # Optional list for additional data
        self.a_values: Optional[list] = None  # Optional list for additional data
        self.start_time = 0.0

        self.add_point: Callable[[float, float, Optional[float]], None] = self._init_point

    def _init_point(self, y, t, z=None, a=None):
        if t == 0.0:
            return    # bad reference, skip.
        self.start_time = t
        self.z_values = [] if z is not None else None  # Initialize z_values if z is provided
        self.a_values = [] if a is not None else None  # Initialize z_values if a is provided
        self._append_point(y, t, z, a)
        self.add_point = self._append_point
    
    def _append_point(self, y, t, z=None, a=None):
        """Add a single (t, y, z) point to the data."""            
        self.t_values.append(t - self.start_time)
        self.y_values.append(y)
        if self.z_values is not None and z is not None:
            self.z_values.append(z)
        if self.a_values is not None and a is not None:
            self.a_values.append(a)
        
        if ((self.is_time and (t - self.start_time) > self.target) or 
            ((not self.is_time) and y > self.target)) and (self.timer_instance == None): # needs to happen only once
            print("About to plot")
            self.timer_instance = self.create_timer(3, self.p_callback)    # wait for 3 sec then trigger the plotting

    def plot(self, plt_plot_func, title="Distance vs Time", xlabel="Time (s)", ylabel="Distance (m)"):
        """Display the X vs T graph."""
        if not self.t_values or not self.y_values:
            raise ValueError("No data to plot. Add some points first.")
        
        self.add_point = lambda *args: None
        self.timer_instance.destroy()

        plt.figure(figsize=(8, 4))
        plt_plot_func()
        # plt.plot(self.t_values, self.y_values, marker='o', linestyle='-', color='green', label='Measured Distance')
        
        # if self.z_values:  # Plot z_values if available
            # plt.plot(self.t_values, self.z_values, linestyle='--', color='blue', label='Lead_V(t)')
        
        plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()
