import matplotlib.pyplot as plt
from typing import Callable
from rclpy.timer import Timer
from rclpy.node import Node

class Plot:
    def __init__(self, plot_point, timer_creator: Node.create_timer):
        self.plot_point = plot_point
        self.create_timer = timer_creator
        self.timer_instance: Timer = None
        
        self.t_values = []
        self.y_values = []
        self.start_time = 0.0

        self.add_point: Callable[[float, float], None] = self._init_point

    def _init_point(self, y, t):
        self.start_time = t
        self._append_point(y, t)
        self.add_point = self._append_point
    
    def _append_point(self, y, t):
        """Add a single (t, y) point to the data."""            
        self.t_values.append(t - self.start_time)
        self.y_values.append(y)
        if y > self.plot_point and self.timer_instance == None: # needs to happen only once
            self.timer_instance = self.create_timer(3, self.plot)    # wait for 3 sec then trigger the plotting

    def plot(self, title="Distance vs Time", xlabel="Time (t)", ylabel="Distance (m)"):
        """Display the Y vs T graph."""
        if not self.t_values or not self.y_values:
            raise ValueError("No data to plot. Add some points first.")
        
        self.add_point = lambda _,__: None
        self.timer_instance.destroy()

        plt.figure(figsize=(8, 4))
        plt.plot(self.t_values, self.y_values, marker='o', linestyle='-', color='green', label='Y(t)')
        plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()
