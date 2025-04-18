import matplotlib.pyplot as plt

class Plot:
    def __init__(self, plot_point, brake):
        self.brake = brake
        
        self.t_values = []
        self.y_values = []
        self.start_time = 0.0
        self.plot_point = plot_point
        self.plotted = 0

    def add_point(self, y, t):
        """Add a single (t, y) point to the data."""
        if t < self.plot_point:
            if self.start_time is None:
                self.start_time = t
            self.t_values.append(t - self.start_time)
            self.y_values.append(y)
        elif not self.plotted:
            self.brake(1.0)
            self.plotted = 1
            self.plot()
            
    def plot(self, title="Y vs Time", xlabel="Time (t)", ylabel="Y(t)"):
        """Display the Y vs T graph."""
        if not self.t_values or not self.y_values:
            raise ValueError("No data to plot. Add some points first.")
        
        plt.figure(figsize=(8, 4))
        plt.plot(self.t_values, self.y_values, marker='o', linestyle='-', color='green', label='Y(t)')
        plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()