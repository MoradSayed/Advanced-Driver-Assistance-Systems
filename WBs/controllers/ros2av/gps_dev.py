import math, numpy as np
from rclpy.node import Node
from controller import GPS, Display
from std_msgs.msg import Float64

from collections.abc import Callable

class GPSdev(GPS):
    _model = "gps"

    def __init__(self, node: Node, TIME_STEP):
        super().__init__("gps")
        self.enable(TIME_STEP)
        self.gps_coords = [0.0, 0.0, 0.0]
        self.gps_speed = 0.0

        self.display = DisplayDev(node.driver.getCurrentSpeed)

        self.node = node
        self.spd_pub = node.create_publisher(Float64, "/get_vel", 10)   # Not official
        self.spd_msg = Float64()

    def process_data(self):     # compute_gps_speed
        self.gps_speed = self.getSpeed() * 3.6
        
        self.spd_msg.data = self.gps_speed * np.sign(self.node.driver.getCurrentSpeed())    # gps_speed has no sign (direction)
        self.spd_pub.publish(self.spd_msg)
        
        self.gps_coords = self.getValues()
        self.display.update_display(self.gps_coords, self.gps_speed)



class DisplayDev(Display):
    _model = "display"

    def __init__(self, speed_func: Callable):
        super().__init__("display")
        self.getCurrentSpeed = speed_func
        self.speedometer_image = self.imageLoad("speedometer.png")

    def update_display(self, gps_coords, gps_speed):
        NEEDLE_LENGTH = 50.0
        self.imagePaste(self.speedometer_image, 0, 0, False)
        current_speed = self.getCurrentSpeed() / 5      #! problem here. why is the speed return scaled?!
        if math.isnan(current_speed):
            current_speed = 0.0
        alpha = current_speed / 260.0 * 3.72 - 0.27
        x = -NEEDLE_LENGTH * math.cos(alpha)
        y = -NEEDLE_LENGTH * math.sin(alpha)
        self.drawLine(100, 95, 100 + x, 95 + y)
        self.drawText(f"GPS coords: {gps_coords[0]:.1f} {gps_coords[2]:.1f}", 10, 130)
        self.drawText(f"GPS speed:  {gps_speed:.1f}", 10, 140)