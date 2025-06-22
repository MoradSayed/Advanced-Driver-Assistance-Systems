from vehicle import Driver
from controller import Keyboard, Brake
import numpy as np

class VeController:
    def __init__(self, TIME_STEP, speedlim, steerlim, driver:Driver, debug:bool = True):       
        self.driver = driver
        self.speedlim:tuple = speedlim
        self.steerlim:tuple = steerlim
        self.brakes = self._init_brakes_()
        
        self.keyboard = Keyboard()
        self.keyboard.enable(TIME_STEP)
        
        # Misc variables
        self.speed = 0.0
        self.steering_angle = 0.0

    def set_speed(self, kmh):
        kmh = max(self.speedlim[0], min(self.speedlim[1], kmh))

        # print(f"setting speed to {kmh} km/h")
        self.speed = kmh
        self.driver.setCruisingSpeed(kmh)

    def set_steering_angle(self, rad):
        rad = max(self.steerlim[0], min(self.steerlim[1], rad))

        self.steering_angle = rad
        self.driver.setSteeringAngle(rad)

    def _init_brakes_(self):
        lf = "left_front_brake"
        rf = "right_front_brake"
        lr = "left_rear_brake"
        rr = "right_rear_brake"
        brakes = [Brake(lf), Brake(rf), Brake(lr), Brake(rr)]
        return brakes
    
    def set_brake_force(self, normalized_force):
        """
        **Normalized_force** (int):  
        a value from 0 (no brakes) to 1 (full brakes)
        """
        for b in self.brakes:
            b.setDampingConstant(normalized_force*1800)     # 1800 from proto `brakeCoefficient`
