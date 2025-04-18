from rclpy.node import Node
from std_msgs.msg import Float64

from .kbh import KeyboardHandler
from .adas import ADAS

class Manager:
    def __init__(self, node: Node, use_kb:bool):
        self.node = node
        
        self.vel_pub = node.create_publisher(Float64, "/cmd_vel"      , 10)
        self.brk_pub = node.create_publisher(Float64, "/brakes"       , 10)
        self.str_pub = node.create_publisher(Float64, "/SteeringAngle", 10)

        self.spd_msg = Float64()
        self.brk_msg = Float64()
        self.str_msg = Float64()
        
        #* Manual driving
        self.vdriver = VehicleDriver(self)
        self.node.create_timer(0.01, lambda: print(self.vdriver, end=""))
        node.create_subscription(Float64, "/get_vel", lambda val: self.vdriver._set_act_speed(val.data), 10)
        if use_kb:
            KeyboardHandler(
                up_func         = self.vdriver.accelerate_func,
                down_func       = self.vdriver.reverse_func,
                left_func       = self.vdriver.left_func,
                right_func      = self.vdriver.right_func,
                neutral_ud_func = self.vdriver.neu_ud,
                neutral_lr_func = self.vdriver.neu_lr,
            )

        #* ADAS
        self.adas = ADAS(self)

    def speed_manager(self, value: float):
        self.spd_msg.data = value
        self.vel_pub.publish(self.spd_msg)

    def brake_manager(self, value: float):
        self.brk_msg.data = value
        self.brk_pub.publish(self.brk_msg)

    def steer_manager(self, value: float):
        self.str_msg.data = value
        self.str_pub.publish(self.str_msg)
    

class VehicleDriver:
    def __init__(self, manager: Manager):
        self.manager = manager
        
        self.speed = 0.0
        self.brake = 0.0
        self.steer = 0.0

        self.act_spd = 0.0

    def _set_act_speed(self, speed):
        self.act_spd = speed

    def accelerate_func(self):
        if self.act_spd > -0.1:
            self.brake = 0.0
            self.speed = 60.0
        else:
            self.speed = 0.0
            self.brake = 1.0

        self.manager.speed_manager(self.speed)
        self.manager.brake_manager(self.brake)

    def reverse_func(self):
        if self.act_spd > 0.1:
            self.speed = 0.0
            self.brake = 1.0
        else:
            self.brake = 0.0
            self.speed = -20.0

        self.manager.speed_manager(self.speed)
        self.manager.brake_manager(self.brake)

    def right_func(self):
        self.steer = (+0.5 * (80.0 - self.act_spd) / 80.0)
        self.manager.steer_manager(self.steer)

    def left_func(self):
        self.steer = (-0.5 * (80.0 - self.act_spd) / 80.0)
        self.manager.steer_manager(self.steer)

    def neu_ud(self):
        # self.speed *= 0.99995
        # return
        
        #* this one
        self.speed = 0.0
        self.brake = 0.0
        self.manager.speed_manager(self.speed)
        self.manager.brake_manager(self.brake)

    def neu_lr(self):
        # sign = -1 * np.sign(self.steer)
        # if np.sign(self.steer + (sign * 0.01745)) == np.sign(self.steer):
        #     self.steer = (self.steer + (sign * 0.01745))
        # else:
        #     self.steer = 0.0
        # return
        
        #* this one
        self.steer = 0.0
        self.manager.steer_manager(self.steer)

    def __repr__(self):
        return f"\r\033[K"+f"vel: {self.speed}, brk: {self.brake}, str: {self.steer:.2f}"
    