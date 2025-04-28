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
        node.create_subscription(Float64, "/get_vel", lambda val: self.vdriver._set_act_speed(val.data), 10)
        if use_kb:
            self.node.create_timer(0.05, lambda: print(self.vdriver, end=""))
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
    #! self.act_spd doesn't give direction (no sign)
    #! still need some performance optimizations
    def __init__(self, manager: Manager):
        self.manager = manager
        
        self.speed = 0.0
        self.brake = 0.0
        self.steer = 0.0

        self.act_spd = 0.0

        self.x_command = lambda _: None
        self.y_command = lambda _: None
        self.tx, self.ty = True, True
        self.timer = manager.node.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        if self.tx:
            self.x_command(self)
        if self.ty:
            self.y_command(self)

    def request_x_action(func):
        def wrapper(self: "VehicleDriver", *args, **kwargs):
            self.tx = False     # cancel
            func(self, *args, **kwargs)
            self.x_command = func
            self.tx = True      # reset
        return wrapper

    def request_y_action(func):
        def wrapper(self: "VehicleDriver", *args, **kwargs):
            self.ty = False     # cancel
            func(self, *args, **kwargs)
            self.y_command = func
            self.ty = True      # reset
        return wrapper

    @request_x_action
    def accelerate_func(self):
        if self.act_spd > -0.1:
            self.brake = 0.0
            self.speed = 60.0
        else:
            self.speed = 0.0
            self.brake = 1.0

        self.manager.speed_manager(self.speed)
        self.manager.brake_manager(self.brake)

    @request_x_action
    def reverse_func(self):
        if self.act_spd > 0.1:
            self.speed = 0.0
            self.brake = 1.0
        else:
            self.brake = 0.0
            self.speed = -20.0

        self.manager.speed_manager(self.speed)
        self.manager.brake_manager(self.brake)

    @request_y_action
    def right_func(self):
        self.steer = (+0.5 * (80.0 - self.act_spd) / 80.0)
        self.manager.steer_manager(self.steer)

    @request_y_action
    def left_func(self):
        self.steer = (-0.5 * (80.0 - self.act_spd) / 80.0)
        self.manager.steer_manager(self.steer)

    @request_x_action
    def neu_ud(self):
        # self.speed *= 0.99995
        # return
        
        #* this one
        self.speed = 0.0
        self.brake = 0.0
        self.manager.speed_manager(self.speed)
        self.manager.brake_manager(self.brake)

    @request_y_action
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

    def _set_act_speed(self, speed):
        """ DON'T USE FOR CONTROLLING PURPOSES, ONLY USED FOR KEYBOARD CONTROLS AND PLOTTING """
        self.act_spd = speed

    def __repr__(self):
        return f"\r\033[K"+f"vel: {self.speed}, brk: {self.brake}, str: {self.steer:.2f}"
    