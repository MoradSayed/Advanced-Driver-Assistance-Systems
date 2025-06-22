from rclpy.node import Node
from std_msgs.msg import Float64

from .driver.kbh import KeyboardHandler
from .driver.app import APKController
from .ros_bridge import ADAS  # type: ignore

MAX_VELOCITY = 100       # Adjustable
MAX_STEER_ANGEL = 0.5   # Fixed - based on vehicle

class Manager:
    def __init__(self, node: Node, use_kb:bool):
        self.node = node
        
        self.vel_pub = node.create_publisher(Float64, "/cmd_vel"      , 10)
        self.brk_pub = node.create_publisher(Float64, "/brakes"       , 10)
        self.str_pub = node.create_publisher(Float64, "/SteeringAngle", 10)
        self.vel_factor = MAX_VELOCITY
        self.str_factor = MAX_STEER_ANGEL

        self.spd_msg = Float64()
        self.brk_msg = Float64()
        self.str_msg = Float64()
        
        #* Manual driving
        if use_kb:
            KeyboardHandler(manager = self)
        else:
            APKController(manager= self)
        # self.node.create_timer(0.05, lambda: print(f"\r\033[K"+log_str, end=""))  #TODO: A Centralized logging method

        #* ADAS
        self.adas = ADAS(self)

    def speed_manager(self, value: float):
        self.spd_msg.data = value * self.vel_factor
        self.vel_pub.publish(self.spd_msg)

    def brake_manager(self, value: float):
        self.brk_msg.data = value
        self.brk_pub.publish(self.brk_msg)

    def steer_manager(self, value: float):
        self.str_msg.data = value * self.str_factor
        self.str_pub.publish(self.str_msg)
