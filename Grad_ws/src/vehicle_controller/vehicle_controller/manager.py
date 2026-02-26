from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from custom_ros2_interfaces.srv import GetACCparams, SetACCstate

import platform, time, os, json
from datetime import datetime
is_rpi = not (platform.system() != "Linux" or "aarch64" not in platform.machine())
if not is_rpi:
    # Likely not a Raspberry Pi → safe to import pynput
    from .driver.kbh import KeyboardHandler
from .driver.app import APKController
from .adas import ADAS

MAX_STEER_ANGLE = 0.5    # Fixed - based on vehicle

class Manager:
    def __init__(self, node: Node, use_kb:bool):
        self.node = node
        self.adas = None
        
        self.params_client = node.create_client(GetACCparams, 'Get_ACC_Params')
        self.state_service = node.create_service(SetACCstate, 'Set_ACC_State', self.state_manager)

        self.vel_pub = node.create_publisher(Float64, "/cmd_vel"      , 10)
        self.pdl_pub = node.create_publisher(Float64, "/cmd_pedal"    , 10)
        self.brk_pub = node.create_publisher(Float64, "/brakes"       , 10)
        self.str_pub = node.create_publisher(Float64, "/SteeringAngle", 10)
        self.spd_msg = Float64()
        self.pdl_msg = Float64()
        self.brk_msg = Float64()
        self.str_msg = Float64()
        
        self.actuator_state_pub = node.create_publisher(Bool, "/actuator_state", 10)
        self.actuator_state_msg = Bool()

        #* Manual driving
        if use_kb:
            if not is_rpi:
                KeyboardHandler(manager = self)
        else:
            APKController(manager= self)

        node.declare_parameter('is_sim', False)    
        self.is_sim = node.get_parameter('is_sim').get_parameter_value().bool_value

        node.declare_parameter('speed', 0)
        node.declare_parameter('level', 0.0)
        if not self.is_sim:
            self.req_params_from_lcd()
        else:
            self.req_speed = node.get_parameter('speed').get_parameter_value().integer_value
            self.req_tgap = node.get_parameter('level').get_parameter_value().double_value
            print(self.req_speed, self.req_tgap)
            self.adas = ADAS(self, self.req_speed, self.req_tgap, self.is_sim)

        #!### TESTING ONLY - to be removed later
        # self.timer_test = self.node.create_timer(3, self.brakes_test)
        #!### TESTING ONLY - to be removed later ###!#

    def req_params_from_lcd(self):
        while not self.params_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for ACC service...')
        req = GetACCparams.Request()
        future = self.params_client.call_async(req)
        future.add_done_callback(self.start_acc)

    def start_acc(self, future):
        response = future.result()
        self.req_speed = response.speed
        self.req_tgap = response.level
        self.node.get_logger().info(f"From control center: {self.req_speed}, {self.req_tgap}")
        self.adas = ADAS(self, self.req_speed, self.req_tgap, self.is_sim)

    def state_manager(self, request: SetACCstate.Request, response: SetACCstate.Response):
        x = request.set_state
        if x == 0: # Stop
            # self.node.get_logger().info("############ STOPPED ############")
            self.set_actuators_state(False)
            if self.adas:
                self.adas.toggle_acc(False)
            self.req_params_from_lcd()
        elif x == 1: # Pause
            # self.node.get_logger().info("############ PAUSED ############")
            self.set_actuators_state(False)
            if self.adas:
                self.adas.toggle_acc(False)
        elif x == 2: # Resume
            # self.node.get_logger().info("############ RESUMED ############")
            self.set_actuators_state(True)
            if self.adas:
                self.adas.toggle_acc(True)
        elif x == 3: # Start
            # self.node.get_logger().info("############ STARTED ############")
            pass

        return response

    def speed_manager(self, value: float):
        # self.spd_msg.data = value * self.req_speed
        # self.vel_pub.publish(self.spd_msg)
        self.pdl_msg.data = float(value)
        self.pdl_pub.publish(self.pdl_msg)

    def brake_manager(self, value: float):
        self.brk_msg.data = abs(value)
        self.brk_pub.publish(self.brk_msg)

    def steer_manager(self, value: float):
        self.str_msg.data = value * MAX_STEER_ANGLE
        self.str_pub.publish(self.str_msg)

    def set_actuators_state(self, state: bool):
        self.actuator_state_msg.data = state
        self.actuator_state_pub.publish(self.actuator_state_msg)

    #!### TESTING ONLY
    def brakes_test(self):
        # self.brake_manager(float(-2))
        self.timer_test.destroy()
        self.node.get_logger().info("Testing method called - simulating brake commands")
        # angles = [0.25, 0.5, 0.75, 0.125+0.75, 0.0]
        self.node.get_logger().info(f"####################### throttle #######################")
        # for i in range(10, 20, 1): # -0.5
        #     self.speed_manager(float(i/100))
        #     self.node.get_logger().info(f"speed at {i}%")
        #     time.sleep(0.1)
        self.speed_manager(float(0.4))
        time.sleep(6)
        self.speed_manager(float(0.0))
        self.node.get_logger().info(f"####################### brakes #######################")
        for i in range(10, 100, 1): # -0.5
            self.brake_manager(float(-i/100))
            self.node.get_logger().info(f"brakes at {i}%")
            time.sleep(0.02)
    #!### TESTING ONLY ###!#
