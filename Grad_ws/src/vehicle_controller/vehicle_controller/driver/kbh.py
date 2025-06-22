from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..manager import Manager

from std_msgs.msg import Float64
from pynput import keyboard

class KeyboardHandler:
    """
    Used to listen for presses and releases to allow for the control of the vehicle with a keyboard
    """

    KEY_MAP = {
        keyboard.Key.up   : 1 << 3, # 1000 (8)
        keyboard.Key.down : 1 << 2, # 0100 (4)
        keyboard.Key.left : 1 << 1, # 0010 (2)
        keyboard.Key.right: 1 << 0  # 0001 (1)
    }

    def __init__(self, manager:"Manager"):

        self.vdriver = VehicleDriver(manager)
        manager.node.create_subscription(Float64, "/get_vel", lambda val: self.vdriver._set_act_speed(val.data), 10)

        self.pressed_keys = 0  # Stores key states as a 4-bit integer
        self.func_map = {
            keyboard.Key.up:    self.vdriver.accelerate_func,
            keyboard.Key.down:  self.vdriver.reverse_func,
            keyboard.Key.left:  self.vdriver.left_func,
            keyboard.Key.right: self.vdriver.right_func
        }
        self.neutral_ud_func = self.vdriver.neu_ud  # No UP or DOWN pressed
        self.neutral_lr_func = self.vdriver.neu_lr  # No LEFT or RIGHT pressed

        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()  # Non-blocking

    def on_press(self, key):
        if key in self.KEY_MAP:
            key_bit = self.KEY_MAP[key]
            if not (self.pressed_keys & key_bit):  # If key is NOT already pressed
                self.pressed_keys |= key_bit  # Mark key as pressed
                self.process_action(key,1)

    def on_release(self, key):
        if key in self.KEY_MAP:
            self.pressed_keys &= ~self.KEY_MAP[key]  # Clear the corresponding bit
            self.process_action(key,0)

    def process_action(self, key, state):
        """
        state: press(1) or release(0)
        """

        if key in (keyboard.Key.up, keyboard.Key.down):
            is_both_ud = int(bool(self.pressed_keys & self.KEY_MAP[keyboard.Key.up]) == bool(self.pressed_keys & self.KEY_MAP[keyboard.Key.down]))
            if is_both_ud:
                self.neutral_ud_func()          # Reset
            elif state:
                self.func_map[key]()            # call corresponding function
            else:
                other_key = keyboard.Key.down if key == keyboard.Key.up else keyboard.Key.up   # swap keys
                self.func_map[other_key]()      # call other function
        else:
            is_both_lr = int(bool(self.pressed_keys & self.KEY_MAP[keyboard.Key.left]) == bool(self.pressed_keys & self.KEY_MAP[keyboard.Key.right]))
            if is_both_lr:
                self.neutral_lr_func()
            elif state:
                self.func_map[key]()
            else:
                other_key = keyboard.Key.right if key == keyboard.Key.left else keyboard.Key.left
                self.func_map[other_key]()

    def get_binary_state(self):
        return format(self.pressed_keys, '04b')         # Convert integer to binary string

    def __repr__(self):
        return self.vdriver

class VehicleDriver:
    """
    Used to simulate an analog control with a digital device like a `keyboard`
    """
    #! still need some performance optimizations
    def __init__(self, manager: "Manager"):
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
            self.speed = 1.0
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
            self.speed = -0.3

        self.manager.speed_manager(self.speed)
        self.manager.brake_manager(self.brake)

    @request_y_action
    def right_func(self):
        self.steer = (80.0 - self.act_spd) / 80.0
        self.manager.steer_manager(self.steer)

    @request_y_action
    def left_func(self):
        self.steer = - (80.0 - self.act_spd) / 80.0
        self.manager.steer_manager(self.steer)

    @request_x_action
    def neu_ud(self):
        self.speed = 0.0
        self.brake = 0.0
        self.manager.speed_manager(self.speed)
        self.manager.brake_manager(self.brake)

    @request_y_action
    def neu_lr(self):
        self.steer = 0.0
        self.manager.steer_manager(self.steer)

    def _set_act_speed(self, speed):
        """ 
        ### DON'T USE THIS VARIABLE FOR VEHICLE PURPOSES  
        ONLY USED TO PROVIDE BETTER KEYBOARD CONTROLS
        """
        self.act_spd = speed

    def __repr__(self):
        return f"vel: {self.speed}, brk: {self.brake}, str: {self.steer:.2f}"
