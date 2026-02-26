import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from std_msgs.msg import Float64, Bool
from custom_ros2_interfaces.srv import SetACCstate
from abc import ABC, abstractmethod

###########  A temp patch to fix gpiozero's SoC problem  ##########
import gpiozero.pins.lgpio
import lgpio

def __patched_init(self, chip=None):
    gpiozero.pins.lgpio.LGPIOFactory.__bases__[0].__init__(self)
    chip = 0
    self._handle = lgpio.gpiochip_open(chip)
    self._chip = chip
    self.pin_class = gpiozero.pins.lgpio.LGPIOPin

gpiozero.pins.lgpio.LGPIOFactory.__init__ = __patched_init
###################################################################
from gpiozero import Button, DigitalOutputDevice

class ActuatorBase(ABC):
    def __init__(self, node: Node, topic_name: str, device_name: str, 
                 killSwitch_pin: int, modeSelector_pin: int, 
                 continue_on_kill_release: bool = False, disable_first: bool = False, disable_delay_secs: float = 1.0):
        """
            Initializes the ActuatorBase node with a kill switch and mode selector for manual/auto control. Subscribes to a specified topic for actuator commands.

        :param killSwitch_pin: GPIO pin number for the kill switch button (active high)
        :type killSwitch_pin: int
        :param modeSelector_pin: GPIO pin number for the mode selector (active high for manual mode)
        :type modeSelector_pin: int
        :param topic_name: ROS topic name to subscribe for receiving actuator commands
        :type topic_name: str
        :param continue_on_kill_release: If True, the actuator will resume automatic control when the kill switch is released. 
        If False, it will remain in manual mode until an external command is received to switch back to auto mode.
        :type continue_on_kill_release: bool
        :param disable_first: If True, the kill switch immediately disables the actuator and forces a safe state (recommended for electronic actuators, e.g., DAC-controlled throttles).
        If False, the kill switch commands a safe state first, then disables the actuator after the configured delay (recommended for mechanical actuators, e.g., steppers).
        :type disable_first: bool
        :param disable_delay_secs: If disable_first is False, this sets the delay in seconds between activating the kill switch and disabling the actuator. This allows time for the actuator to be set to a safe state before being disabled.
        :type disable_delay_secs: float
        """
        self.node = node
        self.device_name = device_name

        self.triggered_state_change = False
        self.is_auto_on = False

        self.pause = continue_on_kill_release
        self.disable_first = disable_first
        self.disable_delay_secs = disable_delay_secs

        if not self.disable_first:
            self.disable_timer: Timer = self.node.create_timer(self.disable_delay_secs, self._timer_callback)
            self.disable_timer.cancel()

        self.callback_method = self._auto_mode
        self.node.create_subscription(Float64, topic_name, lambda msg: self.callback_method(msg), 10)    #? for throttle: '/cmd_pedal'

        self.kill_switch = Button(killSwitch_pin, pull_up=False, bounce_time=0.01)      # Button with internal pull-down (active high)
        self.mode_selector = DigitalOutputDevice(modeSelector_pin, active_high=True, initial_value=False)     # Digital output device for CD4053

        self.kill_switch.when_activated = lambda: self._set_mode(0)
        self.kill_switch.when_deactivated = lambda: self._set_mode(1)

        self.controller_state = self.node.create_client(SetACCstate, 'Set_ACC_State')
        self.state_req = SetACCstate.Request()
        
    def _set_mode(self, state):
        """
        sets the mode of the pedal based on the state of the kill switch.
         - If state is False (kill switch activated to pause), switch to manual mode and notify the controller.
         - If state is True (kill switch deactivated to resume), switch to auto mode and notify the controller.
        
        :param state: False for manual mode (kill switch activated), True for auto mode (kill switch deactivated)
        """
        if not state:
            self.callback_method = lambda _: None
            self._manual_mode()
            self.triggered_state_change = True
            self.state_req.set_state = int(self.pause)    # Req a stop/pause state from controller (0:stop, 1:pause)
            self.controller_state.call_async(self.state_req)
            self.node.get_logger().info(f"{self.device_name} -> manual with request")
        else:
            if self.pause: #? resume after letting go of the kill switch
                self.callback_method = self._auto_mode
                self.triggered_state_change = True
                self.state_req.set_state = 2    # Req a resume state from controller
                self.controller_state.call_async(self.state_req)
                self.node.get_logger().info(f"{self.device_name} -> auto with request")
            #? else: stay disconnected after letting go of the kill switch (keep manual mode)
            else:
                self.node.get_logger().info(f"{self.device_name} -> No change (stays in manual mode)")
        
    def _manual_mode(self, msg=None):
        if self.disable_first:
            self.mode_selector.off()
            self.set_abs_value(0)
        else:
            self.set_abs_value(0)
            if rclpy.ok():
                self.disable_timer.reset()

        self.is_auto_on = False

    def _timer_callback(self):
        self.disable_timer.cancel()
        self.mode_selector.off()

    def _auto_mode(self, msg):
        # self.node.get_logger().info("called")
        abs_position = msg.data      # Expecting a value between 0.0 and 1.0
        self.set_abs_value(abs_position)
        
        if not self.is_auto_on:
            self.mode_selector.on()
            self.is_auto_on = True

    @abstractmethod
    def set_abs_value(self, abs_value: float):
        """
        # IMPORTANT NOTE:
        > This method should be overridden in the derived class to provide the specific implementation for setting the actuator's absolute value based on the received command.
        ---

        Sets the actuator to a specific absolute value (e.g., steering angle or pedal position).
        
        :param abs_value: The target absolute value for the actuator
        :type abs_value: float
        """
        pass

    def destroy_dev(self):
        self._manual_mode()  # Set to manual mode to ensure outputs are safe
        # gpiozero cleans up automatically on program exit; explicitly close devices
        self.mode_selector.close()
        self.kill_switch.close()
