from rclpy.node import Node

from vehicle_interface.Actuator_base import ActuatorBase
from vehicle_interface.attiny85_comms import bus, set_angle

# Constants
ANGLE_RANGE = 360*3   # full angle range
ANGLE_OFFSET = 0.0      # offset at the beginning

# gpiozero devices
SWITCH_PIN = 13
CD4053_PIN_C = 12

class Brakes(ActuatorBase):
    def __init__(self, node: Node):
        super().__init__(
            node = node,
            topic_name = '/brakes',
            device_name = "Brakes",
            killSwitch_pin = SWITCH_PIN,
            modeSelector_pin = CD4053_PIN_C,
            continue_on_kill_release = False,
            disable_first = False,
            disable_delay_secs = 2.0
        )

    def set_abs_value(self, abs_value: float):
        degrees = -(ANGLE_OFFSET + (abs_value*(ANGLE_RANGE-ANGLE_OFFSET)))   # Invert direction if needed (depends on your motor wiring and desired steering direction)
        set_angle(bus, int(degrees))

    def destroy_dev(self):
        super().destroy_dev()
        try:           
            bus.close()
        except Exception:
            pass


# if __name__ == '__main__':
    # main()
