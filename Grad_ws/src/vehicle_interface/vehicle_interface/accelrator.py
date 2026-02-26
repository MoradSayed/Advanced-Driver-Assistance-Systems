from rclpy.node import Node

import board
import busio
from adafruit_mcp4725 import MCP4725

from vehicle_interface.Actuator_base import ActuatorBase

# Constants
VOLTAGE_RANGE = 5.0     # full voltage range
VOLTAGE_OFFSET = 0.9    # offset at the beginning

# gpiozero devices
SWITCH_PIN = 26
CD4053_PIN_B = 16

class Accelrator(ActuatorBase):
    def __init__(self, node: Node):
        i2c = busio.I2C(board.SCL, board.SDA)   # Initialize I2C bus
        self.dac = MCP4725(i2c, address=0x60)   # Initialize DAC

        super().__init__(
            node = node,
            topic_name = '/cmd_pedal',
            device_name = "Accelerator",
            killSwitch_pin = SWITCH_PIN,
            modeSelector_pin = CD4053_PIN_B,
            continue_on_kill_release = True,
            disable_first = True,
            disable_delay_secs = None
        )

    def set_abs_value(self, abs_value: float):
        self.dac.normalized_value = (VOLTAGE_OFFSET + (abs_value*(VOLTAGE_RANGE-VOLTAGE_OFFSET))) / VOLTAGE_RANGE

    def destroy_dev(self):
        super().destroy_dev()
        try:           
            if board:
                board.exit()
        except Exception:
            pass


# if __name__ == '__main__':
    # main()
