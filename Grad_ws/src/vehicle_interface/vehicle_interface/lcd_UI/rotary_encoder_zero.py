
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

from time import sleep
from gpiozero import RotaryEncoder, Button

###########  to fix Enc (2 physical -> 1 logic) problem  ##########
def _half_step_update(self):
    # Called whenever A or B changes
    if self.a.is_active == self.b.is_active:
        self._ticks += 1
    else:
        self._ticks -= 1

# Apply the monkey patch
RotaryEncoder._update = _half_step_update
###################################################################


class REncoder:
    def __init__(self, btn_callback= None):

        self.encoder = RotaryEncoder(a=17, b=18)
        self.counter = 0
    
        self.button = Button(27, bounce_time=0.01)
        self.button.when_activated = self.reset_counter if btn_callback == None else btn_callback

    def rotary_change(self):
        self.encoder.wait_for_rotate()
        # self.counter += self.encoder.steps
        step_holder = self.encoder.steps
        self.encoder.steps = 0
        return step_holder

    def reset_counter(self):
        self.counter = 0
        print("Counter reset")

if __name__ == "__main__":
    REncoder(lambda: print("pressed"))