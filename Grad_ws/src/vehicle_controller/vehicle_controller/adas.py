from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .manager import Manager

class ADAS:
    def __init__(self, manager:"Manager"):
        self.speed_pub = manager.speed_manager
        self.brake_pub = manager.brake_manager
        self.steer_pub = manager.steer_manager

        self.actual_speed = lambda: manager.vdriver.act_spd

        # Write your algorithm here
