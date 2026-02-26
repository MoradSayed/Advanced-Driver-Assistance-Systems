import rclpy
from rclpy.node import Node
from custom_ros2_interfaces.srv import SetACCstate
from std_msgs.msg import Float64, Bool

from vehicle_interface.accelrator import Accelrator
from vehicle_interface.brakes import Brakes

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from vehicle_interface.Actuator_base import ActuatorBase

class ActuatorNode(Node):
    def __init__(self):
        super().__init__('actuators_node')
        self.actuators: list["ActuatorBase"] = [Accelrator(self), Brakes(self)]
        
        self.create_subscription(Bool, '/actuator_state', lambda msg: self.set_actuator_state(msg.data), 10)

    def set_actuator_state(self, state: bool):
        for actuator in self.actuators:
            if not actuator.triggered_state_change:
                if state:
                    actuator.callback_method = actuator._auto_mode
                    self.get_logger().info(f"{actuator.device_name} -> force auto")
                else:
                    actuator.callback_method = lambda _: None
                    actuator._manual_mode()
                    self.get_logger().info(f"{actuator.device_name} -> force manual")
            else:
                actuator.triggered_state_change = False  # Reset for next time

    def destroy_node(self):
        for actuator in self.actuators:
            actuator.destroy_dev()
        super().destroy_node()

def main():
    rclpy.init()
    node = ActuatorNode()
    node.get_logger().info("Actuators are ready.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
