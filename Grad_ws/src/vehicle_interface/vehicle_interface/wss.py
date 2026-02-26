from rclpy.node import Node
from std_msgs.msg import Float32

from vehicle_interface.attiny85_comms import bus, read_speed

class WSS:
    def __init__(self, node: Node):
        self.node = node
        
        self.speed_pub = node.create_publisher(Float32, "/ego_velocity", 10)
        self.speed_msg = Float32()

        self.wss_timer = node.create_timer(0.06, self.run)

    def run(self):
        self.speed_msg.data = float(read_speed(bus))
        self.speed_pub.publish(self.speed_msg)
