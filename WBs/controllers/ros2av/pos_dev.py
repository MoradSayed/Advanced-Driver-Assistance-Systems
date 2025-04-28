import math
from rclpy.node import Node
from example_interfaces.msg import Float64MultiArray
from controller import PositionSensor

class WheelOdom:
    _model = "left_front_sensor" # or "right_front_sensor"
    def __init__(self, node: Node, TIME_STEP, 
                 left_sensor="left_front_sensor", right_sensor="right_front_sensor"):
        
        self.node = node
        self.publisher_ = node.create_publisher(Float64MultiArray, '/wheel_speed', 10)
        self.pub_msg = Float64MultiArray()

        self.left = PositionSensor(left_sensor)
        self.right = PositionSensor(right_sensor)

        self.left.enable(TIME_STEP)
        self.right.enable(TIME_STEP)

    def process_data(self):
        curr_left = self.left.getValue()
        curr_right = self.right.getValue()
        
        if math.isnan(curr_left) or math.isnan(curr_right): 
            self.node.get_logger().warn("Invalid sensor reading, skipping this cycle.")
            return
        
        self.pub_msg.data = [curr_left, curr_right]
        self.publisher_.publish(self.pub_msg)
