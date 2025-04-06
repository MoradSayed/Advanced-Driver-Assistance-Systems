import math
from rclpy.node import Node
from std_msgs.msg import Float64
from controller import PositionSensor

class WheelOdom:
    _model = "left_front_sensor" # or "right_front_sensor"
    def __init__(self, node: Node, TIME_STEP, 
                 left_sensor="left_front_sensor", right_sensor="right_front_sensor", 
                 wheel_radius=0.3):
        
        self.node = node
        self.publisher_ = node.create_publisher(Float64, '/odom', 10)
        self.pub_msg = Float64()

        self.left = PositionSensor(left_sensor)
        self.right = PositionSensor(right_sensor)

        self.left.enable(TIME_STEP)
        self.right.enable(TIME_STEP)

        self.prev_left = 0.0
        self.prev_right = 0.0

        self.wheel_radius = wheel_radius
        self.total_distance = 0.0

    def process_data(self):
        curr_left = self.left.getValue()
        curr_right = self.right.getValue()
        
        if math.isnan(curr_left) and math.isnan(curr_right): 
            self.node.get_logger().warn("Invalid sensor reading, skipping this cycle.")
            return
        
        delta_left = curr_left - self.prev_left
        delta_right = curr_right - self.prev_right

        avg_rotation = (delta_left + delta_right) / 2.0
        distance_step = self.wheel_radius * avg_rotation
        self.total_distance += abs(distance_step)
        
        self.pub_msg.data = float(self.total_distance)
        self.publisher_.publish(self.pub_msg)

        # print(f"{curr_left} & {curr_right} >> {delta_left} | {delta_right} >> {distance_step} / {self.total_distance}")
        self.prev_left = curr_left
        self.prev_right = curr_right

    def get_total_distance(self):
        return self.total_distance
