#! /usr/bin/python3.10

import rclpy, time
from rclpy.node import Node

from manager import Manager

class Controller(Node):
    def __init__(self):
        super().__init__("Vehicle_Controller")
        Manager(self)

    def run(self):
        rclpy.spin(self)        
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    cont = Controller()
    cont.run()

if __name__ == "__main__":
    main()