#! /usr/bin/python3.10

import rclpy, time
from rclpy.node import Node
import argparse

from .manager import Manager

class Controller(Node):
    def __init__(self, use_kb):
        super().__init__("Vehicle_Controller")
        self.man = Manager(self, use_kb)

    def run(self):
        rclpy.spin(self)        
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument('--kb', type=int, default=1)
    parsed_args, _ = parser.parse_known_args()
    
    cont = Controller(bool(parsed_args.kb))
    cont.run()
