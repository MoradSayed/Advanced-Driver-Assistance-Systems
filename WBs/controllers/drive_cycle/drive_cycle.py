#! /usr/bin/python3.10

"""
Copyright (c) 2025, Morad Sayed. BSD 3-Clause License (see LICENSE file).
"""

import rclpy, os
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from vehicle import Driver
import csv

TIME_STEP = 50

def load_drive_cycle_csv(path):
    with open(path) as f:
        reader = csv.reader(f)
        next(reader)
        return [(float(row[0]), float(row[1])) for row in reader]

class Cycle(Node):
    def __init__(self):
        rclpy.init()
        super().__init__("Lead_Vehicle")
        self.driver = Driver()
        self.vel_pub = self.create_publisher(Float32, "/lead_vel", 10)
        self.vel_msg = Float32()

        self.driver.setDippedBeams(True)
        self.driver.setAntifogLights(True)

        #? Method 1
        # self.drive_cycle = [(0, 80)]

        #? Method 2
        self.drive_cycle = load_drive_cycle_csv(os.path.join(os.path.dirname(__file__), "European_Transient_Cycle_(ETC)-p2.csv"))
        self.current_speed = 30.09    #? first speed in the csv file

        self.index = 0

        self.remote_controller()
        self.run()

    def get_target_speed(self):
        offset = 1.766
        if self.index < len(self.drive_cycle) and self.drive_cycle[self.index][0]-offset < self.driver.getTime():
            self.current_speed = self.drive_cycle[self.index][1]
            self.index+=1

    def remote_controller(self):
        self.create_subscription(Bool, "/pause"   , lambda value: self.driver.simulationSetMode(self.driver.SIMULATION_MODE_PAUSE)    , 10)
        self.create_subscription(Bool, "/realtime", lambda value: self.driver.simulationSetMode(self.driver.SIMULATION_MODE_REAL_TIME), 10)
        self.create_subscription(Bool, "/fast"    , lambda value: self.driver.simulationSetMode(self.driver.SIMULATION_MODE_FAST)     , 10)

    def run(self):
        i = 0
        while self.driver.step() != -1:
            if i % (TIME_STEP // int(self.driver.getBasicTimeStep())) == 0:
                self.get_target_speed()
                self.driver.setCruisingSpeed(self.current_speed)
                self.vel_msg.data = self.current_speed
                # self.vel_msg.data = self.driver.getCurrentSpeed()
                self.vel_pub.publish(self.vel_msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            i += 1

def main():
    Cycle()

if __name__ == "__main__":
    main()
