#! /usr/bin/python3.10

"""
Copyright (c) 2025, Morad Sayed. BSD 3-Clause License (see LICENSE file).
"""

import rclpy, os
from rclpy.node import Node
from std_msgs.msg import Float32
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

        self.run()

    def get_target_speed(self):
        offset = 1.766
        if self.index < len(self.drive_cycle) and self.drive_cycle[self.index][0]-offset < self.driver.getTime():
            self.current_speed = self.drive_cycle[self.index][1]
            self.index+=1

    def run(self):
        i = 0
        while self.driver.step() != -1:
            if i % (TIME_STEP // int(self.driver.getBasicTimeStep())) == 0:
                self.get_target_speed()
                self.driver.setCruisingSpeed(self.current_speed)
                self.vel_msg.data = self.current_speed
                # self.vel_msg.data = self.driver.getCurrentSpeed()
                self.vel_pub.publish(self.vel_msg)
            i += 1
            # if self.driver.getTime() > 12:
            #     self.driver.simulationReset()

def main():
    Cycle()

if __name__ == "__main__":
    main()
