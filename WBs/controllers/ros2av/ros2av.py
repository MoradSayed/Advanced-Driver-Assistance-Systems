#! /usr/bin/python3.10

"""
Copyright (c) 2025, Morad Sayed. BSD 3-Clause License (see LICENSE file).
"""

import math, numpy as np, rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from vehicle import Driver

from dataclasses import dataclass
from vehicle_controller import VeController
from camera_dev import CameraDev
from lidar_dev import LidarDev
from gps_dev import GPSdev, DisplayDev
from pos_dev import WheelOdom

TIME_STEP = 50

@dataclass
class Devices_DC:
    camera  : CameraDev    = None
    lidar   : LidarDev     = None
    gps     : GPSdev       = None
    # display : DisplayDev   = None
    odom    : WheelOdom    = None

class Manager(Node):
    def __init__(self, speed_range:tuple = (-20.0, 60.0), steering_range:tuple = (-0.5, 0.5)):
        rclpy.init()
        super().__init__("AV_Manager")

        self.speedlim = speed_range
        self.steerlim = steering_range
        
        self.driver = Driver()
        self.controller = VeController(TIME_STEP, speed_range, steering_range, self.driver)
        self.sensors = Devices_DC()
        self.brakes = self.controller.brakes

        for key in self.sensors.__dict__.keys():
            cls = self.sensors.__annotations__[key]
            Smodel = cls._model
            if self.driver.devices.get(Smodel, None) != None:
                setattr(self.sensors, key, cls(self, TIME_STEP))
            else:
                self.get_logger().error(f"{Smodel} was not found.")

        self.driver.setHazardFlashers(True)
        self.driver.setDippedBeams(True)
        self.driver.setAntifogLights(True)
        self.driver.setWiperMode(Driver.SLOW)

        self.create_subscription(Float64, "/cmd_vel"      , lambda value: self.controller.set_speed(value.data)           , 10)
        self.create_subscription(Float64, "/brakes"       , lambda value: self.controller.set_brake_force(value.data)     , 10)
        self.create_subscription(Float64, "/SteeringAngle", lambda value: self.controller.set_steering_angle(value.data)  , 10)

        self.print_help()
        self.run()

    def print_help(self):
        publishers = [
            ("/av_camera", "sensor_msgs/Image"),
            ("/av_lidar", "sensor_msgs/LaserScan")
        ]

        subscribers = [
            ("/cmd_vel", "std_msgs/Float64", "(-20.0, 60.0) km/h"),
            ("/SteeringAngle", "std_msgs/Float64", "(-0.5 : 0.5) rad"),
            ("/brakes", "std_msgs/Float64", "(0 : 1)")
        ]

        print("\nPublishers:")
        print(f"{'Topic Name':<20} {'Message Type':<30}")
        print("-" * 50)
        for topic, msg_type in publishers:
            print(f"{topic:<20} {msg_type:<30}")

        print("\nSubscribers:")
        print(f"{'Topic Name':<20} {'Message Type':<30} {'Range':<20}")
        print("-" * 70)
        for topic, msg_type, range in subscribers:
            print(f"{topic:<20} {msg_type:<30} {range:<20}")

    def get_device_names(self):
        device_count = self.driver.getNumberOfDevices()
        device_names = [self.driver.getDeviceByIndex(i).getName() for i in range(device_count)]
        for d in device_names:
            print(d)

    def run(self):
        i = 0
        while self.driver.step() != -1:
            if i % (TIME_STEP // int(self.driver.getBasicTimeStep())) == 0:
                for ins in self.sensors.__dict__.values():
                    ins.process_data()
                rclpy.spin_once(self, timeout_sec=0.01)
            i += 1


def main():
    Manager()

if __name__ == "__main__":
    main()