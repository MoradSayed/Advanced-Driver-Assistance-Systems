from vehicle_interface import drivers as drivers
from vehicle_interface.drivers import UI_custom_characters
from .rotary_encoder_zero import REncoder
# from launch_prog import launch_sys
from custom_ros2_interfaces.srv import GetACCparams

from time import sleep
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32

import vehicle_interface.radar as radar 
from vehicle_interface.wss import WSS

class UI_Prog(Node):
    def __init__(self):
        super().__init__("LCD_Logger")
        self.service = self.create_service(GetACCparams, 'Get_ACC_Params', self.run)
        self.sensors_started: bool = False

        self.speed_sub:Node.create_subscription = None
        self.dist_sub :Node.create_subscription = None

        self.display = drivers.Lcd()
        self.wel_screen()

        self.req_speed = 60
        self.req_tgap = 1.5
        self.next = False

        self.exec_list = [
            (self.speed_selection, self.update_speed), 
            (self.timegap_selection, self.update_timegap)
        ]

        try:
            self.selector = REncoder(self.next_screen)
            rclpy.spin(self)
        
        except KeyboardInterrupt:
            pass

        # except Exception as e:
        #     print(f"try-except Error: {e}")

        finally:
            self.display.lcd_clear()
            self.display.lcd_backlight(0)
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
                

    def lcd_print(self, top=None, bottom=None, delay=1):
        self.display.lcd_clear()
        self.display.lcd_display_string('{:^16}'.format(top), 1)
        # scroll second line if more than 16 chars
        if len(bottom) > 16:
            self.display.lcd_display_string(bottom[:16], 2)
            for i in range(len(bottom) - 15):
                self.display.lcd_display_string(bottom[i:i+16], 2)
                sleep(0.5)
        else:
            self.display.lcd_display_string('{:^16}'.format(bottom), 2)
        sleep(delay)

    def next_screen(self):
        # print("NEXT")
        self.next = True
        self.selector.encoder._rotate_event.set()
        self.selector.encoder._rotate_event.clear()

    def wel_screen(self):
        self.lcd_print(top="WELCOME", bottom="starting system", delay=0.5)
        self.lcd_print(top="WELCOME", bottom="starting system...", delay=0)

    def speed_selection(self, update=False):
        if not update:
            self.display.lcd_display_extended_string("Set Speed    "+"{0x00}{0x01}{0x02}", 1)
        self.display.lcd_display_extended_string(f"> {self.req_speed}         "+"{0x03}{0x04}{0x05}", 2)
        # print(f"Edited, {self.req_speed=}")

    def update_speed(self, delta):
        self.req_speed += delta

    def timegap_selection(self, update=False):
        if not update:
            self.display.lcd_display_extended_string("Set Level    "+"{0x00}{0x01}{0x02}", 1)
        self.display.lcd_display_extended_string(f"> {self.req_tgap}s       "+"{0x03}{0x04}{0x05}", 2)
        # print(f"Edited, {self.req_tgap=}")

    def update_timegap(self, delta):
        self.req_tgap += delta

    def run(self, request: GetACCparams.Request, response: GetACCparams.Response):
        #!################# TOBEREMOVED - FOR Testing without the UI
        # radar.RadarDev(self)
        # WSS(self)
        # return self.send_data(response, 40, 2)
        #!#################
        if not self.speed_sub is None and not self.dist_sub is None:
            self.destroy_subscription(self.speed_sub)
            self.destroy_subscription(self.dist_sub)
            self.selector.button.when_activated = self.next_screen

        for screen, update in self.exec_list:
            self.next = False
            screen()
            while not self.next:
                change = self.selector.rotary_change()
                # print(f"{change=}")
                if change and not self.next:
                    update(change)
                    screen(True)
        response_result = self.send_data(response, self.req_speed, self.req_tgap)
        self.display.lcd_clear()
        self.display.lcd_backlight(1)

        # start sensors
        if not self.sensors_started:
            radar.RadarDev(self)
            WSS(self)
            self.sensors_started = True

        # Continue Monitoring simulation data
        self.speed_sub = self.create_subscription(Float32, "/ego_velocity", lambda s: self.display.lcd_display_string(f"Speed      {s.data:.2f}", 1), 2)
        self.dist_sub = self.create_subscription(Float32, "/radar_min", lambda d: self.display.lcd_display_string(f"Dist.      {d.data:.2f}", 2), 2)
        self.selector.button.when_activated = lambda: self.display.lcd_backlight(int(not self.display.get_backlight()))
    
        return response_result

    def send_data(self, response: GetACCparams.Response, speed, timegap):
        response.speed = int(speed)
        response.level = float(timegap)
        self.get_logger().info(f"Sent to controller: {speed}, {timegap}")
        return response

def main():
    rclpy.init()
    ui = UI_Prog()

if __name__ == "__main__":
    main()
