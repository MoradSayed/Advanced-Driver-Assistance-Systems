import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
#import board
#import busio
#from adafruit_mcp4725 import MCP4725
import pyfirmata

print("just got here")
MAX_SPEED = 60.0       # max speed 
MAX_VOLTAGE = 4.0      # max voltage
#DAC_RESOLUTION = 4095  # 12-bit resolution of MCP4725 DAC

board = pyfirmata.Arduino('/dev/ttyACM0')

it = pyfirmata.util.Iterator(board)
it.start()

pin_pwm = board.get_pin('d:3:p')
pin_switch = board.get_pin('d:13:i')
# analog_input = board.get_pin('a:5:i')
cd4053_pin_b = board.get_pin('d:7:o')


class speed_voltage(Node):
    
    def __init__(self):
        super().__init__('speed_voltage_node')
        # Initialize I²C bus and DAC
        #i2c = busio.I2C(board.SCL, board.SDA)
        #self.dac = MCP4725(i2c)

        self.callback_method = self.speed_callback # self.direct_callback
        self.create_subscription(Float64,'/cmd_vel',lambda msg: self.callback_method(msg), 10) 

        self.switch_state = False
        self.switch_timer = self.create_timer(0.01, self.state_checker)
        self.direct_timer = None
        print(f"x -> Normal")
        
        #self.get_logger().info("speed_voltage_node started.")
        
    def state_checker(self):
        state = pin_switch.read()
        if state != self.switch_state:
            if state:
                self.callback_method = lambda _: None
                # self.direct_timer = self.create_timer(0.01, self.direct_callback)
                self.direct_callback()
                print(f"x -> direct")
            else:
                #? Op1: resume after letting go of the accelrator
                # self.direct_timer.cancel()
                # self.callback_method = self.speed_callback
                #? Op2: stay disconnected after letting go of the accelrator
                print(f"x -> stay_disconnected")
                pass
            # print(f"switch = {state}")
        
        self.switch_state = state

    def direct_callback(self, msg=None):
        # duty_cycle = analog_input.read()
        # pin_pwm.write(duty_cycle)
        # print(f"\r{duty_cycle}")
        cd4053_pin_b.write(0)


    def speed_callback(self, msg):
        print("called")
        cd4053_pin_b.write(1)
        speed = msg.data      
        
        voltage = min(self.speed_to_voltage(speed) + 1, MAX_VOLTAGE)              # Convert speed to voltage 
        
        duty_cycle = voltage / MAX_VOLTAGE
        pin_pwm.write(duty_cycle)
        print(f"\r{duty_cycle}")
        
        
        #dac_value = self.voltage_to_dac_value(voltage)      # Convert voltage to DAC value
        
        #self.dac.raw_value = dac_value                      # Send value to MCP4725 DAC
        #self.get_logger().info(f"""Speed: {speed:.2f} km/h → 
        #                       Volt: {voltage:.2f} V → 
        #                       DAC Value: {dac_value}""")

    def speed_to_voltage(self, speed_kmh):
        if speed_kmh < 0:
            speed_kmh = 0
        if speed_kmh > MAX_SPEED:
            speed_kmh = MAX_SPEED
        
        return ((max(0.0, min(MAX_SPEED, speed_kmh))) / MAX_SPEED) * MAX_VOLTAGE

    #def voltage_to_dac_value(self, voltage):
        #return int((DAC_RESOLUTION / MAX_VOLTAGE) * voltage)

    def destroy_node(self):
        # Ensure PWM goes low and the serial connection is closed
        try:
            if pin_pwm:
                pin_pwm.write(0.0)
        except Exception:
            pass
        try:
            if board:
                board.exit()
        except Exception:
            pass
        super().destroy_node()



# def main():
print("ON")
rclpy.init()
node = speed_voltage()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()

# if __name__ == '__main__':
    # main()
