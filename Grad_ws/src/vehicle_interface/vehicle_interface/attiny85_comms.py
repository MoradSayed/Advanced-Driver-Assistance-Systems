"""
RPi5 <-> ATtiny85 I2C Example
==============================
Reads vehicle speed and sends stepper angle commands.

Requirements:
    sudo pip install smbus2
    # or: sudo apt install python3-smbus

I2C Protocol (ATtiny85 slave address 0x10):
    READ  2 bytes -> uint16_t speed in 0.1 km/h  (MSB first)
    WRITE 2 bytes -> int16_t  angle in degrees    (MSB first, two's complement)
"""

import smbus2, time, json, os
from datetime import datetime

#-CONFIG----
I2C_BUS     = 1       # RPi5 default I2C bus
SLAVE_ADDR  = 0x10    # must match I2C_ADDRESS in the ATtiny85 firmware
#-----------

bus = smbus2.SMBus(I2C_BUS)

def read_speed(bus) -> float:
    """
    Reads 2 bytes from the ATtiny85 and returns speed in km/h.
    """
    data = bus.read_i2c_block_data(SLAVE_ADDR, 0, 2)   # register 0 is ignored by slave, just reads 2 bytes
    raw  = (data[0] << 8) | data[1]                     # reconstruct uint16 (MSB first)
    speed_kmh = raw / 10.0                              # convert from 0.1 km/h units
    return speed_kmh

def set_angle(bus, degrees: int):
    """
    Sends an ABSOLUTE target angle (degrees) to the ATtiny85 stepper controller.
    The ATtiny calculates the delta from its current position internally.
    Range: -32768 to +32767 degrees (practically ±180° for steering)
    Positive = CW, Negative = CCW from 0°.
    """
    original_degrees = degrees
    degrees = max(-32768, min(32767, int(degrees)))

    if degrees < 0:
        degrees += 65536
    msb = (degrees >> 8) & 0xFF
    lsb =  degrees       & 0xFF

    # print(f"Setting absolute position: {original_degrees:+d}° → bytes: [0x{msb:02X}, 0x{lsb:02X}]")
    bus.write_i2c_block_data(SLAVE_ADDR, 0, [msb, lsb])

if __name__ == "__main__":
    # --- Example 1: Read speed in a loop every 60 ms ---
    print("Reading speed (Ctrl+C to stop)...\n")
    try:
        combined_data = []
        while 1:
            speed = read_speed(bus)
            data = {
               "timestamp": datetime.now(),
               "value": speed
            }
            combined_data.append(data)
            time.sleep(0.06)                        # 60 ms interval, matches ATtiny85 reporting cadence
    except KeyboardInterrupt:
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "wss_data.ignore", f"wss_data_{datetime.now()}.json"), "w") as json_file:
            json.dump(combined_data, json_file, default=str, indent=4)  

    print()

    # --- Example 2: Send absolute position commands ---
    # positions = [0, 90, 45, 180, 0, -90, 0]  # Absolute angles
    
    # for pos in positions:
    #     print(f"\n  Moving to absolute position: {pos}°")
    #     set_angle(bus, pos)
    #     time.sleep(0.06)  # Safe margin for most moves
    
    # print("\n  Now try rapid updates (stepper will smoothly track each new target):")
    # for angle in range(0, 181, 15):  # Sweep from 0° to 180° in 15° increments
    #     set_angle(bus, angle)
    #     time.sleep(0.06)  # Send new command every 60ms - stepper adapts mid-move!

    bus.close()
