from vehicle_interface.MR76_Radar import MR76_radar as mr76
from vehicle_interface.MR76_Radar.usbcan_ii_libusb_aarch64.USBCAN_Interface import ControlCAN, VCI_USBCAN1, STATUS_OK, VCI_INIT_CONFIG
from rclpy.node import Node
from std_msgs.msg import Float64, Float32
import numpy as np

# Initialize CAN interface
can = ControlCAN()

class RadarDev(mr76.MR76Radar):
    def __init__(self, node: Node):
        self.node = node
        self.lane_width = 3

        self.dist_pub = node.create_publisher(Float32, "/radar_min", 10)
        self.rel_vel_pub = node.create_publisher(Float32, "/radar_rel_vel", 10)
        self.dist_msg = Float32()
        self.rel_vel_msg = Float32()

        if can.open_device(VCI_USBCAN1, 0, 0) != STATUS_OK: #! need better handling
            raise ConnectionRefusedError("Failed to open USBCAN-I")
        
        # Configure CAN: 500 Kbps (MR76 requirement)
        config = VCI_INIT_CONFIG()
        config.AccCode = 0x00000000
        config.AccMask = 0xFFFFFFFF
        config.Filter = 1
        config.Timing0 = 0x00
        config.Timing1 = 0x1C
        config.Mode = 0
        
        can.init_can(VCI_USBCAN1, 0, 0, config)
        can.start_can(VCI_USBCAN1, 0, 0)
        
        # Initialize radar interface
        super().__init__(can, VCI_USBCAN1, 0, 0, sensor_id=0)
        
        # print("MR76 Radar - Basic Detection Example")
        # print("Press Ctrl+C to stop\n")

        self.open_device()
        
    def run(self):
        # Process incoming messages
        self.process_can_messages(timeout_ms=100)

        objects = self.get_objects()
        if len(objects) > 0:
            for obj in objects:
                self.node.get_logger().info(f"len {len(objects)}")
                if obj.object_class.name == "VEHICLE":
                    self.node.get_logger().info(f"got {obj}")
                    # range_m = obj.get_radial_distance()
                    # angle = obj.get_angle_deg()

                    # x = range_m * np.cos(angle)
                    if abs(obj.dist_long) > (self.lane_width/2):
                        continue
                    else:
                        self.rel_vel_msg.data=obj.vrel_long
                        self.rel_vel_pub.publish(self.rel_vel_msg)
                        # distance = range_m * np.sin(angle)
                        self.dist_msg.data = obj.dist_long
                        self.dist_pub.publish(self.dist_msg)
                        return 0

        self.rel_vel_msg.data= 0.0
        self.rel_vel_pub.publish(self.rel_vel_msg)
        self.dist_msg.data = float('inf')
        self.dist_pub.publish(self.dist_msg)

    def open_device(self):
        self.radar_timer = self.node.create_timer(0.01, self.run)
    
    def close_device(self):
        self.node.destroy_timer(self.radar_timer)
        can.close_device(VCI_USBCAN1, 0)
