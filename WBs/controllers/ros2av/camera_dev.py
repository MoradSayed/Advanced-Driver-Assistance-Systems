from rclpy.node import Node
from controller import Camera
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2, numpy as np

class CameraDev(Camera):
    _model = "camera"
    
    def __init__(self, node:Node, TIME_STEP):
        super().__init__("camera")
        self.enable(TIME_STEP)
        self.lwidth = self.getWidth()
        self.lheight = self.getHeight()
        self.lfov = self.getFov()

        self.node = node
        self.img_pub = node.create_publisher(Image, "/av_camera", 10)
        self.bridge = CvBridge()

    def process_data(self):
        img_bytes = self.getImage()  # This should return (width * height * 4) bytes

        # Ensure the received image is of the expected size
        expected_size = self.lwidth * self.lheight * 4  # 4 channels (BGRA)
        if len(img_bytes) != expected_size:
            self.node.get_logger().error(f"Invalid image size! Expected {expected_size}, got {len(img_bytes)}")
        else:
            # Convert bytes to numpy array and reshape to (H, W, 4)
            cv_image = np.frombuffer(img_bytes, dtype=np.uint8).reshape((self.lheight, self.lwidth, 4))
            cv_image = cv_image[:, :, :3]  # Keep only BGR channels

            self.img_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))