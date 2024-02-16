import rclpy
from rclpy.node import Node

from rats_interfaces.msg import CamFrame

import cv2
import numpy as np

import signal
# this stuff makes it ctrl-c-able
def signal_handler(sig, frame):
    exit(0)
signal.signal(signal.SIGINT, signal_handler)


class CameraReader(Node):
    def __init__(self):
        super().__init__('camera_reader')
        self.subscriber = self.create_subscription(CamFrame, 'frame', self.listener_callback, 1)
        self.window = cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)

    def listener_callback(self, msg):
        height = msg.image.height
        width = msg.image.width
        step = msg.image.step
        self.get_logger().debug('Received frame')
        self.get_logger().debug('Frame size: %d x %d' % (width, height))
        frame = np.frombuffer(msg.image.data, dtype=np.uint8).reshape(height, width, 3)
        # intrinsic = np.frombuffer(msg.calibration.intrinsic, dtype=np.float32).reshape(3, 3)
        # distortion = np.frombuffer(msg.calibration.distortion, dtype=np.float32).reshape(1, 5)
        # self.get_logger().info('Intrinsic: %s' % intrinsic)
        # self.get_logger().info('Distortion: %s' % distortion)
        cv2.imshow('Camera', frame)
        cv2.waitKey(10)

def main(args=None):
    rclpy.init(args=args)

    camera_reader = CameraReader()

    rclpy.spin(camera_reader)

    camera_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()