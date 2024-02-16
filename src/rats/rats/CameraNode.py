from libs.utilities import get_camera_params_file
from libs.Camera import Camera

from threading import Thread

import rclpy
from rclpy.node import Node
from rats_interfaces.msg import CamFrame

import signal
# this stuff makes it ctrl-c-able
def signal_handler(sig, frame):
    exit(0)
signal.signal(signal.SIGINT, signal_handler)


class CameraNode(Node):
    """
    ROS2 node for publishing camera images
    
    Attributes:
        _camera (Camera): Camera object
        _pub (rclpy.Publisher): ROS2 publisher
        _frame_id (str): Frame ID for the camera
        _timer (rclpy.Timer): Timer for publishing frames
    """
    def __init__(self, name, topic, camera):
        super().__init__(name)
        self._camera = camera
        self._pub = self.create_publisher(CamFrame, topic, 1)
        self._last_time = self.get_clock().now()
        camera.set_frame_callback(self._publish_frame)
        camera.start()

    def _publish_frame(self, camera: Camera, frame) -> None:
        """
        Publishes a frame from the camera
        """
        if frame is not None:
            msg = CamFrame()
            msg.image.header.stamp = self.get_clock().now().to_msg()
            msg.image.height = frame.shape[0]
            msg.image.width = frame.shape[1]
            msg.image.encoding = 'bgr8'
            msg.image.is_bigendian = False
            msg.image.step = frame.shape[2]
            msg.image.data = frame.tobytes()
            msg.calibration.intrinsic = camera.get_intrinsic().flatten()
            msg.calibration.distortion = camera.get_distortion().flatten()
            self._pub.publish(msg)
            self.get_logger().debug(f'Published frame from {camera._name}')
            self.get_logger().debug(f'Frame time: {(self.get_clock().now() - self._last_time)}')
            self._last_time = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    camera_params = get_camera_params_file('src/rats/config/cameralist.json')
    cameras = [Camera(**camera) for camera in camera_params]
    camera_nodes = [CameraNode(f'CameraNode_{camera._name}', 'frame', camera) for camera in cameras]
    executor = rclpy.executors.MultiThreadedExecutor()
    for node in camera_nodes:
        executor.add_node(node)
    rate = camera_nodes[0].create_rate(2)
    executor_thread = Thread(target=executor.spin)
    executor_thread.start()
    while rclpy.ok():
        rate.sleep()
    executor.shutdown()

if __name__ == '__main__':
    main()