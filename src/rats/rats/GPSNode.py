import time
from math import cos, sin
from threading import Thread
from time import sleep

import rclpy
from rclpy.node import Node

from rats_interfaces.msg import GPSReport
import numpy as np
from numpy.random import PCG64, Generator

from libs.utilities import (calc_average_bearing, calc_bearing, distance_to,
                            meters_to_degrees)

import signal
# this stuff makes it ctrl-c-able
def signal_handler(sig, frame):
    exit(0)
signal.signal(signal.SIGINT, signal_handler)


# the gps python bindings wont compile for me so i made a fake one
# that does absolutely nothing and returns 0 for everything
# so that MockedGPSInterface can use the same interface
class fake_gps:
    def __init__(self):
        self._latitude = 0
        self._longitude = 0
        self._height = 0
        self._time = 0
        self._error = 0

    @staticmethod
    def get_latitude():
        return 0

    @staticmethod
    def get_longitude():
        return 0

    @staticmethod
    def get_height():
        return 0

    @staticmethod
    def get_time():
        return 0

    @staticmethod
    def get_error():
        return 0

    @staticmethod
    def gps_init(ip, port):
        return

    @staticmethod
    def gps_finish():
        return


# it will use fake gps if the gps module is not found
# or you can force it to use fake gps by setting this to True
USE_FAKE_GPS = False

if not USE_FAKE_GPS:
    try:
        # try to import gps module
        from gps import gps
    except ImportError:
        # on failure, use mocked gps
        print("GPSInterface: gps module not found, using mock gps")
        gps = fake_gps
        USE_FAKE_GPS = True
else:
    gps = fake_gps

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self._publisher = self.create_publisher(GPSReport, 'gps', 1)
        self._timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

        # TODO: LOAD FROM CONFIG RIGHT HERE

        gps.gps_init('', 12345)

    def timer_callback(self):
        msg = GPSReport()
        # msg.latitude = float(gps.get_latitude())
        # msg.longitude = float(gps.get_longitude())
        lon = self.i * 0.0001
        lat = self.i * 0.0001
        msg.position.latitude = lat
        msg.position.longitude = lon
        self._publisher.publish(msg)
        # self.get_logger().info('GPS Message: "%s"' % msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    gps_node = GPSNode()

    rclpy.spin(gps_node)

    gps_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()