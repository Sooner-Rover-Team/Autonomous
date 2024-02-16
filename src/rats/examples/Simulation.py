import random
from math import cos, pi, sin
from time import perf_counter_ns, time_ns

import numpy as np
from numpy.random import Generator, PCG64

from examples.renderer import Renderer
from libs.utilities import degrees_to_meters, meters_to_degrees, get_camera_params_file, get_camera_matrices

import rclpy
from rclpy import qos
from rclpy.node import Node
from rats_interfaces.msg import CamFrame, GPSReport, WheelSpeed
from rats_interfaces.srv import SimArucoPos

RUNNING = True

import signal
# this stuff makes it ctrl-c-able
def signal_handler(sig, frame):
    global RUNNING
    RUNNING = False
signal.signal(signal.SIGINT, signal_handler)

class FakeRoverPosition():
    def __init__(self):
        self._real_lat = 0  # degrees
        self._real_lon = 0  # degrees
        self._real_bearing = 0  # radians
        # generator for noise
        self._ADD_NOISE = True
        self._noise_dev_m = 0.38  # 38 cm, makes it so 95% of noise is within .75 meters
        self._noise_generator = Generator(PCG64(time_ns()))
        # made this up, should be close to real speed
        self._SPEED = 1.73  # meters per second
        # pulled this out of my ass, basically how much the rover
        # turns when one wheel is at full speed and the other is stopped
        self._single_side_authority = .53  # i guess the unit is rad/m/s (???)

        self._left_wheel_speed = 0
        self._right_wheel_speed = 0

    def set_wheel_speeds(self, left, right):
        self._left_wheel_speed = left
        self._right_wheel_speed = right

    def update_position(self, dt: float) -> None:
        """
        Updates the real position based on current wheel speeds
        """
        # get the current wheel speeds
        left_wheel_speed = self._left_wheel_speed
        right_wheel_speed = self._right_wheel_speed

        # distance traveled is a function of base speed, wheel speeds, and time
        distance_traveled = (
            self._SPEED * (left_wheel_speed + right_wheel_speed) / 2) * dt

        # figure out how much the bearing has changed using this bullshit number
        # (seriously if someone has a better tank control model please replace this)
        bearing_change = (left_wheel_speed - right_wheel_speed) * \
            self._single_side_authority

        # change the real bearing based on thing above and time
        self._real_bearing += bearing_change * dt

        # take the distance traveled and bearing and figure out change in lat and long
        d_lat = cos(self._real_bearing) * meters_to_degrees(distance_traveled)
        d_lon = sin(self._real_bearing) * meters_to_degrees(distance_traveled)
        self._real_lat += d_lat
        self._real_lon += d_lon

    def get_real_position(self) -> tuple:
        return self._real_lat, self._real_lon

    def get_real_bearing(self) -> float:
        return np.rad2deg(self._real_bearing)
    
    def get_position(self) -> tuple:
        if self._ADD_NOISE:
            # if noise is enabled, add noise to the real position
            error_m = self._noise_generator.normal(0, self._noise_dev_m)
            error_deg = meters_to_degrees(error_m)
            error_bearing = self._noise_generator.uniform(0, 2 * np.pi)
            lat = self._real_lat + error_deg * cos(error_bearing)
            lon = self._real_lon + error_deg * sin(error_bearing)
            return lat, lon
        else:
            # otherwise just set the real position
            return self._real_lat, self._real_lon
        
class SimulationNode(Node):
    def __init__(self):
        super().__init__('simulation_node')
        self._frame_publisher = self.create_publisher(CamFrame, 'frame', 1)
        self._gps_publisher = self.create_publisher(GPSReport, 'gps', 1)
        self._real_pos_publisher = self.create_publisher(GPSReport, 'real_position', 1)
        self._wheel_subscriber = self.create_subscription(WheelSpeed, 'wheel_speeds', self.wheel_listener_callback, 1)
        self._real_aruco_service = self.create_service(SimArucoPos, 'real_aruco', self.real_aruco_callback)
        self._timer = self.create_timer(.5, self.gps_timer_callback)
        self._mocked_gps = FakeRoverPosition()
        self._mocked_gps.set_wheel_speeds(0,0)
        self._last_update_time = perf_counter_ns()

        self._aruco_markers = {}

        camera_params = get_camera_params_file('src/rats/examples/cameralist_sim.json')
        self.intrinsics = {}
        self.distortions = {}
        for camera in camera_params:
            name = camera['name']
            device = camera['device']
            width = camera['width']
            height = camera['height']
            v_fov = camera['v_fov']
            intrinsic, distortion = get_camera_matrices(name, width, height, v_fov)
            self.intrinsics[device] = intrinsic
            self.distortions[device] = distortion

        self.renderer = Renderer()
        self.renderer.add_cameras(camera_params)
        self.renderer.set_rover_position(0, 0, 0)
        self.create_aruco_marker(0, -10, 0, degrees=False)
        # self.create_aruco_marker(1, 20, 0, degrees=False)
        # self.create_aruco_marker(2, 30, 0, degrees=False)

    def create_aruco_marker(self, id, lat, lon, degrees=False):
        if degrees:
            lat_deg = lat
            lon_deg = lon
            lat_m = degrees_to_meters(lat_m)
            lon_m = degrees_to_meters(lon_m)
        else:
            lat_deg = meters_to_degrees(lat)
            lon_deg = meters_to_degrees(lon)
            lat_m = lat
            lon_m = lon

        alt = random.random() + .5
        yaw = random.random()*2*pi 

        self.renderer.add_aruco_marker(id, lat_m, lon_m, alt, yaw)

        pos_deg = (lat_deg, lon_deg)
        self._aruco_markers[id] = pos_deg

    def real_aruco_callback(self, request, response):
        self.get_logger().info('Received request')
        ids = []
        longitudes = []
        latitudes = []
        for id, pos in self._aruco_markers.items():
            ids.append(id)
            lat, lon = pos
            latitudes.append(lat)
            longitudes.append(lon)
        response.ids = ids
        response.latitudes = latitudes
        response.longitudes = longitudes
        return response

    def wheel_listener_callback(self, msg):
        self._mocked_gps.set_wheel_speeds(msg.left, msg.right)

    def render_and_publish(self):
        # figure out how long its been since the last update
        dt = (perf_counter_ns() - self._last_update_time) / 1e9
        self._last_update_time = perf_counter_ns()

        # update sim params
        self._mocked_gps.update_position(dt)
        real_bearing_deg = self._mocked_gps.get_real_bearing()
        real_lat_deg, real_lon_deg = self._mocked_gps.get_real_position()
        real_bearing_rad = np.deg2rad(real_bearing_deg)
        real_lat_m = degrees_to_meters(real_lat_deg)
        real_lon_m = degrees_to_meters(real_lon_deg)
        self.renderer.set_rover_bearing(real_bearing_rad)
        self.renderer.set_rover_position(real_lat_m, real_lon_m, 1)

        # render the camera view
        device, frame_data = self.renderer.render_and_process_window()

        width = frame_data.shape[1]
        height = frame_data.shape[0]

        frame_msg = CamFrame()
        # frame_msg.header.stamp = self.get_clock().now().to_msg()
        intrinsic = self.intrinsics[device]
        distortion = self.distortions[device]
        frame_msg.calibration.intrinsic = np.array(intrinsic).flatten()
        frame_msg.calibration.distortion = np.array(distortion).flatten()
        frame_msg.image.width = width
        frame_msg.image.height = height
        frame_msg.image.step = width * 3
        frame_msg.image.data = frame_data.tobytes()
        self._frame_publisher.publish(frame_msg)

        real_pos_msg = GPSReport()
        real_pos_msg.position.latitude = real_lat_deg
        real_pos_msg.position.longitude = real_lon_deg
        real_pos_msg.position.has_bearing = True
        real_pos_msg.position.bearing = real_bearing_deg
        self._real_pos_publisher.publish(real_pos_msg)

    def gps_timer_callback(self):
        msg = GPSReport()
        msg.header.stamp = self.get_clock().now().to_msg()
        lat, lon = self._mocked_gps.get_position()
        msg.position.latitude = lat
        msg.position.longitude = lon
        msg.position.has_bearing = False
        self._gps_publisher.publish(msg)

def main():
    rclpy.init()
    simulation_node = SimulationNode()
    while RUNNING:
        rclpy.spin_once(simulation_node, timeout_sec=0)
        simulation_node.render_and_publish()
    simulation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()