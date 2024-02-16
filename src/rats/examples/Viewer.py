import numpy as np
import pygame
from pygame.locals import *

from math import pi, sin, cos

from libs.utilities import degrees_to_meters

import rclpy
from rclpy import qos
from rclpy.node import Node
from rats_interfaces.msg import GPSReport, GPSPath
from rats_interfaces.srv import SimArucoPos

import signal
# this stuff makes it ctrl-c-able
global_running = True

def signal_handler(sig, frame):
    global global_running 
    global_running = False
    # exit(0)


signal.signal(signal.SIGINT, signal_handler)

VIEWER_WIDTH = 1280
VIEWER_HEIGHT = 720


class Viewer:
    def __init__(self):
        pygame.init()
        self._width = VIEWER_WIDTH
        self._height = VIEWER_HEIGHT
        self._screen = pygame.display.set_mode(
            (self._width, self._height), RESIZABLE)
        self.clock = pygame.time.Clock()
        lat_m = 0
        lon_m = 0
        self._camera_pos = np.array([lon_m, lat_m], dtype=np.float32)
        self._scale = 50 / self._width  # meters per pixel
        self._rover_size = 1  # meter
        self._orig_rover_rect = pygame.surface.Surface(
            (int(100*2/3), 100), pygame.SRCALPHA)
        self._orig_rover_rect.fill((0, 0, 0))
        self._orig_rover_rect.set_alpha(100)
        self._mouse_pressed = False

        self.GPS_COLOR = (255, 0, 0)
        self.ESTIMATED_COLOR = (0, 120, 255)
        self.TARGET_COLOR = (0, 255, 255)
        self.MARKER_COLOR = (0, 255, 0)
        self.TRACKED_COLOR = (200, 180, 0)

        self._real_positions = []
        self._real_bearings = [0]
        self._real_gps_targets = []
        self._real_marker_positions = {}
        self._gps_measurements = []
        self._positon_estimates = []
        self._bearing_estimate = 0
        self._tracked_objects = {}

    def coords_in_screen(self, x, y):
        return 0 <= x < self._width and 0 <= y < self._height

    def world_to_screen(self, x, y):
        x = (x - self._camera_pos[0]) / self._scale
        y = (y - self._camera_pos[1]) / self._scale
        return int(x + self._width/2), int(self._height/2 - y)

    def draw_gps_measurements(self):
        for lat, lon in self._gps_measurements:
            lat = degrees_to_meters(lat)
            lon = degrees_to_meters(lon)
            lat_x, lat_y = self.world_to_screen(lon, lat)
            if self.coords_in_screen(lat_x, lat_y):
                pygame.draw.circle(
                    self._screen, self.GPS_COLOR, (lat_x, lat_y), 5)

    def draw_position_estimates(self):
        for lat, lon in self._positon_estimates:
            lat = degrees_to_meters(lat)
            lon = degrees_to_meters(lon)
            lat_x, lat_y = self.world_to_screen(lon, lat)
            if self.coords_in_screen(lat_x, lat_y):
                pygame.draw.circle(
                    self._screen, self.ESTIMATED_COLOR, (lat_x, lat_y), 5)

    def draw_gps_targets(self):
        # draw a blue circle at each target position
        for lat, lon in self._real_gps_targets:
            lat = degrees_to_meters(lat)
            lon = degrees_to_meters(lon)
            target_x, target_y = self.world_to_screen(lon, lat)
            if self.coords_in_screen(target_x, target_y):
                pygame.draw.circle(self._screen, self.TARGET_COLOR,
                                   (target_x, target_y), 10)

    def draw_marker_objects(self):
        for id, pos in self._real_marker_positions.items():
            lat_m = degrees_to_meters(pos[0])
            lon_m = degrees_to_meters(pos[1])
            x, y = self.world_to_screen(lon_m, lat_m)
            if self.coords_in_screen(x, y):
                pygame.draw.circle(self._screen, self.MARKER_COLOR, (x, y), 10)

    def draw_tracked_objects(self):
        for id, obj in self._tracked_objects.items():
            lat, lon = obj
            lat_m = degrees_to_meters(lat)
            lon_m = degrees_to_meters(lon)
            x, y = self.world_to_screen(lon_m, lat_m)
            if self.coords_in_screen(x, y):
                pygame.draw.circle(
                    self._screen, self.TRACKED_COLOR, (x, y), 10)

    def draw_rover(self):
        # get rover coordinates
        if len(self._real_positions) == 0:
            return
        lat, lon = self._real_positions[-1]
        rover_x, rover_y = self.world_to_screen(
            degrees_to_meters(lon), degrees_to_meters(lat))

        # get rover bearing
        real_bearing_deg = self._real_bearings[-1]
        real_bearing_rad = np.deg2rad(real_bearing_deg)

        # draw rover
        front = pygame.surface.Surface((int(100*2/3), 100//3), pygame.SRCALPHA)
        # TODO: Add a listener for the lights
        # if rover._lights.current_color == 'g':
        #     front.fill((0, 255, 0))
        # elif rover._lights.current_color == 'o':
        #     front.fill((255, 165, 0))
        # elif rover._lights.current_color == 'r':
        front.fill((255, 0, 0))
        self._orig_rover_rect.blit(front, (0, 0))
        rover_rect = pygame.transform.rotate(
            self._orig_rover_rect, -real_bearing_deg)
        rover_rect = pygame.transform.scale(rover_rect, (int(
            self._rover_size/self._scale), int(self._rover_size/self._scale)))
        bounds = rover_rect.get_bounding_rect()

        rover_rect.set_alpha(100)
        self._screen.blit(rover_rect, (rover_x-bounds.width //
                                       2, rover_y-bounds.height//2))

        calc_bearing_deg = self._bearing_estimate
        calc_bearing_rad = calc_bearing_deg * pi / 180

        # draw rover bearing
        bearing_x = rover_x + sin(calc_bearing_rad) * 50
        bearing_y = rover_y - cos(calc_bearing_rad) * 50
        pygame.draw.line(self._screen, (0, 0, 0),
                         (rover_x, rover_y), (bearing_x, bearing_y), 3)

        # draw dots represnting camera positions
        # for camera in rover._object_tracker.cameras:
        #     camera_offset_side_deg = meters_to_degrees(camera._position[0])
        #     camera_offset_front_deg = meters_to_degrees(camera._position[2])
        #     camera_lon_deg = lon + \
        #         cos(real_bearing_rad) * camera_offset_side_deg + \
        #         -sin(real_bearing_rad) * camera_offset_front_deg
        #     camera_lat_deg = lat + \
        #         -sin(real_bearing_rad) * camera_offset_side_deg + \
        #         -cos(real_bearing_rad) * camera_offset_front_deg

        #     camera_x, camera_y = self.world_to_screen(
        #         degrees_to_meters(camera_lon_deg), degrees_to_meters(camera_lat_deg))
        #     if self.coords_in_screen(camera_x, camera_y):
        #         pygame.draw.circle(self._screen, (0, 0, 0),
        #                            (camera_x, camera_y), 5)

    def handle_key_press(self, key):
        if key == K_ESCAPE:
            self._running = False
        elif key == K_UP:
            self._camera_pos[1] += 50 * self._scale
        elif key == K_DOWN:
            self._camera_pos[1] -= 50 * self._scale
        elif key == K_LEFT:
            self._camera_pos[0] -= 50 * self._scale
        elif key == K_RIGHT:
            self._camera_pos[0] += 50 * self._scale
        elif key == K_EQUALS:
            self._scale /= 1.1
        elif key == K_MINUS:
            self._scale *= 1.1
        elif key == K_r:
            rover_lat = degrees_to_meters(self._real_positions[-1][0])
            rover_lon = degrees_to_meters(self._real_positions[-1][1])
            self._camera_pos = np.array(
                [rover_lon, rover_lat], dtype=np.float32)
            self._scale = 50 / self._width

    def main_loop(self):
        for event in pygame.event.get():
            if event.type == QUIT:
                self._running = False
            # check key presses
            elif event.type == KEYDOWN:
                self.handle_key_press(event.key)

            # check mouse presses
            elif event.type == MOUSEBUTTONDOWN:
                self._mouse_pressed = True
            elif event.type == MOUSEBUTTONUP:
                self._mouse_pressed = False

            elif event.type == MOUSEWHEEL:
                self._scale *= 1 + event.y * -.1
                self._scale = max(self._scale, 1e-6)

        self._width = self._screen.get_width()
        self._height = self._screen.get_height()
        rel_x, rel_y = pygame.mouse.get_rel()
        if self._mouse_pressed:
            self._camera_pos[0] -= rel_x * self._scale
            self._camera_pos[1] += rel_y * self._scale

        self._screen.fill((255, 255, 255))

        self.draw_gps_targets()
        self.draw_rover()
        self.draw_gps_measurements()
        self.draw_position_estimates()
        self.draw_marker_objects()
        self.draw_tracked_objects()

        pygame.display.flip()


class ViewerNode(Node):
    def __init__(self):
        super().__init__('viewer')
        self._viewer = Viewer()
        self._gps_measurement_sub = self.create_subscription(
            GPSReport, 'gps', self.gps_measurement_callback, 1)
        self._received_path_sub = self.create_subscription(
            GPSPath, 'received_path', self.received_path_callback, 1)
        self._estimatated_position_sub = self.create_subscription(
            GPSReport, 'rover_position', self.estimated_position_callback, 1)
        self._real_pos_sub = self.create_subscription(
            GPSReport, 'real_position', self.real_pos_callback, 1)
        self._tracked_position_sub = self.create_subscription(
            GPSReport, 'tracked_position', self.tracked_position_callback, 1)
        self._real_aruco_client = self.create_client(SimArucoPos, 'real_aruco')

    def gps_measurement_callback(self, msg: GPSReport):
        lat = msg.position.latitude
        lon = msg.position.longitude
        self._viewer._gps_measurements.append((lat, lon))

    def received_path_callback(self, msg: GPSPath):
        self.get_logger().info('Received gps target %s' % msg)
        for point in msg.points:
            lat = point.latitude
            lon = point.longitude
            self._viewer._real_gps_targets.append((lat, lon))

    def estimated_position_callback(self, msg: GPSReport):
        lat = msg.position.latitude
        lon = msg.position.longitude
        bearing = msg.position.bearing
        self._viewer._positon_estimates.append((lat, lon))
        self._viewer._bearing_estimate = bearing

    def real_pos_callback(self, msg: GPSReport):
        lat = msg.position.latitude
        lon = msg.position.longitude
        bearing = msg.position.bearing
        self._viewer._real_positions.append((lat, lon))
        self._viewer._real_bearings.append(bearing)

    def tracked_position_callback(self, msg: GPSReport):
        lat = msg.position.latitude
        lon = msg.position.longitude
        obj_id = msg.id
        self._viewer._tracked_objects[obj_id] = (lat, lon)

    def get_aruco_positions(self):
        req = SimArucoPos.Request()
        self.get_logger().info('Waiting for aruco marker positions...')
        future = self._real_aruco_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        for lat, lon, id in zip(result.latitudes, result.longitudes, result.ids):
            print(lat, lon, id)
            self._viewer._real_marker_positions[id] = (lat, lon)
        self.get_logger().info('Received aruco marker positions')


def main(args=None):
    rclpy.init(args=args)
    node = ViewerNode()
    node.get_aruco_positions()
    while global_running:
        rclpy.spin_once(node, timeout_sec=0)
        node._viewer.main_loop()