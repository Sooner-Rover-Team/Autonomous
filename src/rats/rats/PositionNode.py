import numpy as np
import time

import rclpy
from rclpy.node import Node
from libs.utilities import calc_bearing, get_coordinates, calc_average_bearing, distance_to, meters_to_degrees
from rats_interfaces.msg import GPSReport, TrackerReport, WheelSpeed
from math import exp, sin, cos

import signal
# this stuff makes it ctrl-c-able
def signal_handler(sig, frame):
    exit(0)
signal.signal(signal.SIGINT, signal_handler)

class ObjectFilter:
    def __init__(self) -> None:
        self._num_observations = 0
        self._object_lat = 0
        self._object_lon = 0
        self._last_observed_time = 0
        self._weight_function = lambda x: 1/(x+1)

    def add_observation(self, lat, lon):
        self._last_observed_time = time.perf_counter()
        # if self._num_observations == 0:
        #     self._object_lat = lat
        #     self._object_lon = lon
        # else:
        weight = self._weight_function(self._num_observations)
        self._object_lat = (1-weight) * self._object_lat + weight * lat
        self._object_lon = (1-weight) * self._object_lon + weight * lon

    def get_position(self):
        return self._object_lat, self._object_lon
    
    def get_time(self):
        return time.perf_counter() - self._last_observed_time

class PositionFilter:
    def __init__(self) -> None:
        self._bearing_est = 0.0
        self._latitude_est = 0.0
        self._longitude_est = 0.0
        self._velocity_est = 0.0
        self._left_speed = 0.0
        self._right_speed = 0.0
        self._last_update_time = 0.0
        time_constant = 0.5
        self._weight_function = lambda x: exp(-x/time_constant)
        self._set_avg_bearing_list(0, 5)


    def _set_avg_bearing_list(self, value, length):
        """
        Sets the average bearing list to a list of a given length
        and fills it with the given value

        Args:
            value (float): Value to fill the list with
            length (int): Length of list
        """
        self._average_bearing_length = length
        self._average_bearing = [value] * length
        self._average_bearing_index = 0

    def _calc_avg_bearing(self, lat1, lon1, lat2, lon2, is_reversed=False):
        """
        Calculates bearing between two points while updating and
        using running average of bearings recorded in a circular queue
        to smooth out noise

        Args:
            lat1 (float): Latitude of first point
            lon1 (float): Longitude of first point
            lat2 (float): Latitude of second point
            lon2 (float): Longitude of second point

        Returns:
            float: Bearing between two points
        """
        bearing = calc_bearing(lat1, lon1, lat2, lon2)

        if is_reversed:
            bearing += 180
            if bearing > 360:
                bearing -= 360

        self._average_bearing[self._average_bearing_index] = bearing
        self._average_bearing_index = (
            self._average_bearing_index + 1) % self._average_bearing_length

        # if the average magnitude is greater than 90, then the average
        # is probably on the other side of the circle, so we need to
        # add 360 to all the negative bearings in the queue
        return calc_average_bearing(self._average_bearing)



    def gps_update(self, latitude, longitude):
        time_delta = (time.perf_counter() - self._last_update_time)
        weight = self._weight_function(time_delta)
        distance = distance_to(self._latitude_est, self._longitude_est, latitude, longitude)
        wheel_factor = (self._left_speed + self._right_speed) / 2
        self._latitude_est = weight * self._latitude_est + (1-weight) * latitude * wheel_factor
        self._longitude_est = weight * self._longitude_est + (1-weight) * longitude * wheel_factor
        self._velocity_est = (1-weight) * self._velocity_est + weight * (distance / time_delta) * wheel_factor
        if (abs(wheel_factor) > 0.2):
            self._bearing_est = self._calc_avg_bearing(self._latitude_est, self._longitude_est, latitude, longitude)
        self._last_update_time = time.perf_counter()

    def get_bearing(self):
        return self._bearing_est

    def predict_location(self):
        time_delta = (time.perf_counter() - self._last_update_time)
        velocity_deg = meters_to_degrees(self._velocity_est)
        pred_lat = self._latitude_est + velocity_deg * time_delta * sin(self._bearing_est)
        pred_lon = self._longitude_est + velocity_deg * time_delta * cos(self._bearing_est)
        return pred_lat, pred_lon

    def wheel_update(self, left, right):
        self._left_speed = left
        self._right_speed = right
    

class PositionNode(Node):
    def __init__(self):
        super().__init__('position_node')
        self._gps_subscriber = self.create_subscription(GPSReport, 'gps', self.gps_listener_callback, 1)
        self._tracker_subscriber = self.create_subscription(TrackerReport, 'tracker_report', self.tracker_listener_callback, 1)
        self._wheel_speed_subscriber = self.create_subscription(WheelSpeed, 'wheel_speeds', self.wheel_speed_listener_callback, 1)
        self._rover_pos_pub = self.create_publisher(GPSReport, 'rover_position', 1)
        self._tracked_pos_pub = self.create_publisher(GPSReport, 'tracked_position', 1)
        self._position_filter = PositionFilter()
        self._object_dict = dict()

    def wheel_speed_listener_callback(self, msg: WheelSpeed):
        self._position_filter.wheel_update(msg.left, msg.right)

    def gps_listener_callback(self, msg: GPSReport):
        longitude = msg.position.longitude
        latitude = msg.position.latitude

        self._position_filter.gps_update(latitude, longitude)
        pred_lat, pred_lon = self._position_filter.predict_location()
        pred_bearing = self._position_filter.get_bearing()

        self.get_logger().info("\ngps lat: %3.6f lon %3.6f\nprd lat: %3.6f lon %3.6f" % (latitude, longitude, pred_lat, pred_lon) )

        pos_msg = GPSReport()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.position.latitude = pred_lat
        pos_msg.position.longitude = pred_lon
        pos_msg.position.has_bearing = True
        pos_msg.position.bearing = pred_bearing
        self._rover_pos_pub.publish(pos_msg)

    def tracker_listener_callback(self, msg: TrackerReport):
        self.get_logger().info('Received tracker report')
        rel_bearing = msg.rel_bearing
        rel_distance = msg.rel_distance
        is_aruco = msg.is_aruco
        obj_id = msg.aruco_id

        if obj_id not in self._object_dict.keys():
            self._object_dict[obj_id] = ObjectFilter()

        if is_aruco:
            self.get_logger().info('ARUCO')
        else:
            self.get_logger().info('NOT ARUCO')

        self.get_logger().info('Bearing: %f' % rel_bearing)
        self.get_logger().info('Distance: %f' % rel_distance)

        if rel_distance > 0:
            pred_lat, pred_lon = self._position_filter.predict_location()
            pred_bearing = self._position_filter.get_bearing()

            self.get_logger().info("pred lat: %3.6f, pred lon: %3.6f" % (pred_lat, pred_lon))
            self.get_logger().info("est_vel: %f" % (self._position_filter._velocity_est))

            latitude, longitude = get_coordinates(pred_lat, pred_lon, rel_distance, pred_bearing + rel_bearing)

            obj = self._object_dict[obj_id]
            obj.add_observation(latitude, longitude)
            obj_lat, obj_lon = obj.get_position()


            tracker_pos = GPSReport()
            tracker_pos.header.stamp = self.get_clock().now().to_msg()
            tracker_pos.position.latitude = obj_lat
            tracker_pos.position.longitude = obj_lon
            tracker_pos.position.has_bearing = False
            tracker_pos.id = obj_id
            self._tracked_pos_pub.publish(tracker_pos)

            

def main(args=None):
    rclpy.init(args=args)

    position_node = PositionNode()

    rclpy.spin(position_node)

    position_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()