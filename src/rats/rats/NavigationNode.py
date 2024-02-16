from libs.utilities import abs_clamp, calc_bearing, wrap_bearing, distance_to
from time import sleep, perf_counter_ns
from threading import Thread

# os.chdir(os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.time import Time
from rats_interfaces.msg import GPSReport, WheelSpeed, GPSPoint, GPSPath
from rats_interfaces.action import NavigationAction

import signal
# this stuff makes it ctrl-c-able


def signal_handler(sig, frame):
    exit(0)


signal.signal(signal.SIGINT, signal_handler)


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self._position_subscriber = self.create_subscription(
            GPSReport, 'rover_position', self.rover_pos_callback, 1)
        self._tracked_object_subscriber = self.create_subscription(
            GPSReport, 'tracked_position', self.tracked_object_callback, 1)
        self._action_server = ActionServer(
            self, NavigationAction, 'navigate', self.navigate_to_location_callback)
        self._wheel_speed_publisher = self.create_publisher(
            WheelSpeed, 'wheel_speeds', 1)
        self._path_publisher = self.create_publisher(
            GPSPath, 'received_path', 1)

        # some default values for config stuff
        self._STOPPING_DISTANCE = 1.0  # meters
        self._WAIT_PERIOD = .5
        self._MAX_SPEED = 0.9
        self._MIN_SPEED = 0.1
        self._REVERSE_WHEEL_MOD = 0.7
        self._MAX_BEARING_ERROR = 200
        self._K_P = 0.0165
        self._K_I = 0.002

        # the speed the rover will drive at initially
        self._initial_speed = .5

        # the speed the rover will try and maintain when driving straight
        self._straight_speed = .8

        # variable to remember the total bearing error
        self._accumulated_error = 0.0

        self._last_update_time = self.get_clock().now()

        self._locations = []
        self._location_index = 0

        self._tracked_objects = {}

        # set this true if the rover is looking for a target vs just driving to a location
        self._looking_for_target = False

        # this will be set true if the rover finds a target
        self._found_target = False

        self._target_object = None

        self._running = False

    def tracked_object_callback(self, msg: GPSReport):
        obj_id = msg.id
        obj_lat = msg.position.latitude
        obj_lon = msg.position.longitude

        # TODO: ADD TIME OF UPDATE SO IT CAN STALE OUT
        if obj_id == self._target_object:
            self._found_target = True

        self._tracked_objects[msg.id] = (obj_lat, obj_lon)

    def navigate_to_location_callback(self, goal_handle):
        self.get_logger().info('Received request to navigate to location')
        self._running = True
        gps_path = goal_handle.request.path
        for g in gps_path:
            self._locations.append((g.latitude, g.longitude))
        self._looking_for_target = goal_handle.request.search_for_object
        self._found_target = False
        self._location_index = 0

        # broadcast the path to the rest of the system
        # mainly for the viewer
        path_msg = GPSPath()
        path_msg.num_points = len(gps_path)
        for point in gps_path:
            path_msg.path.append(point)
        self.get_logger().info('Broadcasting path: %s' % path_msg)
        self._path_publisher.publish(path_msg)

        # TODO: Change this to make it more general
        self._target_object = goal_handle.request.aruco_id

        self._accumulated_error = 0

        # TODO: Some sort of initial maneuver
        # self._wheels.set_wheel_speeds(self._initial_speed, self._initial_speed)

        while self._running:
            feedback_msg = NavigationAction.Feedback()
            if self._looking_for_target and self._found_target:
                found_object = True
            else:
                found_object = False

            feedback_msg.percent_traveled = self._location_index / \
                len(self._locations)
            feedback_msg.found_object = found_object

            # this is cursed
            rclpy.spin_once(self, timeout_sec=.5)

        result = NavigationAction.Result()
        result.destination_reached = found_object
        goal_handle.succeed()

        return NavigationAction.Result()

    def rover_pos_callback(self, msg: GPSReport):
        self.get_logger().info('Received rover position')
        rover_lon = msg.position.longitude
        rover_lat = msg.position.latitude
        rover_bearing = msg.position.bearing
        msg_time = Time.from_msg(msg.header.stamp)
        dt = (msg_time - self._last_update_time).nanoseconds / 1e9
        self._last_update_time = msg_time

        if self._running:
            self.drive_to_target(rover_lon, rover_lat, rover_bearing, dt)

    def drive_to_target(self, rover_lon, rover_lat, rover_bearing, dt):
        if self._looking_for_target and self._found_target:
            target_lat, target_lon = self._tracked_objects[self._target_object]
        else:
            target_lat, target_lon = self._locations[self._location_index]

        abs_bearing = calc_bearing(
            rover_lat, rover_lon, target_lat, target_lon)
        bearing_to = wrap_bearing(abs_bearing - rover_bearing)

        self.get_logger().info('Bearing to: %f' % bearing_to)

        distance = distance_to(rover_lat, rover_lon, target_lat, target_lon)

        self.get_logger().info('Distance to: %f' % distance)

        if distance < self._STOPPING_DISTANCE:
            self._location_index += 1
            self.get_logger().info('Made it to a waypoint')
            self._accumulated_error = 0
            if self._location_index == len(self._locations):
                self.get_logger().info('Made it to the final waypoint')
                self._running = False
                stop_msg = WheelSpeed()
                stop_msg.header.stamp = self.get_clock().now().to_msg()
                stop_msg.left = 0.0
                stop_msg.right = 0.0
                self._wheel_speed_publisher.publish(stop_msg)
        else:
            left_speed, right_speed = self.calculate_speeds(
                self._straight_speed, bearing_to, dt, self._K_P, self._K_I)

            self.get_logger().info('Left speed: %f' % left_speed)
            self.get_logger().info('Right speed: %f' % right_speed)

            wheel_speed_msg = WheelSpeed()
            wheel_speed_msg.header.stamp = self.get_clock().now().to_msg()
            wheel_speed_msg.left = left_speed
            wheel_speed_msg.right = right_speed
            self._wheel_speed_publisher.publish(wheel_speed_msg)

    def calculate_speeds(self, speed, bearing_error, time, kp, ki):
        """
        Gets the adjusted speed values for the wheels based off of the current
        bearing error as well as the accumulated bearing error. (A PID loop using
        the P and I constants).

        Args:
            speed: The speed the rover is going (float, in range 0-1)
            bearing_error: The error in the bearing (degrees)
            time: The time between calls to this function (seconds)
            kp: The proportional constant
            ki: The integral constant   

        Returns:
            The adjusted speed values for the wheels
        """
        left_speed = 0
        right_speed = 0

        # Updates the error accumulation
        self._accumulated_error += bearing_error * time
        self._accumulated_error = abs_clamp(
            self._accumulated_error, -self._MAX_BEARING_ERROR, self._MAX_BEARING_ERROR)

        # Gets the adjusted speed values
        left_speed = speed + (bearing_error * kp +
                              self._accumulated_error * ki)
        right_speed = speed - (bearing_error * kp +
                               self._accumulated_error * ki)

        # Makes sure the speeds are within the min and max
        left_speed = abs_clamp(left_speed, self._MIN_SPEED, self._MAX_SPEED)
        right_speed = abs_clamp(right_speed, self._MIN_SPEED, self._MAX_SPEED)

        # prevents complete pivots
        # the side that is going backwards is slowed down
        if abs(left_speed - right_speed) > self._MAX_SPEED * 2 - .2:
            if left_speed > 0:
                right_speed += abs(right_speed) * (1-self._REVERSE_WHEEL_MOD)
            else:
                left_speed += abs(left_speed) * (1-self._REVERSE_WHEEL_MOD)

        return (left_speed, right_speed)


def main(args=None):
    rclpy.init(args=args)

    navigation_node = NavigationNode()

    rclpy.spin(navigation_node)

    navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
