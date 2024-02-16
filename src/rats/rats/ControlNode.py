from libs.utilities import meters_to_degrees
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rats_interfaces.msg import GPSPoint, GPSPath
from rats_interfaces.action import NavigationAction

import signal
# this stuff makes it ctrl-c-able
def signal_handler(sig, frame):
    exit(0)
signal.signal(signal.SIGINT, signal_handler)

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self._action_client = ActionClient(self, NavigationAction, 'navigate')

    def send_path(self, path: list[tuple], search_for_object, object_id):
        goal_msg = NavigationAction.Goal()

        debug_msg = GPSPath()
        debug_msg.num_points = len(path)
        for point in path:
            gps_point = GPSPoint()
            gps_point.latitude = point[0]
            gps_point.longitude = point[1]
            goal_msg.path.append(gps_point)
            debug_msg.path.append(gps_point)

        goal_msg.search_for_object = search_for_object
        goal_msg.aruco_id = object_id

        self.get_logger().info('Waiting on action server')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.goal_feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_feedback_callback(self, feedback_msg):
        self.get_logger().info('Feedback received')
        traveled = feedback_msg.percent_traveled
        self.get_logger().info('Percent traveled: %f' % traveled)
        found_object = feedback_msg.found_object
        self.get_logger().info('Found object: %s' % found_object)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')


def main(args=None):
    rclpy.init(args=args)

    control_node = ControlNode()

    # path_m = [(10, 0), (10, 10), (0, 10), (0, 0)]
    path_m = [(0, 0), (-12, 2)]

    path_deg = [(meters_to_degrees(lat), meters_to_degrees(lon)) for lat, lon in path_m]

    control_node.send_path(path_deg, True, 0)

    rclpy.spin(control_node)

    control_node.destroy_node()
    rclpy.shutdown()