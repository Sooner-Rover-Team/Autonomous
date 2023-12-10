from math import cos, sin
from threading import Thread
from time import sleep
import time

from numpy.random import Generator, PCG64

from libs.utilities import calc_bearing, distance_to, meters_to_degrees, calc_average_bearing
from libs.Wheels import WheelInterface

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

class GPSInterface:
    """
    Interface for GPS
    
    Attributes:
        latitude (float): Current latitude
        longitude (float): Current longitude
        old_latitude (float): Previous latitude
        old_longitude (float): Previous longitude
        height (float): Current height
        time (float): Current time
        error (float): Current error
        bearing (float): Current bearing
        running (bool): Whether the GPS is running
        _UPDATE_PERIOD (float): How often to query the GPS for new data
        _AVERAGE_BEARING_LENGTH (int): How many bearings to average over time
    """
    def __init__(self):
        self.latitude = 0 # degrees
        self.longitude = 0 # degrees 
        self.old_latitude = 0 # degrees
        self.old_longitude = 0 # degrees
        self.height = 0 # meters (? not sure never used this) 
        self.time = 0 # seconds (? not sure about this either)
        self.error = 0 # meters (? this needs to be checked)
        self.bearing = 0.0 # degrees
        self.running = True
        self._UPDATE_PERIOD = .25 # seconds

        # create a circular queue of average bearings
        # to smooth out some noise
        self._AVERAGE_BEARING_LENGTH = 4
        self._average_bearing = [0] * self._AVERAGE_BEARING_LENGTH
        self._average_bearing_index = 0

    def get_position(self):
        """
        Returns current latitude and longitude

        Returns:
            tuple: (latitude, longitude)
        """
        return (self.latitude, self.longitude)

    def distance_to(self, lat:float, lon:float):
        """
        Returns distance in kilometers between self and given latitude and longitude

        Args:
            lat (float): Latitude of second point
            lon (float): Longitude of second point

        Returns:
            float: Distance in kilometers between self and given latitude and longitude
        """
        return distance_to(self.latitude, self.longitude, lat, lon)

    def bearing_to(self, lat:float, lon:float):
        """ 
        Calculates difference between given bearing to location and current bearing
        Positive is turn right, negative is turn left

        Args:
            lat (float): Latitude of second point
            lon (float): Longitude of second point

        Returns:
            float: Difference between given bearing to location and current bearing
        """
        resultbearing = calc_bearing(self.latitude, self.longitude, lat, lon) - self.bearing
        if resultbearing < -180:
            resultbearing += 360
        elif resultbearing > 180:
            resultbearing -= 360
        return resultbearing

    def start(self, ip, port):
        """
        Starts the GPS
        """
        self.swift_ip = ip
        self.swift_port = port
        gps.gps_init(self.swift_ip, self.swift_port)
        self.running = True
        self._thread = Thread(target=self._update_fields_loop, name=(
                'update GPS fields'), args=())
        self._thread.daemon = True
        self._thread.start()

    def stop(self):
        """
        Stops the GPS
        """
        self.running = False
        self._thread.join()
        gps.gps_finish()

    def calc_avg_bearing(self, lat1, lon1, lat2, lon2):
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
        self._average_bearing[self._average_bearing_index] = bearing
        self._average_bearing_index = (self._average_bearing_index + 1) % self._AVERAGE_BEARING_LENGTH

        # if the average magnitude is greater than 90, then the average
        # is probably on the other side of the circle, so we need to
        # add 360 to all the negative bearings in the queue
        return calc_average_bearing(self._average_bearing)

    def _update_fields_loop(self):
        """
        Updates the GPS fields repeatedly
        """
        while(self.running):
            if gps.get_latitude() != 0 or gps.get_longitude() != 0:
                self.old_latitude = self.latitude
                self.old_longitude = self.longitude

                self.latitude = gps.get_latitude()
                self.longitude = gps.get_longitude()
                self.height = gps.get_height()
                self.time = gps.get_time()
                self.error = gps.get_error()
                self.bearing = self.calc_avg_bearing(self.old_latitude, self.old_longitude, self.latitude, self.longitude)
            else:
                pass
            # maybe print info or sleep or something idk
            sleep(self._UPDATE_PERIOD)

class MockedGPSInterface(GPSInterface):
    def __init__(self, wheels: WheelInterface):
        super().__init__()
        self._wheels = wheels
        self._real_lat = 0 # degrees
        self._real_lon = 0 # degrees
        self._real_bearing = 0 # radians
        # generator for noise
        self._ADD_NOISE = True
        self._noise_dev_m = 0.03 # 30 centimeters
        self._noise_generator = Generator(PCG64(time.time_ns()))
        # made this up, should be close to real speed
        self._SPEED = 1.73 # meters per second
        # pulled this out of my ass, basically how much the rover 
        # turns when one wheel is at full speed and the other is stopped
        self._single_side_authority = .53 # i guess the unit is rad/m/s (???)

    def update_position(self, dt: float) -> None:
        """
        Updates the real position based on current wheel speeds
        """
        # get the current wheel speeds
        left_wheel_speed, right_wheel_speed = self._wheels.get_wheel_speeds()

        # distance traveled is a function of base speed, wheel speeds, and time
        distance_traveled = (self._SPEED * (left_wheel_speed + right_wheel_speed) / 2) * dt

        # figure out how much the bearing has changed using this bullshit number
        # (seriously if someone has a better tank control model please replace this)
        bearing_change = (left_wheel_speed - right_wheel_speed) * self._single_side_authority
        
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
        return self._real_bearing

    def _update_fields_loop(self) -> None:
        """
        Updates the GPS fields repeatedly
        """
        while(self.running):
            # remember old lat and long
            self.old_latitude = self.latitude
            self.old_longitude = self.longitude
            
            # update the real position
            if self._ADD_NOISE:
                # if noise is enabled, add noise to the real position
                noise_dev_deg = meters_to_degrees(self._noise_dev_m)
                self.latitude = self._real_lat + self._noise_generator.normal(0, noise_dev_deg)
                self.longitude = self._real_lon + self._noise_generator.normal(0, noise_dev_deg)
            else:
                # otherwise just set the real position
                self.latitude = self._real_lat
                self.longitude = self._real_lon

            # update the bearing
            self.bearing = self.calc_avg_bearing(self.old_latitude, self.old_longitude, self.latitude, self.longitude)
            sleep(self._UPDATE_PERIOD)