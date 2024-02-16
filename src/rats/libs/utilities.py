from math import cos, radians, degrees, sin, atan2, pi, sqrt, asin
import numpy as np
import os
import cv2
import json
import socket
from typing import Dict
from argparse import ArgumentParser
from configparser import ConfigParser

def get_marker_location(corners, marker_size, intrinsic, distortion):
    _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, intrinsic, distortion)

    # this distance to the marker is the magnitude of the translation vector
    distance_to_marker = np.linalg.norm(tvecs[0][0])

    # this angle is the angle between the camera's x-axis and the marker's z-axis
    angle = degrees(atan2(tvecs[0][0][0], tvecs[0][0][2]))

    return angle, distance_to_marker


def abs_clamp(n: float, minn: float, maxn: float) -> float:
    """
    Clamps a number between a minimum and maximum value, but keeps the sign of the number

    Args:
        n (float): Number to clamp
        minn (float): Minimum value
        maxn (float): Maximum value

    Returns:
        float: Clamped number
    """
    sign = 1 if n > 0 else -1
    return max(min(maxn, abs(n)), minn) * sign

def wrap_bearing(bearing: float) -> float:
    """
    Wraps a bearing between -180 and 180

    Args:
        bearing (float): Bearing to wrap

    Returns:
        float: Wrapped bearing
    """
    if bearing > 180:
        return -(360-bearing)
    elif bearing < -180:
        return 360+bearing
    else:
        return bearing

def send_udp(host: str, port: int, message: bytearray) -> None: 
    """
    Sends a message over UDP to a specific host and port
    
    Args:
        host (str): IP address of host
        port (int): Port of host
        message (bytearray): Message to send
    """
    #sends a message over UDP to a specific host and port
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect((host, port))
            s.sendall(message)
        except OSError:
            pass
        #TODO: add better error handling or at least reporting

def parse_arguments():
    """
    Parses command line arguments
    
    Returns:
        args: Arguments
    """
    arg_parser = ArgumentParser()
    arg_parser.add_argument("-ll", "--latLong", required=True, type=str, help="takes a filename for a text file, then reads that file for latlong coordinates")
    arg_parser.add_argument("-c", "--camera", default=0, type=int, help="takes a number representing which camera to use")
    arg_parser.add_argument("-t1", "--tag1", default=-1, type=int, help="takes the id value of the first tag, defaults to -1 if id not assigned")
    arg_parser.add_argument("-t2", "--tag2", default=-1, type=int, help="takes the id value of the second tag, defaults to -1 if id not assigned")
    args = arg_parser.parse_args()
    return args

def parse_latlong_file(dir, filename) -> list:
    """
    Parses a file containing latitude and longitude coordinates
    
    Args:
        filename (str): Name of file to parse
        
    Returns:
        list: List of coordinates
    """
    locations = []
    path = os.path.join(dir, filename)
    with open(path) as f:
        line_number = 0
        for line in f:
            line_number += 1
            try:
                coords = [float(item.replace('\ufeff',"")) for item in line.strip().split()]
            except:
                print("Parse Error on line " + str(line_number) + ": Please enter <lat long>")
                break
            else:
                if len(coords) != 2:
                    print("Error on line " + str(line_number) + ": Insufficient number of coordinates. Please enter <lat long>")
                    break        
                locations.append(coords)

    return locations

def parse_config_file(filepath) -> Dict[str, Dict[str, str]]:
    """
    Parses a config file
    
    Args:
        filename (str): Name of file to parse
        
    Returns:
        dict[str, dict[str, str]]: Dictionary of config values with the top
        level keys being the section names and the second level keys
        being the config values
    """
    config = ConfigParser()
    success = config.read(filepath)
    if success == []:
        raise FileNotFoundError(f'Could not find config file at {filepath}')
    return {section: {entry.upper(): val for entry,val in config[section].items()} for section in config.sections()}

def calc_average_bearing(bearings: list) -> float:
    """
    Calculates the average bearing from a list of bearings

    Args:
        bearings (list): List of bearings between -180 and 180

    Returns:
        float: Average bearing
    """
    # if the average magnitude is greater than 90, then the average
    # is on the bottom side of the circle, so we need to
    # add 360 to all the negative bearings in the queue to properly
    # calculate the average
    average_mag = sum([abs(x) for x in bearings]) / len(bearings)
    print('av_mag', average_mag)
    if average_mag > 90:
        temp_bearings = [x + 360 if x < 0 else x for x in bearings]
        avg = (sum(temp_bearings) / len(bearings))
        # if the result is over 180, must be mapped back to -180 to 180
        if avg > 180:
            avg = -(360-avg)
    else:
        avg = sum(bearings) / len(bearings)
    return avg

def distance_to(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """ 
    Returns distance in meters between given latitude and longitude

    Args:
        lat1 (float): Latitude of first point
        lon1 (float): Longitude of first point
        lat2 (float): Latitude of second point
        lon2 (float): Longitude of second point

    Returns:
        float: Distance in meters between given latitude and longitude
    """
    EARTH_RADIUS = 6371.301
    delta_lat = (lat2 - lat1) * (pi/180.0)
    delta_lon = (lon2 - lon1) * (pi/180.0)

    a = sin(delta_lat/2) * sin(delta_lat/2) + cos(lat1 * (pi/180.0)) * cos(lat2 * (pi/180.0)) * sin(delta_lon/2) * sin(delta_lon/2)
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    return EARTH_RADIUS * c * 1000

def calc_bearing(lat1:float, lon1:float, lat2:float, lon2:float) -> float:
    """
    Calculates bearing between two points. (0 is North, 90 is East, +/-180 is South, -90 is West)

    Args:
        lat1 (float): Latitude of first point
        lon1 (float): Longitude of first point
        lat2 (float): Latitude of second point
        lon2 (float): Longitude of second point

    Returns:
        float: Bearing between two points. (0 is North, 90 is East, +/-180 is South, -90 is West)
    """
    x = cos(lat2 * (pi/180.0)) * sin((lon2-lon1) * (pi/180.0))
    y = cos(lat1 * (pi/180.0)) * sin(lat2 * (pi/180.0)) - sin(lat1 * (pi/180.0)) * cos(lat2 * (pi/180.0)) * cos((lon2-lon1) * (pi/180.0))
    return (180.0/pi) * atan2(x,y)

def get_coordinates(lat: float, lon: float, distance: float, bearing: float) -> "(float, float)":
    """
    Calculate latitude and longitude given distance (in m) and bearing (in degrees)

    Args:
        lat (float): Latitude of the starting point
        lon (float): Longitude of the starting point
        distance (float): Distance to the destination point in meters
        bearing (float): Bearing to the destination point in degrees

    Returns:
        tuple: A tuple containing the latitude and longitude of the destination point
    """
    # https://stackoverflow.com/questions/7222382/get-lat-long-given-current-point-distance-and-bearing
    EARTH_RADIUS = 6371.301
    brng = radians(bearing)      # Assuming bearing is in degrees
    d = distance / 1000 # Distance in km

    lat1 = radians(lat)   # Current lat point converted to radians
    lon1 = radians(lon)  # Current long point converted to radians

    lat2 = asin(sin(lat1)*cos(d/EARTH_RADIUS) + 
                cos(lat1)*sin(d/EARTH_RADIUS)*cos(brng))
    lon2 = lon1 + atan2(sin(brng)*sin(d/EARTH_RADIUS)*cos(lat1),
                        cos(d/EARTH_RADIUS)-sin(lat1)*sin(lat2))
    return degrees(lat2), degrees(lon2)

def degrees_to_meters(degrees: float) -> float:
    """
    Converts degrees to meters (Spherical Earth model)

    Args:
        degrees (float): Degrees to convert

    Returns:
        float: Meters
    """
    return degrees * 111139

def meters_to_degrees(meters: float) -> float:
    """
    Converts meters to degrees (Spherical Earth model)

    Args:
        meters (float): Meters to convert

    Returns:
        float: Degrees
    """
    return meters / 111139

def generate_default_matrices(width, height, v_fov) -> tuple[np.ndarray, np.ndarray]:
    """
    Generates default intrinsic and distortion matrices for simulation.
    ALWAYS USE PROPERLY CALIBRATED CAMERAS FOR REAL WORLD USE!
    
    Args:
        width (int): Width of camera
        height (int): Height of camera
        v_fov (float): Vertical field of view of camera
        
    Returns:
        tuple[np.ndarray, np.ndarray]: Intrinsic and distortion matrices
    """
    # generate default intrinsic and distortion matrices
    f = width / (4.0 * np.tan(np.radians(v_fov / 2)))
    intrinsic = np.array([
        [f, 0, width / 2],
        [0, f, height / 2],
        [0, 0, 1]
    ], dtype=np.float32)
    distortion = np.array([0, 0, 0, 0, 0], dtype=np.float32)
    return intrinsic, distortion

def get_camera_matrices(name, width, height, v_fov) -> tuple[np.ndarray, np.ndarray]:
    """
    Gets the intrinsic and distortion matrices from a calibration file
    
    Args:
        name (str): Name of camera
        width (int): Width of camera
        height (int): Height of camera
        v_fov (float): Vertical field of view of camera
        
    Returns:
        tuple[np.ndarray, np.darray]: Intrinsic and distortion matrices
    """
    cwd = os.getcwd()
    ind = cwd.find('Autonomous')
    cwd = cwd[:ind+11]
    # get the intrinsic and distortion matrices from a calibration file
    default_intrinsic, default_distortion = generate_default_matrices(width, height, v_fov)
    try:
        intrinsic = np.load(f'{cwd}/cfg/{name}_{width}_{height}_intrinsic.npy')
    except FileNotFoundError:
        print(f'Could not find intrinsic matrix for {name} at {cwd}/cfg/{name}_{width}_{height}_intrinsic.npy'
            f'\nUsing default intrinsic matrix')
        intrinsic = default_intrinsic

    try:
        distortion = np.load(f'{cwd}/cfg/{name}_{width}_{height}_distortion.npy')
    except FileNotFoundError:
        print(f'Could not find distortion matrix for {name} at {cwd}/cfg/{name}_{width}_{height}_distortion.npy'
            f'\nUsing default distortion matrix')
        distortion = default_distortion

    return intrinsic, distortion

def get_camera_params_file(camerafile: str) -> list:
    """
    Gets a list of cameras from a file
    
    Args:
        camerafile (str): Path to camera json file
        renderer (Renderer): Renderer to use for mocked cameras

    Returns:
        list[Camera]: List of cameras
    """
    cameras = []
    camerafile = os.path.abspath(camerafile)
    if not os.path.exists(camerafile):
        raise FileNotFoundError(f'Could not find camera file at {camerafile}')
    with open(camerafile) as f:
        data = json.load(f)
        indices = data['indices']
        camera_data = [data['cameras'][i] for i in indices]
        for camera in camera_data:
            name = camera['name']
            device = camera['device']
            width = camera['width']
            height = camera['height']
            fps = camera['fps']
            v_fov = camera['v_fov']
            matrix_name = camera['matrix_name']
            c_format = camera['format']
            x_offset = camera['x_offset']
            z_offset = -camera['z_offset']
            yaw = camera['yaw']
            intrinsic, distortion = get_camera_matrices(matrix_name, width, height, v_fov)
            position = np.array([x_offset, 0, z_offset], dtype=np.float32)
            camera_params = {
                'device': device,
                'width': width,
                'height': height,
                'fps': fps,
                'v_fov': v_fov,
                'name': name,
                'format': c_format,
                'intrinsic': intrinsic,
                'distortion': distortion,
                'position': position,
                'yaw': yaw
            }
            cameras.append(camera_params)
    return cameras


if __name__ == "__main__":
    print("This is a library, not a script.")
    cfg = parse_config_file("/home/benton/Documents/code/Autonomous/cfg/config.ini")
    print(cfg)