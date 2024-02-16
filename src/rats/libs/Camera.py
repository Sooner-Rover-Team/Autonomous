import cv2
import threading
import os
import numpy as np

class Camera:
    """
    Interface for Camera

    Attributes:
        _camera_id (int): Camera ID
        _width (int): Frame width
        _height (int): Frame height
        _framerate (int): Frame rate
        _format (str): Frame format
        _intrinsic (np.ndarray): Camera intrinsic matrix
        _distortion (np.ndarray): Camera distortion matrix
        _cap (cv2.VideoCapture): OpenCV capture object
        _frame (np.ndarray): Current frame
        _running (bool): Whether the camera is running
        _frame_lock (threading.Lock): Lock for the frame
        _cap_thread (threading.Thread): Thread for reading frames
    """
    def __init__(self, device, width, height, fps, v_fov=50, name="", matrix_name="default", format='MJPG', intrinsic=None, distortion=None, position=None, yaw=None) -> None:
        self._device = device
        self._width = width
        self._height = height
        self._fps = fps
        self._v_fov = v_fov
        self._name = name
        self._format = format

        if intrinsic is not None:
            self._intrinsic = intrinsic
        else:
            self._intrinsic = None

        if distortion is not None:
            self._distortion = distortion
        else:
            self._distortion = None

        if position is not None:
            self._position = position
        else:
            self._position = np.array([0, 0, 0], dtype=np.float32)

        if yaw is not None:
            self._yaw = yaw
        else:
            self._yaw = 0

        self._cap = None
        self._frame = None
        self._running = False
        self._frame_lock = threading.Lock()
        self._cap_thread = threading.Thread(target=self._cap_loop, name='camera read loop')

    def set_frame_callback(self, callback: callable) -> None:
        """
        Sets a callback for when a frame is read
        
        Args:
            callback (callable): Callback function
        """
        self._frame_callback = callback

    def _open_camera(self, camera_id: int) -> cv2.VideoCapture:
        """
        Opens the camera
        
        Args:
            camera_id (int): Camera ID

        Returns:
            cv2.VideoCapture: OpenCV capture object
        """
        if os.name == 'nt':
            # use directshow on windows (seems to work better)
            backend = cv2.CAP_DSHOW
        else:
            # TODO: Figure out the best backend for the rover
            backend = cv2.CAP_V4L2
        cap = cv2.VideoCapture(camera_id, backend)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        cap.set(cv2.CAP_PROP_FPS, self._fps)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*self._format.upper()))
        return cap

    def __del__(self):
        self.stop()

    def load_intrinsic(self, path: str) -> None:
        """
        Loads the intrinsic matrix from a file
        
        Args:
            path (str): Path to the intrinsic matrix
        """
        self._intrinsic = np.load(path)

    def load_distortion(self, path: str) -> None:
        """
        Loads the distortion matrix from a file

        Args:
            path (str): Path to the distortion matrix
        """
        self._distortion = np.load(path)

    def set_intrinsic(self, intrinsic: np.ndarray) -> None:
        """
        Sets the intrinsic matrix
        
        Args:
            intrinsic (np.ndarray): Intrinsic matrix
        """
        self._intrinsic = intrinsic

    def set_distortion(self, distortion: np.ndarray) -> None:
        """
        Sets the distortion matrix

        Args:
            distortion (np.ndarray): Distortion matrix
        """
        self._distortion = distortion

    def get_intrinsic(self) -> np.ndarray:
        """
        Gets the intrinsic matrix
        
        Returns:
            np.ndarray: Intrinsic matrix
        """
        return self._intrinsic
    
    def get_distortion(self) -> np.ndarray:
        """
        Gets the distortion matrix

        Returns:
            np.ndarray: Distortion matrix
        """
        return self._distortion

    def set_property(self, prop: int, value: int) -> None:
        """
        Sets a cv2 property of the camera
        
        Args:
            prop (int): Property to set
            value (int): Value to set the property to
        """
        self._cap.set(prop, value)

    def get_property(self, prop: int) -> int:
        """
        Gets a cv2 property of the camera

        Args:
            prop (int): Property to get

        Returns:
            int: Value of the property
        """
        return self._cap.get(prop)
    
    def set_position(self, position: np.ndarray) -> None:
        """
        Sets the position of the camera
        
        Args:
            position (np.ndarray): Position of the camera
        """
        self._position = position

    def set_yaw(self, theta: float) -> None:
        """
        Sets the yaw of the camera

        Args:
            theta (float): Yaw of the camera
        """
        self._yaw = theta

    def get_position(self) -> np.ndarray:
        """
        Gets the position of the camera
        
        Returns:
            np.ndarray: Position of the camera
        """
        return self._position
    
    def get_yaw(self) -> float:
        """
        Gets the yaw of the camera

        Returns:
            float: Yaw of the camera
        """
        return self._yaw

    def start(self) -> None:
        """
        Starts the camera
        """
        if not self._running:
            if self._cap is None:
                self._cap = self._open_camera(self._device)
            self._running = True
            self._cap_thread.start()

    def stop(self) -> None:
        """
        Stops the camera
        """
        if self._running:
            self._running = False
            self._cap_thread.join()
            if self._cap is not None:
                self._cap.release()
                self._cap = None

    def _cap_loop(self) -> None:
        """
        Thread loop for reading frames
        """
        # last_time = perf_counter_ns()
        while self._running:
            grabbed, frame = self._cap.read()
            if grabbed and frame is not self._frame:
                # elapsed = perf_counter_ns() - last_time
                # last_time = perf_counter_ns()
                # print(f'Frame time: {elapsed/1e6} ms')
                if self._frame_callback is not None:
                    self._frame_callback(self, frame)
                else:
                    with self._frame_lock:
                        self._frame = frame

    def get_frame(self) -> np.ndarray:
        """
        Gets the current frame
        
        Returns:
            np.ndarray: Current frame
        """
        with self._frame_lock:
            return self._frame
