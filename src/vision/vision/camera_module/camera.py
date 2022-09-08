from collections import deque
import imp
from typing import Union, Tuple
import cv2
import numpy as np
from threading import Thread, Semaphore
import pickle
import os
from rclpy.node import Node

from utils.json_handler import JsonHandler

# @author Wellington Castro <wvmcastro>

class Camera:
    def __init__(self,node: Node, device_id: Union[int, str] = 0,
                 params_file_name: str = "",
                 lens_correction: bool = True,
                 threading: bool = False):
        self._node = node
        self.id = device_id
        self.lens_correction = lens_correction
        self.params_file_name = params_file_name
        self.threading = threading
        self.capture = cv2.VideoCapture(self.id)
        self.thread_stopped = True
        self.is_file = os.path.isfile(self.id)

        self.json_handler = JsonHandler()
        self.frame = None

        if self.params_file_name != "":
            self._load_params()
            self.set_device(self.frame_width, self.frame_height)
            self.capture_frame = self._capture_and_correct_frame
        else:
            self.capture_frame = self._capture_frame

        self._calibrate_sensors()

        if self.threading == True:
            self._buffer_semaphore = Semaphore(0)
            self.buffer = deque(maxlen=1)
            self._start_reading_process()
            self._read = self._threaded_read
        else:
            self._read = self._sequential_read

    def _calibrate_sensors(self):
        i = 0
        while i < 5:
            ret, _ = self._capture_frame()
            if ret == True:
                i += 1

    def _start_reading_process(self) -> None:
        self.thread_stopped = False
        p = Thread(target=self._update, args=())
        p.daemon = True
        p.start()

    def stop(self) -> None:
        if not self.thread_stopped:
            self.thread_stopped = True

    def _update(self) -> None:
        while not self.thread_stopped:
            frame = self.capture_frame()
            bufferFull = len(self.buffer)
            self.buffer.append(frame)
            if not bufferFull:
                self._buffer_semaphore.release()

    def _capture_frame(self) -> np.ndarray:
        ret, frame = self.capture.read()
        if self.is_file and not ret:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.capture.read()
        return ret, frame

    def _capture_and_correct_frame(self) -> np.ndarray:
        ret, frame = self.capture.read()

        if self.is_file and not ret:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.capture.read()

        frame = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)
        return ret, frame

    def read(self) -> np.ndarray:
        try:
            return self._read()
        except Exception as excecption:
            self._node.get_logger().fatal(f"Camera:{excecption}")

    def _threaded_read(self) -> np.ndarray:
        self._buffer_semaphore.acquire()
        frame =  self.buffer.popleft()
        return frame

    def _sequential_read(self) -> np.ndarray:
        frame = self.capture_frame()
        return frame

    def _load_params(self) -> None:
        file1 = os.environ['ROS_ARARA_ROOT']+"src/" \
                + self.params_file_name
        if os.path.exists(file1):
            self.params_file_name = file1
        elif not os.path.exists(self.params_file_name):
            return

        with open(self.params_file_name, "rb") as fp:
            params = pickle.load(fp)

        """ mapx and mapy are the matrix with the lens correction map """
        self.mapx = params['matrix_x']
        self.mapy = params['matrix_y']
        # self.mapx, self.mapy = cv2.convertMaps(mapx, mapy, cv2.CV_16SC2)

        """ The frame width and height """
        self.frame_width = params['default_frame_width']
        self.frame_height = params['default_frame_height']

        """ The matrix of the intrinsic parameters and the vector of distortion coefficients """
        self.camera_matrix = params['cam_matrix']
        self.dist_vector = params['dist_vector']

    def set_device(self, width: int, height: int) -> None:
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def __repr__(self) -> str:
        return "Camera(device_id=%r, params_file_name=%r, lens_correction=%r, " \
               "threading=%r)" % (self.id, self.params_file_name,
                                  self.lens_correction, self.threading)
