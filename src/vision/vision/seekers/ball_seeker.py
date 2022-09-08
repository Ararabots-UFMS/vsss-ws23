from typing import Tuple
import numpy as np
import cv2
import time

from vision.seekers.seeker import Seeker

# @author Wellington Castro <wvmcastro>

class BallSeeker(Seeker):
    """ This class takes a binary image with one object and finds its position.
        It also uses the temporal information to predict a search window in the
        picture."""

    def __init__(self, img_shape, color_thrs: Tuple[np.uint8, np.uint8]):

        # Stores the frame shape
        self.img_shape = img_shape
        self._color_thrs = color_thrs

        # Stores the obj last pos. Will help to predict the search region
        self.last_pos = None

        # Stores the last time the obj pos was coputed
        self.last_time = None

        self.obj_size = None

        # This speed is in the image world
        self.speed = None

        self.lost = 0

    def update_time(self, t=None):
        if t is None:
            self.last_time = time.time()
        else:
            self.last_time = t

    def update_state(self, pos):
        now = time.time()

        if np.all(self.last_pos != None):
            self.speed = (pos - self.last_pos) / (now - self.last_time)

        self.last_pos = pos
        self.update_time(t=now)

    def get_search_region(self, img):

        now = time.time()
        delta_t = now - self.last_time

        # Unpack the obj speed in x and y components
        vx = self.speed[0]
        vy = self.speed[1]

        s =  self.obj_size

        # Calculates the min search region
        if vy < 0:
            start_line = self.last_pos[1] - s + vy * delta_t
            end_line = self.last_pos[1] + s
        else:
            start_line = self.last_pos[1] - s
            end_line = self.last_pos[1] + s + vy * delta_t

        if vx < 0:
            start_col = self.last_pos[0] - s + vx * delta_t
            end_col = self.last_pos[0] + s
        else:
            start_col = self.last_pos[0] - s
            end_col = self.last_pos[0] + s + vx * delta_t

        # Make sure the window is inside the image boundaries
        start_line = int(start_line) if start_line >= 0 else 0
        end_line = int(end_line) if end_line < self.img_shape[0] else self.img_shape[0]-1
        start_col = int(start_col) if start_col >= 0 else 0
        end_col = int(end_col) if end_col < self.img_shape[1] else self.img_shape[1]-1

        # The origin must be [start_col, start_line] instead of [start_line, start_col]
        # because in OpenCV the firt coord is the column coord, but in numpy the first coord
        # is the line coord
        return np.array([start_col, start_line]), img[start_line:end_line, start_col:end_col]

    def get_obj_pos(self, img):
        if cv2.__version__[0] == '4':
            cnts, *_ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        else:
            _, cnts, *_ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        pos = np.array([None, None])
        c_x, c_y = None, None

        for cnt in cnts:
            # Compute the contour moment to extract its center
            M = cv2.moments(cnt)

            # Calculates the contour center
            if M['m00'] != 0:
                c_x = int(M['m10'] / M['m00'])
                c_y = int(M['m01'] / M['m00'])

                # If it is the first time the obj is detected.
                # Calculates its size
                if  self.obj_size is None:
                    # Rect is a list that has the [(x,y)center, (width, height), angle of rotation]
                    rect = cv2.minAreaRect(cnt)
                    self.obj_size = max(rect[1][0], rect[1][1])

        return np.array([c_x, c_y])


    def seek(self, img):
        if np.all(self.last_pos != None) and np.all(self.speed != None):
            origin, region = self.get_search_region(img)
        else:
            self.update_time()
            origin = np.array([0,0])
            region = img
            self.lost += 1

        region = cv2.inRange(region, self._color_thrs[0], self._color_thrs[1])
        pos = self.get_obj_pos(region)

        if np.all(pos != None):
            pos += origin
            self.update_state(pos)
        else:
            self.reset()

        return self.last_pos

    def reset(self, color_thrs=None) -> None:
        if color_thrs is not None:
            self._color_thrs = color_thrs
        self.last_pos = None
        self.last_time = None
        self.obj_size = None
        self.obj_speed = None
