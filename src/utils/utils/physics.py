from collections import deque
from utils.linalg import *
import numpy as np
import time

class MovingBody:
    def __init__(self, buffer_size=15):
        self._position = Vec2D.origin()
        self._speed = Vec2D.origin()

        self.position_buffer_x = deque((0 for _ in range(buffer_size)), maxlen=buffer_size)
        self.position_buffer_y = deque((0 for _ in range(buffer_size)), maxlen=buffer_size)

        self.speed_buffer_x = deque((0 for _ in range(buffer_size)), maxlen=buffer_size)
        self.speed_buffer_y = deque((0 for _ in range(buffer_size)), maxlen=buffer_size)

        t0 = time.time()
        self.time_buffer = deque((t0 for i in range(buffer_size)), maxlen=buffer_size)

        self._last_update = t0
        self._speed = Vec2D.origin()

        self.orientation = .0
        self.vorientation = .0

    def position_prediction(self, seconds=0.5):

        dt = seconds
        p0 = self.position
        v = self.speed
        p = p0 + v*dt

        return p

    def get_time_on_axis(self, axis, value) -> float:
        if axis == 0:
            ds = value - self.position[0]
            dt = ds / (self._speed[0] + 1e-5)
        else:
            ds = value - self.position[1]
            dt = ds / (self._speed[1] + 1e-5)
        
        if dt < 0 or abs(dt) > 2: 
            dt = 0

        return dt
    
    @property
    def speed(self) -> Vec2D:
        return self._speed
    
    @property
    def position(self) -> Vec2D:
        return self._position
    
    @position.setter
    def position(self, position: Vec2D) -> None:
        
        t = time.time()
        # dt = t - self._last_update
        # last_pos = self.position
        # TODO: Refatorar aqui para usar a speed direto do simulador!
        # self._speed = (0.1 / dt ) * (position - last_pos) + 0.9 * self._speed

        self.position_buffer_x.append(position[0])
        self.position_buffer_y.append(position[1])
        self.time_buffer.append(float(time.time()))
        
        self._last_update = t
        self._position = position

    def __repr__(self):
        return "--position: " + str(self.position) + \
               "--speed: " + str(self.speed) + \
               "--orientation: " + str(self.orientation)
