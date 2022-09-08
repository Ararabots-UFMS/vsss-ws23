from typing import List
import numpy as np
import cv2
import time
import math
#TODO: (Aruco) Re-importar caso necessite 
#from vision_module.seekers.aruco_seeker import ArucoSeeker
from vision.seekers.ball_seeker import BallSeeker
from vision.seekers.general_mult_obj_seeker import GeneralMultObjSeeker
from vision.seekers.circular_color_tag_seeker import CircularColorTagSeeker

# @author Wellington Castro <wvmcastro>

# Sorry for these globals, but is good for code reading
# Go Ararabots!
ID = 0
POS = 1
ANGLE = 2
SPEED_QUEUE_SIZE = 60.0


class Things:
    # This is an auxiliary class to hold the variables from the things identified
    # by this hawk eye system
    def __init__(self):
        self.id = -1

        self.lost_counter = 0

        # Stores the (x,y) pos from the rebot
        self.pos = np.array([None, None])

        # The orientation is stored in radians in relation with the x axis
        self.orientation = None

        # Saves the time of the last update
        self.last_update = None

        # This variable is used as the kalman filter object
        self.kalman = None

        self.angular_kalman = None

        self.init_kalman()

    def init_angular_kalman(self):
        dt = 1.0
        self.angular_kalman = cv2.KalmanFilter(3, 1, 0)
        self.angular_kalman.transitionMatrix = np.array([[1., dt, .5 * dt ** 2],
                                                         [0., 1., dt],
                                                         [0., 0., 1.]]).reshape(3, 3)
        self.angular_kalman.processNoiseCov = 1e-5 * np.eye(3)
        self.angular_kalman.measurementNoiseCov = 1e-1 * np.ones((1, 1))
        self.angular_kalman.measurementMatrix = 0. * np.zeros((1, 3))
        self.angular_kalman.measurementMatrix[0, 0] = 1.
        self.angular_kalman.errorCovPost = 1. * np.ones((3, 3))
        self.angular_kalman.statePost = np.array([[0., 0., 0.]]).reshape(3, 1)

    def init_kalman(self):
        # estimated frame rate
        dt = 1.0
        self.kalman = cv2.KalmanFilter(6, 2, 0)
        self.kalman.transitionMatrix = np.array([[1., 0., dt, 0, .5 * dt ** 2, 0.],
                                                 [0., 1., 0., dt, 0., .5 * dt ** 2],
                                                 [0., 0., 1., 0., dt, 0.],
                                                 [0., 0., 0., 1., 0., dt],
                                                 [0., 0., 0., 0., 1., 0.],
                                                 [0., 0., 0., 0., 0., 1.]]).reshape(6, 6)

        self.kalman.processNoiseCov = 1e-5 * np.eye(6)
        self.kalman.measurementNoiseCov = 1e-1 * np.ones((2, 2))

        self.kalman.measurementMatrix = 0. * np.zeros((2, 6))
        self.kalman.measurementMatrix[0, 0] = 1.
        self.kalman.measurementMatrix[1, 1] = 1.

        self.kalman.errorCovPost = 1. * np.ones((6, 6))
        self.kalman.statePost = np.array([[0., 0., 0., 0., 0., 0.]]).reshape(6, 1)

        self.init_angular_kalman()

    def set_dt(self, time_now):
        dt = time_now - self.last_update
        self.kalman.transitionMatrix[0, 2] = dt
        self.kalman.transitionMatrix[1, 3] = dt

    def update(self, id, pos, orientation=None):
        now = time.time()
        if self.last_update is None and np.all(pos is not None):  # first run
            self.init_kalman()
            # A initialization state must be provided to the kalman filter
            self.kalman.statePost = np.array([[pos[0], pos[1], 0., 0., 0., 0.]]).reshape(6, 1)
            if orientation is not None:
                self.angular_kalman.statePost = np.array([[orientation, 0., 0.]]).reshape(3, 1)
            else:

                self.angular_kalman.statePost = np.array([[0, 0., 0.]]).reshape(3, 1)
            self.lost_counter = 0
        else:
            self.kalman.predict()
            self.angular_kalman.predict()

            # updates the kalman filter
            if np.all(pos is not None) and self.lost_counter < 60:
                self.kalman.correct(pos.reshape(2, 1))
                if orientation is not None:
                    self.angular_kalman.correct(np.array([orientation]).reshape(1, 1))
                self.lost_counter = 0
            else:  # updates the lost counter
                self.lost_counter += 1

            # uses the kalman info
            state = self.kalman.predict()
            pos = np.array([state[0, 0], state[1, 0]])

            if orientation is not None and abs(abs(orientation) - math.pi) < 0.15:
                # self.init_angular_kalman()
                self.angular_kalman.statePost = np.array([[orientation, 0., 0.]]).reshape(3, 1)
            elif orientation is None:
                orientation = self.angular_kalman.predict()[0, 0]

        if self.lost_counter >= 60:  # if the thing was lost in all previous 10 frames
            self.reset()
        else:
            # Updates the robot's state variables
            self.id = id
            self.last_update = now
            self.pos = pos
            self.orientation = orientation

    def reset(self):
        self.pos = np.array([-1, -1]) # np.array([None, None])
        self.last_update = -1 # None
        self.orientation = -1 # None


class HawkEye:
    """ This class will be responsible of locate and identify all objects present
        in the field """

    # https://docs.opencv.org/master/d9/d8b/tutorial_py_contours_hierarchy.html#gsc.tab=0

    def __init__(self, field_origin, conversion_factor_x, conversion_factor_y, seekers, num_robots_yellow_team,
                 num_robots_blue_team, img_shape, aux_params):

        self.team_seekers = seekers

        self.field_origin = field_origin
        self.conversion_factor_x = conversion_factor_x
        self.conversion_factor_y = conversion_factor_y
        self.rad_to_degree_factor = 180.0 / math.pi
        self.num_robots_yellow_team = num_robots_yellow_team
        self.num_robots_blue_team = num_robots_blue_team

        self.yellow_team_seeker, self.seek_yellow_team = self.get_seeker(
            self.team_seekers["yellow"], 
            self.num_robots_yellow_team, 
            aux_params)

        self.blue_team_seeker, self.seek_blue_team = self.get_seeker(
            self.team_seekers["blue"],
            self.num_robots_blue_team,
            aux_params)

        self.ball_seeker = BallSeeker(img_shape, aux_params["ball"])
    
    def get_seeker(self, seeker_name: str, num_robots: int = 0, aux_params: dict = None):
        if seeker_name == "aruco":
            camera_matrix = aux_params["aruco"][0]
            distortion_vector = aux_params["aruco"][1]
            # seeker = ArucoSeeker(camera_matrix, 
            #                            distortion_vector,
            #                            num_robots)
            #return seeker, self.aruco_seek
            return None, self.aruco_seek
        elif seeker_name == "color":
            return CircularColorTagSeeker(aux_params["color"]), self.color_seek
        elif seeker_name == "kmeans":
            seeker = GeneralMultObjSeeker(num_robots)
            return seeker, self.kmeans_seek

    def aruco_seek(self, img: np.ndarray, 
                         robots_list: List[Things], 
                         seeker, 
                         opt=None):
        pass
    #TODO: (Aruco) Descomentar caso necessite 
    # def aruco_seek(self, img: np.ndarray, 
    #                      robots_list: List[Things], 
    #                      seeker: ArucoSeeker, 
    #                      opt=None):
    #     """ This function expects a binary image with the team robots and a list
    #                 of Things objects to store the info """
    #     img = 255 - img
    #     robots = seeker.seek(img, degree=False)
    #     # TODO: ISSO AQUI UM DIA VAI DAR MERDA
    #     # EXPLODIU, O DIA CHEGOU 14/08/2019, a casa caiu
    #     # é a nova era, petista
    #     tags = [9, 14, 18, 23, 28]
    #     # tags_in_game = []
    #     k = 0

    #     len_robots = len(robots)
    #     for i in range(len(robots_list)):
    #         id = None if k >= len_robots else robots[k][ID]
    #         if tags[i] == id:
    #             pos = self.pixel_to_real_world(robots[k][POS])
    #             _orientation = robots[k][ANGLE]
    #             k += 1
    #         else:
    #             pos, _orientation = None, None

    #         # if tags[i] in tags_in_game:
    #         robots_list[i].update(i, pos, orientation=_orientation)

    def pixel_to_real_world(self, pos):
        # This function expects that pos is a 1D numpy array
        pos = pos - self.field_origin
        pos[0] *= self.conversion_factor_x
        pos[1] *= -self.conversion_factor_y
        return pos


    def seek_ball(self, img: np.ndarray, 
                        ball: Things, 
                        opt=None):
        pos = self.ball_seeker.seek(img)
        if np.all(pos is not None):
            pos = self.pixel_to_real_world(pos)

        ball.update(0, pos)

    def kmeans_seek(self, img: np.ndarray, 
                          robots_list: List[Things], 
                          seeker: GeneralMultObjSeeker, 
                          opt=None):
        adv_centers = seeker.seek(img)

        if not (adv_centers is None) and adv_centers.size:
            for i in range(len(robots_list)):
                pos = self.pixel_to_real_world(adv_centers[i, ...])
                robots_list[i].update(i, pos)
    
    def color_seek(self, binary_img: np.ndarray, 
                         robots_list: List[Things], 
                         seeker: CircularColorTagSeeker,
                         opt = None) -> None:
        color_img = opt
        robots = seeker.seek(binary_img, color_img)
        num_detected_robots = len(robots)

        k = 0
        for i in range(len(robots_list)):
            if k < num_detected_robots and robots[k][ID] == i:
                pos = self.pixel_to_real_world(robots[k][POS])
                orientation = robots[k][ANGLE]
                k += 1
            else:
                pos, orientation = None, None
            
            robots_list[i].update(i, pos, orientation)


    def reset(self, aux_params=None) -> None:
        try:
            yellow_seeker = self.team_seekers["yellow"]
            self.yellow_team_seeker.reset(aux_params[yellow_seeker])
        except:
            self.yellow_team_seeker.reset()
        try:
            blue_seeker = self.team_seekers["blue"]
            self.blue_team_seeker.reset(aux_params[blue_seeker])
        except:
            self.blue_team_seeker.reset()
        
        self.ball_seeker.reset(aux_params["ball"])