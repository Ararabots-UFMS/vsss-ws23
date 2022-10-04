import cv2
import numpy as np
import rclpy
from rclpy.node import Node
import time
import sys
import pickle
import os
from time import sleep

from vision.camera_module.camera import Camera
from vision.vision_utils.params_setter import ParamsSetter
from vision.vision_utils.color_segmentation import ColorSegmentation
from vision.seekers.things_seeker import HawkEye
from vision.seekers.things_seeker import Things
from vision import COLORS
from sys_interfaces.msg import GameTopic
from utils.json_handler import JsonHandler
from vision.ros_vision_publisher import RosVisionPublisher
from rclpy.qos import QoSPresetProfiles
from vision.seekers.circular_color_tag_seeker import CircularColorTagSeeker


# @author Wellington Castro <wvmcastro>

HEIGHT = 1


class Vision:

    def __init__(self,node: Node, camera, num_blue_robots, num_yellow_robots,
                 params_file_name="", colors_params="", method="", vision_owner: str = 'Player_One'):

        self._node = node
        # This object will be responsible for publish the game state info
        # at the bus. Mercury is the gods messenger
        self.mercury = RosVisionPublisher(self._node, True)

        # Lists used to unpack the info from Things objects and publish those
        # infos in the vision topic

        # Ball info
        self.ball_pos = np.array([[.0, .0]])
        self.ball_speed = np.array([[.0, .0]])

        # Home team info
        self.yellow_team_pos = np.array([[0.0, 0.0]] * 5)
        self.yellow_team_orientation = np.array([0.0] * 5)
        self.yellow_team_speed = np.array([[0.0, 0.0]] * 5)

        # Adv team info
        self.blue_team_pos = np.array([[0.0, 0.0]] * 5)
        self.blue_team_orientation = np.array([0.0] * 5)
        self.blue_team_speed = np.array([[0.0, 0.0]] * 5)

        # Subscribes to the game topic
        self._node.create_subscription(
            GameTopic,
            'game_topic', 
            self.on_game_state_change,
            qos_profile=QoSPresetProfiles.SYSTEM_DEFAULT
        )

        self.game_state = None

        self.camera = camera
        self.params_file_name = params_file_name
        self.num_yellow_robots = num_yellow_robots
        self.num_blue_robots = num_blue_robots

        self.arena_vertices = []
        self.arena_size = ()
        self.arena_image = None
        self.arena_mask = None
        self.raw_image = None
        self.warp_matrix = None
        self.pipeline = None
        self.fps = 0
        self.last_time = None
        self.new_time = None
        self._colors_thresholds = {}
        self.hawk_eye_extra_params = {}

        # Super necessary to compute the robots positions
        self.origin = None
        self.conversion_factor_x = None
        self.conversion_factor_y = None

        self.game_on = False
        self.in_calibration_mode = False
        self.finish = False

        # Creates the lists to the home team and the adversary
        self.yellow_team = [Things() for _ in range(num_yellow_robots)]
        self.blue_team = [Things() for _ in range(num_blue_robots)]

        # Object to store ball info
        self.ball = Things()

        # seekers description
        self.seekers = {}

        if self.params_file_name != "":
            self.load_params()

        # Initialize the vitamins according to the chosen method
        if method == "color_segmentation":
            self.colors_params_file = colors_params
            self.load_colors_params()
            self.pipeline = self.color_seg_pipeline
            self.color_calibrator = ColorSegmentation(camera, self.colors_params_file)
        else:
            print("Method not recognized!")

        self.params_setter = ParamsSetter(camera, params_file_name)

        self.set_origin_and_factor()

        # Gets a initialization frame
        _, self.raw_image = camera.read()
        
        self.warp_perspective()

        # The hawk eye object will be responsible to locate and identify all
        # objects at the field
        self.load_colors_hawkeye()
        self.hawk_eye = HawkEye(self.origin, self.conversion_factor_x, self.conversion_factor_y,
                                self.seekers, self.num_yellow_robots, self.num_blue_robots,
                                self.arena_image.shape, self.hawk_eye_extra_params)

    def on_game_state_change(self, data):
        self.game_state = data.game_state
        if self.game_state:
            pass
            #TODO: Descomentar isso aqui
            # self.hawk_eye.reset()
            # self.reset_all_things()

    def toggle_calibration(self, new_value: bool = None) -> None:
        if new_value is not None:
            self.in_calibration_mode = new_value
        else:
            self.in_calibration_mode = not self.in_calibration_mode

    def reset_all_things(self):
        # Used when the game state changes to playing
        self.ball.reset()

        for i in range(len(self.yellow_team)):
            self.yellow_team[i].reset()

        for i in range(len(self.blue_team)):
            self.blue_team[i].reset()

    def start(self):
        self.game_on = True

    def pause(self):
        self.game_on = False

    def stop(self):
        self.pause()
        self.finish = True

    def update_fps(self):
        self.new_time = time.time()
        self.fps = 0.9*self.fps + 0.1 / (self.new_time - self.last_time)
        self.last_time = self.new_time

    def set_origin_and_factor(self):
        """ This function calculates de conversion factor between pixel to centimeters
            and finds the (0,0) pos of the field in the image """

        # for x
        x = np.sort(self.arena_vertices[:, 0])

        # xo is the third smaller vertice element because the first two ones are
        # the goal vertices
        xo = np.mean(x[2:6])

        y_sorted = np.sort(self.arena_vertices[:, 1])

        upper_y = np.mean(y_sorted[:2])

        # the yo origin is the most bottom vertice
        yo = np.mean(y_sorted[-2:])
        self.origin = np.array([xo, yo])

        rightmost_x = np.mean(x[10:14])

        # The height in pixels is the diff between yo and ymax
        height_px = abs(yo - upper_y)
        width_px = abs(xo - rightmost_x)

        # now just calculate the pixel to cm factor

        self.conversion_factor_x = 150.0 / width_px
        self.conversion_factor_y = 130.0 / height_px

    def load_params(self):
        """ Loads the warp matrix and the arena vertices from the arena parameters file"""
        params = JsonHandler.read(self.params_file_name)
        self.seekers = JsonHandler.read("parameters/game.json")["seekers"]
        self.arena_vertices = np.array(params['arena_vertices'])
        self.warp_matrix = np.asarray(params['warp_matrix']).astype("float32")
        self.arena_size = (params['arena_size'][0], params['arena_size'][1])

        self.create_mask()

    def load_colors_params(self):
        file1 = os.environ['ROS_ARARA_ROOT']+"src/" + self.colors_params_file
        if os.path.exists(file1):
            filename = file1
        elif os.path.exists(self.colors_params_file):
            filename = self.colors_params_file
        else:
            self._node.get_logger().fatal("Color params file load failed")
            return
        
        try:
            with open(filename, 'rb') as fp:
                self._colors_thresholds = pickle.load(fp)
                self.load_colors_hawkeye()
                try:
                    self.hawk_eye.reset(self.hawk_eye_extra_params)
                except AttributeError:
                    pass
        except Exception as exception:
            self._node.get_logger().fatal("Color params file load failed")
            self._node.get_logger().fatal(repr(exception))
        
    def load_colors_hawkeye(self) -> None:
        thrs = self._colors_thresholds
        self.hawk_eye_extra_params["ball"] = (thrs["orange"]["min"], 
                                              thrs["orange"]["max"])

        if "aruco" in self.seekers.values():
            self.hawk_eye_extra_params["aruco"] = (self.camera.camera_matrix, 
                                                   self.camera.dist_vector)
        if "color" in self.seekers.values():
            primary_colors = {"blue", "yellow", "orange"}
            secondary_colors = set(self._colors_thresholds.keys()) - primary_colors
            secondary_colors = sorted(list(secondary_colors))
            colors = []
            for color in secondary_colors:
                colors.append((thrs[color]["min"], thrs[color]["max"]))
            self.hawk_eye_extra_params["color"] = colors

    def warp_perspective(self):
        """ Takes the real world arena returned by camera and transforms it
            to a perfect retangle """
        # self._node.get_logger().fatal(repr(self.raw_image))
        self.arena_image = cv2.warpPerspective(self.raw_image, self.warp_matrix, self.arena_size)

    def create_mask(self):
        """ Creates the image where the mask will be stored """
        _arena_mask = np.zeros((self.arena_size[1], self.arena_size[0], 3), np.uint8)

        """ Here is drawn the arena shape with all pixles set to white color """
        cv2.fillConvexPoly(_arena_mask, np.asarray(self.arena_vertices), COLORS.WHITE)

        """ Gets the binary mask used in the bitwise operation of the
            set_dark_border function """
        self.arena_mask = cv2.inRange(_arena_mask, COLORS.WHITE, COLORS.WHITE)

    def set_dark_border(self):
        """ Applies a bitwise operation between the arena image and the arena mask
            to get rid of the pixels behind the goal lines"""
        self.arena_image = cv2.bitwise_and(self.arena_image, self.arena_image, mask=self.arena_mask)

    def get_filter(self, img, lower, upper):
        """ Returns a binary images where the white pixels corresponds to the pixels
        of the img that are between lower and upper threshold """
        temp_value_mask = cv2.inRange(img, np.array(lower), np.array(upper))
        return temp_value_mask

    def color_seg_pipeline(self):
        """ Wait until the color parameters are used """
        self.arena_image = cv2.cvtColor(self.arena_image, cv2.COLOR_BGR2HSV)

        thr = self._colors_thresholds
        self.blue_seg = self.get_filter(self.arena_image, 
                                        thr["blue"]["min"], 
                                        thr["blue"]["max"])

        self.yellow_seg = self.get_filter(self.arena_image, 
                                          thr["yellow"]["min"], 
                                          thr["yellow"]["max"])

    def run(self):
        while not self.finish:

            self.last_time = time.time()
            while self.game_on and not self.in_calibration_mode:
                _, self.raw_image = self.camera.read()
                self.warp_perspective()
                self.set_dark_border()
                self.pipeline()

                self.hawk_eye.seek_yellow_team(self.yellow_seg, 
                                               self.yellow_team, 
                                               self.hawk_eye.yellow_team_seeker,
                                               opt=self.arena_image)
                
                self.hawk_eye.seek_blue_team(self.blue_seg,
                                             self.blue_team,
                                             self.hawk_eye.blue_team_seeker,
                                             opt=self.arena_image)

                self.hawk_eye.seek_ball(self.arena_image, self.ball)

                self.send_message(ball=True, yellow_team=True, blue_team=True)
                self.update_fps()

            if self.in_calibration_mode:
                sleep(0.016)

        self.camera.stop()
        self.camera.capture.release()

    def unpack_things_to_lists(self, things, positions_list, orientations_list):
        """ Auxiliary  function created to not duplify code in the send_message
            function"""
        for thing in things:
            # The id of the thing will be its index in the lists
            id = thing.id
            if id >= 0:
                positions_list[id] = thing.pos
                orientations_list[id] = thing.orientation

    def send_message(self, ball: bool = False,
                     yellow_team: bool = False,
                     blue_team: bool = False) -> None:
        """ This function will return the message in the right format to be
            published in the ROS vision bus """

        if ball:
            self.unpack_things_to_lists([self.ball], self.ball_pos, [[]])

        if yellow_team:
            self.unpack_things_to_lists(self.yellow_team, self.yellow_team_pos,
                                        self.yellow_team_orientation)

        if blue_team:
            self.unpack_things_to_lists(self.blue_team, self.blue_team_pos,
                                        self.blue_team_orientation)

        self.mercury.publish(self.ball_pos[0], self.yellow_team_pos,
                             self.yellow_team_orientation, self.blue_team_pos,
                             self.blue_team_orientation, self.fps)


if __name__ == "__main__":
    from threading import Thread

    num_yellow_robots = 1
    num_blue_robots = 1
    home_tag = "aruco"

    arena_params = "../parameters/ARENA.json"
    colors_params = "../parameters/COLORS.bin"
    camera = Camera(sys.argv[1], "../parameters/CAMERA_ELP-USBFHD01M-SFV.json", threading=True)

    v = Vision(camera, num_blue_robots, num_yellow_robots, arena_params,
               colors_params, method="color_segmentation")

    v.game_on = True

    t = Thread(target=v.run, args=())
    t.daemon = True
    t.start()

    i = 0
    last_time = time.time()
    cv2.namedWindow('control')
    show = False

    while True:
        arena = v.arena_image
        key = cv2.waitKey(1) & 0xFF
        if show:
            cv2.imshow('vision', cv2.cvtColor(arena, cv2.COLOR_HSV2BGR))
            cv2.imshow('segs', np.hstack([v.blue_seg, v.yellow_seg, v.ball_seg]))
        if key == ord('q'):  # exit
            v.pause()
            v.finish = True
            break
        elif key == ord('s'):  # show/hide
            show = not show
            if show == False:
                cv2.destroyWindow("vision")
                cv2.destroyWindow("segs")
        elif key == ord('c'):  # open cropper
            v.params_setter.run()
            v.load_params()
        elif key == ord('g'):  # "get color"
            v.color_calibrator.run()
            v.load_colors_params()
        elif key == ord('f'):
            print(v.fps, "frames/second")

    cv2.destroyAllWindows()
