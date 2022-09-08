#!/usr/bin/python3
from platform import node
import rclpy
from rclpy.node import Node
import sys
import cv2
from time import time
from enum import Enum
from threading import Thread

from vision.camera_module.camera import Camera
from vision.vision import Vision
from vision.ros_vision_publisher import RosVisionService
from utils.model import Model
from utils.camera_loader import CameraLoader


class VisionOperations(Enum):
    """
    This class stores vision operations for service request
    """
    SHOW = 1
    CROPPER = 2
    COLOR_CALIBRATION = 3


class VisionNode(Node):
    """
    A node for spinning the Vision
    """

    def __init__(self, vision_owner: str = 'Player_One'):
        """
        :param color: int
        """
        super().__init__('vision', namespace=vision_owner)
        self.team_colors = ['blue', 'yellow']
        self.yellow_robots = 5
        self.blue_robots = 5
        self.show = False
        self.state_changed = 0

        arena_params = "parameters/ARENA.json"
        colors_params = "parameters/COLORS.bin"

        try:
            device = int(sys.argv[1])
        except ValueError:
            device = sys.argv[1]
        except IndexError:
            model = Model()
            _, device = CameraLoader(model.game_opt['camera']).get_index()

        self.camera = Camera(self, device, "parameters/CAMERA_ELP-USBFHD01M-SFV.bin", threading=False)

        self.vision = Vision(self, self.camera, self.blue_robots, self.yellow_robots,
                             arena_params, colors_params, method="color_segmentation", vision_owner=vision_owner)
        self.vision.game_on = True

        self.thread = Thread(target=self.vision.run, args=())
        self.thread.daemon = True
        self.thread.start()

        # Creates the service responsible for vision modes and operations
        self.service = RosVisionService(self, self.vision_management)

        self.timer = self.create_timer(1, self.tick)

    def create_new_timer(self, new_timer_period: int = 1):
        self.timer.destroy()
        self.timer = self.create_timer(new_timer_period, self.tick)

    def vision_management(self, request, response) -> int:
        """
        This is the reading function for a service response
        :param req: variable to get the request operation
        :return: bool
        """
        success = True
        self.state_changed = request.operation
        response.success = success
        return response

    def tick(self):
        if self.show:
            cv2.imshow('vision', cv2.cvtColor(self.vision.arena_image, cv2.COLOR_HSV2BGR))
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.create_new_timer(1)  # 1hz
                self.show = not self.show
                cv2.destroyAllWindows()
                self.vision.computed_frames = 0
                self.vision.t0 = time()
        if self.state_changed:  # Process requisition
            if self.state_changed == VisionOperations.SHOW.value:
                self.create_new_timer(1/60)  # 60hz
                self.show = True  # not self.show

            elif self.state_changed == VisionOperations.CROPPER.value:
                self.vision.toggle_calibration(True)
                self.vision.params_setter.run()
                self.vision.load_params()
                self.vision.toggle_calibration(False)

            elif self.state_changed == VisionOperations.COLOR_CALIBRATION.value:
                # This will verify if the color segmentation technique is
                # the chosen one
                if self.vision.colors_params_file != "":
                    # if it is, execute the color calibrator
                    self.vision.toggle_calibration(True)
                    self.vision.color_calibrator.run()
                    self.vision.load_colors_params()
                    self.vision.toggle_calibration(False)

            self.state_changed = 0

def main(args=None):
    rclpy.init(args=args)

    try:
        owner_name = sys.argv[2]
    except IndexError:
        owner_name = 1

    vision_node = VisionNode()
    
    # while rclpy.ok():
    #     vision_node.tick()

    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except BaseException:
        print('exception in server')
        raise
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        vision_node.vision.stop()
        cv2.destroyAllWindows()
        
        vision_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()