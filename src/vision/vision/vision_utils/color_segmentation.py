#!/usr/bin/python3
import cv2
import numpy as np
import copy
import os
import pickle
import os

from argparse import ArgumentParser
from utils.json_handler import JsonHandler
from vision.camera_module.camera import Camera
from vision import COLORS


# @author Wellington Castro <wvmcastro>

class ColorSegmentation:

    def __init__(self, cam, color_params_file=""):
        self.json_handler = JsonHandler()
        self.params_file = color_params_file
        self.camera = cam

        self._shortcuts = {'o': "orange", 'y': "yellow", 
                           'b': "blue", 'g': "green", 'p': "pink", 
                           'i': "ice", 'v': "violet"}

        self._colors = self._shortcuts.values()
        self._threshs = {c : {"min": np.uint8((255, 255, 255)), "max": np.uint8((0, 0, 0))} 
                            for c in self._colors}
        
        self.last_key = None
        self.reset_all()
        self.load_params()

        self.temp_min = np.uint8([])
        self.temp_max = np.uint8([])

    def reset_all(self):
        for c in self._colors:
            self.reset(c)
        
        self.reset("temp")

    def reset(self, component):
        if component != "temp":
            self._threshs[component]["min"] = np.uint8([255, 255, 255])
            self._threshs[component]["max"] = np.uint8([0, 0, 0])
        else:
            self.temp_min = np.uint8([])
            self.temp_max = np.uint8([])

    def load_params(self):
        file1 = os.environ['ROS_ARARA_ROOT']+"src/" + self.params_file
        if os.path.exists(file1):
            filename = file1
        elif os.path.exists(self.params_file):
            filename = self.params_file
        else:
            return
        
        try:
            print(filename)
            with open(filename, 'rb') as fp:
                self._threshs = pickle.load(fp)
        except:
            print("File load failed")

    def write_params(self):
        if self.params_file[0] != '/':
            filename = os.environ['ROS_ARARA_ROOT'] + "src/" + self.params_file
        else:
            filename = self.params_file

        with open(filename, "wb+") as fp:
            pickle.dump(self._threshs, fp)

    def onMouse_get_color(self, event, x, y, flags, arg):

        if self.temp_min.size > 0 and self.temp_max.size > 0:

            if event == cv2.EVENT_LBUTTONUP and (flags & cv2.EVENT_FLAG_SHIFTKEY):
                self.temp_min = np.minimum(self.temp_min, self.hsv_frame[y, x, :])
                self.temp_max = np.maximum(self.temp_max, self.hsv_frame[y, x, :])
                self.visited.append([y, x])
            elif event == cv2.EVENT_LBUTTONUP:
                for i in range(y - 1, y + 2):
                    for j in range(x - 1, x + 2):
                        self.temp_min = np.minimum(self.temp_min, self.hsv_frame[i, j, :])
                        self.temp_max = np.maximum(self.temp_max, self.hsv_frame[i, j, :])
                        self.visited.append([i, j])

            c = self._shortcuts[self.last_key]
            self._threshs[c]["min"] = self.temp_min
            self._threshs[c]["max"] = self.temp_max

    def onMouse_no_mode(self, event, x, y, flags, arg):
        pass

    def draw_selected(self, mask):
        pos = np.argwhere(mask == 255)
        self.frame[pos[:, 0], pos[:, 1], :] = COLORS.RED

    def draw_mask(self):
        mask = cv2.inRange(self.hsv_frame, self.temp_min, self.temp_max)
        return mask

    def erase_previous(self):
        color = self._shortcuts[self.last_key]
        self.reset(color)
        self.reset("temp")

    def run(self):
        window_name = "color segmentation"
        cv2.namedWindow(window_name)

        should_exit = False
        self.visited = []
        _, frame = self.camera.read()
        aux_mask = np.empty(frame.shape).astype("uint8")

        while (not should_exit):
            _, self.frame = self.camera.read()
            self.hsv_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

            key = cv2.waitKey(1) & 0xFF

            if self.temp_min.size > 0 and self.temp_max.size > 0:
                self.mask = self.draw_mask()
                aux_mask[:, :, 0], aux_mask[:, :, 1], aux_mask[:, :, 2] = self.mask, self.mask, self.mask
                self.draw_selected(self.mask)
                cv2.imshow("Segment", aux_mask)
            else:
                cv2.destroyWindow("Segment")

            cv2.imshow(window_name, self.frame)

            if key == ord('q'):
                should_exit = True
            elif chr(key) in self._shortcuts:  # orange
                c = self._shortcuts[chr(key)]
                
                if c not in self._threshs.keys():
                    self._threshs[c] = {"min": np.uint8((255,255,255)),
                                        "max": np.uint8((0,0,0))}

                self.temp_min = self._threshs[c]["min"]
                self.temp_max = self._threshs[c]["max"]
                self.last_key = chr(key)
                self.visited = []
                cv2.setMouseCallback(window_name, self.onMouse_get_color)
            elif key == ord(' '):
                print("LAST KEY", self.last_key)
                self.erase_previous()
                self.visited = []
            elif key == ord('s'):
                self.write_params()
                print("saving")
            elif key == 27:  # no mode
                self.reset("temp")
                cv2.setMouseCallback(window_name, self.onMouse_no_mode)
                self.visited = []
                self.last_key = None

        cv2.destroyAllWindows()


def makeArgParser() -> ArgumentParser:
    parser = ArgumentParser()
    parser.add_argument("device", type=str, default="0", help="camera device")
    parser.add_argument("--camera_params_file",
                        type=str,
                        default="../../parameters/CAMERA_ELP-USBFHD01M-SFV.json",
                        help="camera params file for lens correction")
    parser.add_argument("--color_params_file",
                        type=str,
                        default="../../parameters/COLORS.bin",
                        help="color params file to store the color thresholds")
    return parser


if __name__ == "__main__":
    parser = makeArgParser()
    args = parser.parse_args()
    
    cam_id = int(args.device) if len(args.device) == 0 else args.device
    camera = Camera(cam_id, args.camera_params_file)
    c = ColorSegmentation(camera, args.color_params_file)
    c.run()
