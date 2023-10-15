import numpy as np
import cv2
import time
import os
from argparse import ArgumentParser

from vision.camera_module.camera import Camera
from vision import COLORS
from utils.yaml_handler import YamlHandler

class ParamsSetter:

    def __init__(self, cam, params_file):
        # modes definitions
        self.NO_MODE     = -1
        self.ADD_MODE    = 0
        self.DELETE_MODE = 1
        self.EDIT_MODE   = 2

        self.RADIUS             =   3
        self.BAR_HEIGHT         = 50
        self.MOUSE_LAST_STATE   = None

        self.cam = cam
        self.params_file = params_file

        self.value_min = None
        self.matrix_transform = None
        self.arena_size = None
        self.vertices_points = []

        self.img_width = None
        self.img_height = None
        self.frame = None
        self.mouse_last_pos = None

    def get_status_bar(self, h, w, status):
        font = cv2.FONT_HERSHEY_SIMPLEX
        status_bar = np.zeros((h, w, 3), np.uint8)

        if status == self.ADD_MODE:
            text = "ADD POINT MODE"
            color = COLORS.GREEN
        elif status == self.DELETE_MODE:
            text = "DELETE POINT MODE"
            color = COLORS.RED
        elif status == self.EDIT_MODE:
            text = "EDIT POINT MODE"
            color = COLORS.YELLOW
        elif status == self.NO_MODE:
            text = "NO MODE SELECTED"
            color = COLORS.BLUE

        status_bar[:][:] = color
        return cv2.putText(status_bar, text, (0,30), font, 1, (0, 0, 0), 2, cv2.LINE_AA)

    def inside_circle(self, points, pos, radius):
        i = 0
        for point in points:
            d = np.asarray(point) - np.asarray(pos)
            if np.linalg.norm(d) <= radius:
                return i
            i += 1
        return None

    def draw_components(self, img, pts, sort=True):
        if len(pts):
            if sort:
                sorted_points = self.sort_clockwise(pts)
            else:
                sorted_points = pts

            for (i,pt) in enumerate(sorted_points):
                cv2.circle(img, tuple(pt), self.RADIUS, COLORS.RED, -1)
                if i != 0:
                    segment = np.array([sorted_points[j], sorted_points[i]]).reshape((-1, 1, 2))
                    cv2.polylines(img, [segment], True, COLORS.GREEN, 2)
                j = i

            if len(pts):
                segment = np.array([sorted_points[j], sorted_points[0]]).reshape((-1, 1, 2))
                cv2.polylines(img, [segment], True, COLORS.GREEN, 2)

    def draw_guide_lines(self, img):
        if self.mouse_last_pos != None:
            x,y = self.mouse_last_pos
            cv2.line(img, (x, 0), (x, self.img_width - 1), COLORS.GREEN, 1)
            # horizontal line
            cv2.line(img,  (0, y), (self.img_height - 1, y), COLORS.GREEN, 1)

    def onMouse_add_mode(self, event, x, y, flags, pts):
        if event == cv2.EVENT_LBUTTONUP:
            yt = y-self.BAR_HEIGHT
            pts.append((x, yt))

        self.mouse_last_pos = (x,y-self.BAR_HEIGHT)

    def onMouse_delete_mode(self, event, x, y, flags, pts):
        i = self.inside_circle(pts, (x,y-self.BAR_HEIGHT), self.RADIUS)
        if event == cv2.EVENT_LBUTTONUP:
            if i != None:
                del pts[i]

    def onMouse_edit_mode(self, event, x, y, flags, pts):
        i = self.inside_circle(pts, (x,y-self.BAR_HEIGHT), 10*self.RADIUS)

        if event == cv2.EVENT_LBUTTONDOWN:
            self.MOUSE_LAST_STATE = cv2.EVENT_LBUTTONDOWN
        elif event == cv2.EVENT_LBUTTONUP:
            self.MOUSE_LAST_STATE = cv2.EVENT_LBUTTONUP

        if event == cv2.EVENT_MOUSEMOVE and self.MOUSE_LAST_STATE == cv2.EVENT_LBUTTONDOWN and i != None:
            pts[i] = (x,y-self.BAR_HEIGHT)

    def onMouse_no_mode(self, event, x, y, flags, pts):
        print(x,y)
        pass

    def sort_clockwise(self, pts):
        n = len(pts)
        if n > 1:
            points = np.asarray(pts).reshape(n, 2)

            points[:, 1:] = -points[:, 1:]
            tl_index = np.argsort(np.linalg.norm(points, axis=1))[0]
            tl = points[tl_index]
            d = points - tl
            thetas = np.arctan2(d[:, 1:], d[:, :1]).reshape(n)
            thetas[tl_index] = 10 # this way top left will always be the first
            order =  np.argsort(-thetas)

            points[:, 1:] = -points[:, 1:]
            return points[order, ]
        return pts

    def get_matrix_transform(self, pts):
        points = self.sort_clockwise(pts)
        (tl, tr, br, bl) = points

        width_a = np.sqrt( (tl[0] - tr[0])**2 + (tl[1] - tr[1])**2 )
        width_b = np.sqrt( (bl[0] - br[0])**2 + (bl[1] - br[1])**2 )
        final_width = max(int(width_a), int(width_b)) - 1

        height_a = np.sqrt( (tl[0] - bl[0])**2 + (tl[1] - bl[1])**2 )
        height_b = np.sqrt( (tr[0] - br[0])**2 + (tr[1] - br[1])**2 )
        final_height = max(int(height_a), int(height_b)) - 1

        dst = np.array([
            [0,0],
            [final_width, 0],
            [final_width, final_height],
            [0, final_height]], dtype="float32")

        self.matrix_transform = cv2.getPerspectiveTransform(points.astype("float32"), dst)
        self.arena_size = (final_width, final_height)

    def nothing(self, x):
        pass

    def valueAdjust(self, imageSrc):
        # Creates the window to see de threshold applied
        window_name = 'Cropper Value Adjust'
        cv2.namedWindow(window_name)

        # Creates the trackbar
        cv2.createTrackbar('minV', window_name, 0, 255, self.nothing)

        LAB_MAX = np.uint8([255, 255, 255])
        lab_min = np.uint8([0, 0, 0])

        while True:
            # get the value
            v_threshold = cv2.getTrackbarPos('minV', window_name)
            lab_min[0] = v_threshold

            _image = cv2.cvtColor(imageSrc, cv2.COLOR_BGR2LAB)
            thsv = cv2.inRange(_image, lab_min, LAB_MAX)
            _image = cv2.bitwise_and(_image, _image , mask=thsv)
            thImage = cv2.cvtColor(_image, cv2.COLOR_LAB2BGR)

            # the red pixels represent the pixels tath passed throw the threshold
            (h, w) = thImage.shape[:2]
            thImage = thImage.reshape((thImage.shape[0] * thImage.shape[1], 3))
            _index = np.where(np.all(thImage != (0,0,0), axis=1) == True)
            thImage[_index] = (0,0,255)
            thImage = thImage.reshape((h, w, 3))
            # Show the result
            cv2.imshow(window_name, thImage)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s') or key == ord('q'):
                    break

        cv2.destroyWindow(window_name)
        if key == ord('q'):
            return False
        else:
            self.value_min = np.array([v_threshold,0,0])
            return True

    def save_params(self):
        yaml_handler = YamlHandler()

        if(self.params_file == ""):
            self.params_file = input("Please insert params file name: ")

        params = yaml_handler.read(self.params_file)

        if not (self.matrix_transform is None) and not (self.arena_size is None):
            params['warp_matrix'] = self.matrix_transform.tolist()
            params['arena_size'] = self.arena_size
        if self.vertices_points != []:
            params['arena_vertices'] = self.vertices_points
        if not(self.value_min is None):
            params['value_min'] = self.value_min.tolist()

        return yaml_handler.write(params, self.params_file)

    def print_manual(self):
        print("-------------------------------------------")
        print("A to enter Add point mode")
        print("E to enter Edit point mode")
        print("D to enter Delete point mode")
        print("ESC to exit mode selection")
        print("S to save")
        print("Q to quit")
        print("--------------------------------------------\n")

    def run(self):
        self.print_manual()

        cv2.namedWindow('cropper')
        mode = self.NO_MODE
        warped = False
        arena_countour = False
        value_adjusted = False
        value_saved = False

        _, self.frame = self.cam.read()
        self.img_width, self.img_height = self.frame.shape[:2]
        points = []

        while True:
            _, self.frame = self.cam.read()

            if warped:
                self.frame = cv2.warpPerspective(self.frame, self.matrix_transform, self.arena_size)

            h,w = self.frame.shape[:2]
            status_bar = self.get_status_bar(self.BAR_HEIGHT, w, mode)

            if not warped:
                self.sort_clockwise(points)
                self.draw_components(self.frame, points)
            else:
                self.draw_components(self.frame, points, sort=False)

            if mode == self.ADD_MODE or mode == self.EDIT_MODE:
                self.draw_guide_lines(self.frame)

            cv2.imshow('cropper', np.vstack([status_bar, self.frame]))


            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                mode = self.ADD_MODE
                break
            elif key == ord('a'):
                mode = self.ADD_MODE
                cv2.setMouseCallback('cropper', self.onMouse_add_mode, points)
            elif key == ord('d'):
                mode = self.DELETE_MODE
                cv2.setMouseCallback('cropper', self.onMouse_delete_mode, points)
            elif key == ord('e'):
                mode = self.EDIT_MODE
                cv2.setMouseCallback('cropper', self.onMouse_edit_mode, points)
            elif key == ord('v'):
                value_adjusted = self.valueAdjust(self.frame)
            elif key == 27:
                mode = self.NO_MODE
                cv2.setMouseCallback('cropper', self.onMouse_no_mode, points)
            elif key == ord('s') or (value_adjusted == True and value_saved == False):
                if value_adjusted == True and value_saved == False:
                    self.save_params()
                    value_saved = True
                    print("Everything saved!")
                else:
                    if warped == True:
                        self.vertices_points = points
                        self.save_params()
                        print("Warp matrix and vertices saved!")
                    else:
                        if len(points) == 4:
                            warped = True
                            print("Calculating prespective transform")
                            self.get_matrix_transform(points)
                            points = []
                            mode = self.NO_MODE
                            print("Done!")
                        else:
                            print("Select four points")

        cv2.destroyWindow('cropper')
        cv2.destroyAllWindows()

def makeArgParser() -> ArgumentParser:
    parser = ArgumentParser()
    parser.add_argument("device", type=str, default="0", help="camera device")
    parser.add_argument("--camera_params_file", 
                         type=str, 
                         default="../../parameters/CAMERA_ELP-USBFHD01M-SFV.yml",
                         help="camera params file for lens correction")
    parser.add_argument("--arena_params_file",
                        type=str,
                        default="../../parameters/ARENA.yml",
                        help="color params file to store the color thresholds")
    return parser


if __name__ == '__main__':
    parser = makeArgParser()
    args = parser.parse_args()

    cam = Camera(args.device, args.camera_params_file)
    setter = ParamsSetter(cam, args.arena_params_file)
    setter.run()
