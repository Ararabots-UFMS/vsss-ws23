from argparse import ArgumentParser
import time
import numpy as np
import uuid
import cv2
import sys

from vision.camera_module.camera import Camera


def make_parser() -> ArgumentParser:
    parser = ArgumentParser()

    parser.add_argument("camera", type=str, help="camera device for \
                                                  example: /dev/video0")
    parser.add_argument("frame_width", type=int, help="frame width in pixels")
    parser.add_argument("frame_height", type=int, help="frame height in pixels")

    return parser

if __name__ == '__main__':
    parser = make_parser()
    args = parser.parse_args()

    cap = Camera(args.camera, "", False)
    cap.set_device(args.frame_width, args.frame_height)

    i = 0
    while(True):
        _, frame = cap.read()
        cv2.imshow("frame", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):
            name = uuid.uuid4().hex + ".jpg"
            cv2.imwrite(name, frame)
            print("Frame %03d saved" % (i))
            i += 1
        elif key == ord('q'):
            print("Exiting")
            break

cv2.destroyAllWindows()
