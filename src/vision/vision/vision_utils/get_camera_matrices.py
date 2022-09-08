from argparse import ArgumentParser
import numpy as np
import glob
import cv2
import time
import sys
import pickle
import os

from vision.camera_module.camera import Camera

# @authors: Wellington Castro
# Last modification: 16/10/2019

def make_parser() -> ArgumentParser:
    parser = ArgumentParser()

    parser.add_argument("camera", type=str, help="camera device for \
                                                  example: /dev/video0")
    parser.add_argument("frame_width", type=int, help="frame width in pixels")
    parser.add_argument("frame_height", type=int, help="frame height in pixels")
    
    parser.add_argument("h_centers", type=int, help="number of intersections \
        on the chessboard horizontal axis")
    parser.add_argument("v_centers", type=int, help="number of intersections \
        on the chessboard vertical axis ")
    
    parser.add_argument("--scale", type=float, default=1.0, help="scale factor\
         of the output image in relation of the original image")
    
    parser.add_argument("--camera_name", type=str, default="", help="camera name to save\
        the calibration matrices")
    
    return parser


def get_error(objp, imgpoints, rvecs, tvecs, mtx, dist):
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    return mean_error/len(objpoints)

def save_params(filename: str, params:dict) -> None:
    file1 = os.environ['ROS_ARARA_ROOT']+"src/" + filename
    if os.path.exists(file1):
        filename = file1
    elif not os.path.exists(filename):
        return
    
    with open(filename, "rb+") as fp:
        pickle.dump(params, fp)
        

if __name__ == '__main__':
    parser = make_parser()
    args = parser.parse_args()
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((args.h_centers * args.v_centers,3), np.float32)
    objp[:,:2] = np.mgrid[0:args.h_centers, 0:args.v_centers].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    images = glob.glob('*.jpg')

    for fname in images:
        frame = cv2.imread(fname)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Find the chessboard centers
        ret, corners = cv2.findChessboardCorners(gray, (args.h_centers, args.v_centers), None)

        if ret:
            objpoints.append(objp)
            cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)
            cv2.drawChessboardCorners(frame, (args.h_centers, args.v_centers), corners,ret)

        cv2.imshow('window', frame)
        cv2.waitKey(100)


    cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    h, w =  frame.shape[:2]
    nh, nw = int(args.scale * h), int(args.scale * w)
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (nw,nh))

    # get undistorion maps
    mapx,mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (nw,nh), 5)

    e = get_error(objp, imgpoints, rvecs, tvecs, mtx, dist)
    print("STD Error:", e)

    save = input("Save camera matrices? (y/n): ")
    print(save)

    if save == 'y':
        params = {}
        params['matrix_x'] = mapx
        params['matrix_y'] = mapy
        params['cam_matrix'] = mtx
        params['dist_vector'] = dist
        params['default_frame_width'] = args.frame_width
        params['default_frame_height'] = args.frame_height
        
        if args.camera_name == "":
            name = input("Please insert camera name (default is ELP-USBFHD01M-SFV): ")
            if name == "":
                name = "ELP-USBFHD01M-SFV"
        else:
            name = args.camera_name

        file_name = "../../parameters/CAMERA_" + name + ".bin"
        save_params(file_name, params)
        

    print("Showing result, press q to exit")
    if save == 'y':
        cap = Camera(args.camera, "../../parameters/CAMERA_" + name + ".bin")
    else:
        cap = Camera(args.camera, "", False, False)

    while(True):
        u_frame = cap.read()
        if save != 'y':
            u_frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

        cv2.imshow('Undistorted', u_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cv2.destroyAllWindows()

    delete = input("Delete all images (y/n)): ")
    if delete == 'y':
        os.system("rm *.jpg")
        print("all .jpg files removed!")
        
    
