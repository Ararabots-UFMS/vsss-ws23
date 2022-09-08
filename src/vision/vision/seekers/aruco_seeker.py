import numpy as np
import cv2
import cv2.aruco as aruco
import math

from vision.seekers.seeker import Seeker

# @author Wellington Castro <wvmcastro>

class ArucoSeeker(Seeker):

    def __init__(self, cam_mtx, dist_vec, num_tags, num_bits=3, num_markers=5):
        """ Initializes the objects necessary to perform the detection
            of the aruco tags """

        # Hyper params to create the aruco markers dictionary
        # self.num_markers = num_markers
        self.num_markers = 68
        self.num_bits = num_bits

        # This variable will keep how many tags must be identified
        self.num_tags = num_tags

        # Creates the Aruco dictionary according to the number of bits and
        # number of desired markers
        self.aruco_dict = aruco.Dictionary_create(self.num_markers, self.num_bits)
        self.aruco_params = aruco.DetectorParameters_create()

        # Important info from the camera needed by the ArucoDetectMarkers
        self.camera_matrix = cam_mtx
        self.distortion_vector = dist_vec

    def get_aruco_state(self, img, rvec, tvec, degree=False):
        # x axis
        x_axis = np.float32([[0,0,0],[1.0,0,0]]).reshape(-1,3)

        # Do not know what is jac (sorry again). Imgpts are the start and end points
        # of each vector of the base
        imgpts, jac = cv2.projectPoints(x_axis, rvec, tvec, self.camera_matrix, self.distortion_vector)

        # takes the tail and nose of the orientation vector
        t, n = imgpts[0][0], imgpts[1][0]

        # calculates the orientation vector
        orientation_vec = (n-t)
        orientation_vec[1] *= -1
        orientation_angle = math.atan2(orientation_vec[1], orientation_vec[0])

        if degree == True:
            orientation_angle *= self.rad_to_degree_factor

        # returns the center of the aruco marker and its x axis orientation
        return t, orientation_angle

    def seek(self, img, degree=False):

        # Try to locate all markers in the img
        corners, ids, _ = aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_params)

        identified_markers = []

        if np.any(ids != None):
            # That means at least one Aruco marker was recognized

            # Reshapes the ids matrix to an ids vector for indexing simplicity
            ids = ids.reshape(ids.shape[0] * ids.shape[1])

            # Sort the ids vector, that way the same marker will be always in the
            # same pos in the things_list
            sorted_ids = np.sort(ids)

            i = 0
            # rospy.logfatal('ids.size: '+str(ids.size))
            # rospy.logfatal('sorted[i]: ' + str(sorted_ids[i]))

            while(i < ids.size and i < self.num_tags):

                # Finds the original index of the marker before sorting
                index = np.argwhere(ids == sorted_ids[i])[0,0]

                # Estimate the marker's pose
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[ index ], 0.075,
                                self.camera_matrix, self.distortion_vector)

                # Gets the marker state, ie: its center and x axis orientation
                center, orientation = self.get_aruco_state(img, rvec, tvec, degree)

                identified_markers.append([sorted_ids[i], center, orientation])

                i += 1

        return identified_markers

    def reset(self, opt=None):
        # In the case fo the aruco seeker this function is unecessary
        # But was left here to mantain the minimum structure of a seeker
        # all seekers must have a reset method
        pass
