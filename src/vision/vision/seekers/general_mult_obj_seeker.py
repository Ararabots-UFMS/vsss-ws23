import numpy as np
import cv2
from sklearn.cluster import KMeans
import time

from vision.seekers.seeker import Seeker


# @author Wellington Castro <wvmcastro>

class GeneralMultObjSeeker(Seeker):

    def __init__(self, num_objects):
        self.num_objects = num_objects
        self.kmeans = KMeans(n_clusters=self.num_objects, n_init=1, max_iter=30)
        self.objects = None

    def seek(self, img):
        """
            This function receives a binary image with objects and return
            its centers positions
            param img : np.array([uint8]).shape([m,n])
            img is a binary image of a team
           :return: objecs: np.array([float, float]).shape([k, 2])
           object has the position of the center of each object in img
        """
        if cv2.__version__[0] == '4':
            cnts, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        else:
            _, cnts, *_ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        centroids_list = []
        num_cnts = len(cnts)
        if num_cnts > 0:
            cnts_array = np.array(cnts[0]).reshape(-1, 2)

            for i in range(1, num_cnts):
                cnts_array = np.vstack([cnts_array, np.array(cnts[i]).reshape(-1, 2)])

            if cnts_array.shape[0] > self.num_objects:
                first_iteration = 1
                if np.all(self.objects != None):
                    first_iteration = 0
                    self.kmeans.init = self.objects

                self.kmeans.fit(cnts_array)
                newObjects= self.kmeans.cluster_centers_

                if not first_iteration and np.all(self.objects != None):
                    diff = newObjects - self.objects
                    distances = np.linalg.norm(diff, axis=1)
                    changes = np.where(distances > 2.5)[0]
                    self.objects[changes,:] = self.kmeans.cluster_centers_[changes,:]
                else:
                    self.objects = newObjects

        return self.objects

    def reset(self, opt=None):
        self.kmeans.init = 'k-means++'
        self.objects = None
