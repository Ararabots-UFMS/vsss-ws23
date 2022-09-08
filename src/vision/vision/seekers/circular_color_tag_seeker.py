from typing import List, Tuple
import cv2
import numpy as np
import math

from vision.seekers.seeker import Seeker

IMAGE = np.ndarray
ROBOT_STATE = Tuple[int, np.array, float]

class CircularColorTagSeeker(Seeker):
    def __init__(self, color_thresholds: List[Tuple[np.ndarray, np.ndarray]]):
        self._colors = color_thresholds
        
        self._radius_thresh = -1
        self._l_thresh = 0
        self._r_thresh = 0
        self._h = 0
        self._w = 0

        self._theta = math.pi / 4
    
    def seek(self, binary_img: IMAGE, color_img: IMAGE):
        if self._h == 0:
            self._h, self._w, *_ = binary_img.shape

        first_centroids = self.get_main_color_centroids(binary_img)
        slices = self.get_crop_areas(first_centroids)

        patches = []
        top_lefts = []
        for k, s in enumerate(slices):
            patches.append((k, color_img[s[0], s[1], ...]))
            top_lefts.append(np.array([s[1].start, s[0].start]))
        
        ids, second_centroids = self.segment_and_get_second_centroids(patches, top_lefts)
        robots = self.compute_robot_states(ids, first_centroids, second_centroids)
        robots.sort(key = lambda r: r[0])
        return robots
    
    def get_main_color_centroids(self, img) -> List[np.ndarray]:
        cnts = self.get_contours(img)
        
        centroids = []
        # compute all areas and centroids
        for cnt in cnts:
            m = cv2.moments(cnt)
            a = m["m00"]
            if a != 0:    
                cx = m["m10"] / a
                cy = m["m01"] / a
                centroids.append((a, np.array([cx, cy])))
        
        centroids.sort(key = lambda x: x[0], reverse=True)

        if len(centroids) < 2:
            n = len(centroids)
        else:
            self.l_thresh = 0.7 * centroids[0][0]
            self.r_thresh = 1.3 * centroids[0][0]

            n = 1
            for c in centroids[1:]:
                if self.l_thresh <= c[0] <= self.r_thresh:
                    n += 1
        
        if self._radius_thresh < 0 and n != 0:
            *_, radius = cv2.minEnclosingCircle(cnts[0])
            self._radius_thresh = 2 * radius

        return [centroids[i][1] for i in range (n)]
    
    def get_contours(self, img: IMAGE):
        if cv2.__version__[0] == '4':
            cnts, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        else:
            _, cnts, *_ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        return cnts

    def get_crop_areas(self, centroids) -> List[Tuple[slice, slice]]:
        
        slices = []
        for centroid in centroids:
            x_min = int(max(0, centroid[0] - self._radius_thresh))
            x_max = int(min(self._w, centroid[0] + self._radius_thresh))
            y_min = int(max(0, centroid[1] - self._radius_thresh))
            y_max = int(min(self._h, centroid[1] + self._radius_thresh))

            slices.append((slice(y_min, y_max, 1), slice(x_min, x_max, 1)))
        
        return slices

    def segment_and_get_second_centroids(self, patches: List[Tuple[int, IMAGE]],
                                               top_lefts: List[np.ndarray]) -> \
                                    Tuple[List[int], List[np.ndarray]]:
        n = len(patches)
        centroids = [None] * n
        ids = [None] * n
        for i, color in enumerate(self._colors):
            for j, (k, patch) in enumerate(patches):
                thresholded = cv2.inRange(patch, color[0], color[1])
                if np.any(thresholded):
                    cnts = self.get_contours(thresholded)
                    cnt = cnts[0]
                    m = cv2.moments(cnt)
                    a = m["m00"]
                    if a != 0:
                        cx = m["m10"] / a
                        cy = m["m01"] / a
                        centroids[k] = top_lefts[j] + np.array([cx, cy])
                        ids[k] = i
                        del patches[j]
                        del top_lefts[j]
                        break
        
        return ids, centroids
    
    def compute_robot_states(self, ids: List[int], 
                             first_centroids: List[np.ndarray], 
                             second_centroids: List[np.ndarray]) -> \
                             List[ROBOT_STATE]:
        robots = []
        i = 0
        for c1, c2 in zip(first_centroids, second_centroids):
            if c2 is not None:
                c = (c1 + c2) / 2
                vec = c2 - c1
                angle = math.atan2(-vec[1], vec[0])  - self._theta
                robots.append((ids[i], c, angle))

            i += 1
        return robots
    
    def reset(self, opt = None) -> None:
        if opt is not None:
            self._colors = opt

        self._radius_thresh = -1
        self._l_thresh = 0
        self._r_thresh = 0
        self._h = 0
        self._w = 0


            
                    

