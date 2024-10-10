import os
from typing import Optional
import cv2
import numpy as np
import math

import logging
import time

from cv_basics.signs import Signs
from cv_basics.config import DetectorConfig
from cv_basics.utils import get_dominant_color, get_frame, resize, my_estimatePoseSingleMarkers

class Detector:

    def __init__(self):
        self.config = DetectorConfig()      

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters =  cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)        

    def get_mask(self, cv2_image, 
                 lower : np.ndarray, 
                 upper : np.ndarray, 
                 lower2 : Optional[np.ndarray] = None, 
                 upper2 : Optional[np.ndarray] = None):    
        
        if self.config.detect_tl:
            return cv2_image, None
                   
        hsv = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)
        
        lower1 = np.array(lower)
        upper1 = np.array(upper)        
        mask1 = cv2.inRange(hsv, lower1, upper1)
        
        if lower2 is not None and upper2 is not None:
            lower2 = np.array(lower)            
            upper2 = np.array(upper)        
            mask2 = cv2.inRange(hsv, lower2, upper2)

            mask = cv2.add(mask1, mask2)
        else:
            mask = mask1
        
        res = cv2.bitwise_and(cv2_image, cv2_image, mask = mask)
        kernel = np.ones((self.config.mask_ks,self.config.mask_ks),np.uint8)
        mask = cv2.dilate(mask, kernel, iterations = self.config.mask_iter)
        mask = cv2.bitwise_not(mask)

        return res, mask
    
    def get_red_mask(self, cv2_image):
        return self.get_mask(cv2_image,
                         lower = self.config.lower_red_1, 
                         upper = self.config.upper_red_1, 
                         lower2 = self.config.lower_red_2, 
                         upper2 = self.config.upper_red_2)                  

    def detect_red(self, res : np.ndarray):
        img = cv2.medianBlur(res, 5)
        ccimg = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cimg = cv2.cvtColor(ccimg, cv2.COLOR_BGR2GRAY)
        # я игнорирую красные круги вне диапазона радиусов 20-30 ?
        circles = cv2.HoughCircles(cimg, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=self.config.light_minRadius, maxRadius=self.config.light_maxRadius)
        # circles = cv2.HoughCircles(cimg, cv2.HOUGH_GRADIENT, 1, 80,
        #                        param1=50, param2=10, minRadius=20, maxRadius=30)
    
        if circles is not None:            
            circles = np.uint16(np.around(circles))
            # best_area = 0
            for i in circles[0, :]:
                cv2.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)

            cv2.imshow('detected circles', cimg)
            cv2.imshow('res', res)
            return 20 # TEMP                        
            
        cv2.imshow('detected circles', cimg)
        cv2.imshow('res', res)
        return None    


    def detect_traffic_sign(self, image) -> dict[str, float]:
        return {}  # -> {"left": [3.1, 5.4], "right": 7.1}
                                    

    def detect_road_line(self):
        pass


    # def detect_aruco(self, image, newcameramtx, dst, mtx, dist, draw=True) -> float:
    def detect_aruco(self, image, draw=True) -> float:
        """ 
        ArUco маркер цифра 1 
        """
        frame = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # frame = resize(frame, (200, 200))
        # image = (image - self.mean) / self.std
        # image = np.transpose(image, [2, 0, 1])        

        markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(frame)
        if markerIds:            
            # rvecs, tvecs, trash = my_estimatePoseSingleMarkers(markerCorners, 5.3, newcameramtx, dist)
            # distance = math.sqrt(tvecs[0][0]**2 + tvecs[0][1]**2 + tvecs[0][2]**2)

            aruco_perimeter = cv2.arcLength(markerCorners[0], True)
            bad_distance =  (2*(self.config.aruco_w + self.config.aruco_h)*self.config.focal) / aruco_perimeter

            # for idx in range(len(markerIds)):
            #     if draw:
            #         cv2.drawFrameAxes(dst, mtx, dist, rvecs[idx], tvecs[idx], 5)
            #     print('marker id:%d, pos_x = %f,pos_y = %f, pos_z = %f' % (markerIds[idx], tvecs[idx][0], tvecs[idx][1], tvecs[idx][2]))

            if draw:
                # cv2.aruco.drawDetectedMarkers(dst, markerCorners, markerIds)        
                # cv2.imshow('detect', dst)
                cv2.aruco.drawDetectedMarkers(image, markerCorners, markerIds)                       
                cv2.imshow('detect', image)

            return bad_distance
        return None


    # def get_objects(self, image, newcameramtx, dst, mtx, dist) -> dict[str, float]:
    def get_objects(self, image) -> dict[str, float]:
        """ 
        Возваращает словарь с названиями Обнаруженных объектов и расстояниями до них
        """

        objects = {}

        # aruco = self.detect_aruco(image, newcameramtx, dst, mtx, dist)
        aruco = self.detect_aruco(image)
        if aruco is not None:                            
            objects["aruco"] = aruco        

        # у нас может быть список сразу из нескольких одинаковых / разных знаков !  
        traffic_signs = self.detect_traffic_sign(image)
        if traffic_signs is not None:                
            objects.update(traffic_signs)
                
        roi, mask = self.get_red_mask(image)
        distance_red = self.detect_red(roi)
        if distance_red is not None:
            objects["traffic_light"] = distance_red

        return objects

def main():
    detector = Detector()
    print( detector.get_objects() )

if __name__ == "__main__":
    main()