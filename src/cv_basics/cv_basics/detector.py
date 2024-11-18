import os
from typing import Optional
import cv2
import numpy as np
import math

import logging
import time

from cv_basics.signs import Signs
from cv_basics.config import DetectorConfig
from cv_basics.detect_signs import DetectTrafficSign
from cv_basics.utils import get_dominant_color, get_frame, resize, my_estimatePoseSingleMarkers

class Detector:

    def __init__(self):
        self.config = DetectorConfig() 
        self.sign_detector = DetectTrafficSign()        

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.parameters =  cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)        

    def get_mask(self, cv2_image, 
                 lower : np.ndarray, 
                 upper : np.ndarray, 
                 lower2 : Optional[np.ndarray] = None, 
                 upper2 : Optional[np.ndarray] = None):                   
                   
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
        
        circles = cv2.HoughCircles(cimg, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=self.config.light_minRadius, maxRadius=self.config.light_maxRadius)
        # circles = cv2.HoughCircles(cimg, cv2.HOUGH_GRADIENT, 1, 80,
        #                        param1=50, param2=10, minRadius=20, maxRadius=30)
    
        if circles is not None:            
            circles = np.uint16(np.around(circles))   
            pers = []         
            for i in circles[0, :]:
                cv2.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)

                light_perimeter = 2*math.pi*(i[2]**2)   
                pers.append(light_perimeter)
            l_per = max(pers)
            bad_distance =  (self.config.traffic_light_perimeter*self.config.focal_length) / l_per

            return bad_distance
        return None    

    def detect_traffic_sign(self, image, target_size = 340) -> dict[str, float]:                
        output = self.sign_detector.predict( image, self.config.focal_length, self.config.traffic_sign_perimeter )
        return output  # -> {"left": [3.1, 5.4], "right": 7.1}
                                    
   
    def detect_aruco(self, image, draw=True) -> float:
        """ 
        ArUco маркер цифра 1 
        """
        # frame = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)       

        markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(image)
        if markerIds is not None:            
            aruco_perimeter = cv2.arcLength(markerCorners[0], True)
            bad_distance =  (self.config.aruco_perimeter*self.config.focal_length) / aruco_perimeter
            if draw:
                cv2.aruco.drawDetectedMarkers(image, markerCorners, markerIds)                       

            return bad_distance
        return None

    
    def get_objects(self, image) -> dict[str, float]:
        """ 
        Возваращает словарь с названиями Обнаруженных объектов и расстояниями до них
        """

        objects = {}

        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) 
        
        aruco = self.detect_aruco(rgb_image)
        if aruco is not None:                            
            objects["aruco"] = aruco                 
        
        traffic_signs = self.detect_traffic_sign(rgb_image)
        if traffic_signs is not None:                
            objects.update(traffic_signs)             
                
        roi, mask = self.get_red_mask(image)
        distance_red = self.detect_red(roi)
        if distance_red is not None:
            objects["traffic_light"] = distance_red             

        return objects

def main():
    detector = Detector()    

    cap = cv2.VideoCapture(0)      # -> BGR format
    cap.set(cv2.CAP_PROP_FPS,32)   

    while cap.isOpened(): 
        cv2.startWindowThread()
        cv2.namedWindow("preview")

        success, image = cap.read()  
        cv2.imshow('raw',image)  
        
        detected_objects = detector.get_objects(image)
        print(detected_objects)

        if len(detected_objects.keys()) > 1:
            min_dist_key = list(detected_objects.keys())[0]
            min_dist = list(detected_objects.values())[0][0]
            for key, val in detected_objects.items():                
                if sorted(val)[0] < min_dist:
                    min_dist = sorted(val)[0]
                    min_dist_key = key

            print(min_dist_key)

        condition = cv2.waitKey(10) & 0xFF
        if condition in {ord("q"), ord("Q"), 27}:                
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()