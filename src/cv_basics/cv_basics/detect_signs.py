from pathlib import Path
import torch
import numpy as np
import cv2
import time

def distance_to_camera(knownWidth, focalLength, perWidth): 
    return (knownWidth * focalLength) / perWidth

# def detect_traffic_sign_opencv(self, frame):
#         output = {}

#         gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
#         img = cv2.medianBlur(gray, 37)
#         circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT,
#                                 1, 50, param1=120, param2=40)
                
#         if not circles is None:
#             circles = np.uint16(np.around(circles))
#             # print (circles[:, :, 2][0])
#             max_r, max_i = 0, 0
#             for i in range(len(circles[:, :, 2][0])):
#                 if circles[:, :, 2][0][i] > 50 and circles[:, :, 2][0][i] > max_r:
#                     # print (circles[:, :, 2][0][i])
#                     max_i = i
#                     max_r = circles[:, :, 2][0][i]
#             x, y, r = circles[:, :, :][0][max_i]
#             if y > r and x > r:
#                 square = frame[y-r:y+r, x-r:x+r]

#                 zone_0 = square[square.shape[0]*3//8:square.shape[0]
#                                 * 5//8, square.shape[1]*1//8:square.shape[1]*3//8]
#                 cv2.imshow('Zone0', zone_0)
#                 zone_0_color = get_dominant_color(zone_0, 1)

#                 zone_1 = square[square.shape[0]*1//8:square.shape[0]
#                                 * 3//8, square.shape[1]*3//8:square.shape[1]*5//8]
#                 cv2.imshow('Zone1', zone_1)
#                 zone_1_color = get_dominant_color(zone_1, 1)

#                 zone_2 = square[square.shape[0]*3//8:square.shape[0]
#                                 * 5//8, square.shape[1]*5//8:square.shape[1]*7//8]
#                 cv2.imshow('Zone2', zone_2)
#                 zone_2_color = get_dominant_color(zone_2, 1)

#                 if zone_1_color[2] < 60:
#                     if sum(zone_0_color) > sum(zone_2_color):                            
#                         return Signs.LEFT
#                     else:
#                         return Signs.RIGHT
#                 else:
#                     if sum(zone_1_color) > sum(zone_0_color) and sum(zone_1_color) > sum(zone_2_color):
#                         return Signs.FORWARD                   
#             else:
#                 print("N/A") 
#                 return None        

#             for i in circles[0, :]:
#                 cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
#                 cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)

#             perimeter = 2*(row['xmax']-row['xmin']+row['ymax']-row['ymin'])            

#             dist = distance_to_camera(known_perimeter, focal_length, perimeter)
#             if row["name"] in output:
#                 temp = output[row["name"]]
#                 output[row["name"]] = []
#                 output[row["name"]].append(temp)
#                 output[row["name"]].append( dist )
#             else:                
#                 output[row["name"]] = dist

class DetectTrafficSign:

    def __init__(self, 
                 dir_yolo = '/home/none/rtk/circut_ws/src/cv_basics/cv_basics/yolov5' , 
                 path = '/home/none/rtk/circut_ws/src/cv_basics/cv_basics/yolov5/weights/best.pt'):        
        self.load_model(dir_yolo, path)

    def load_model(self, dir_yolo, path):
        self.model = torch.hub.load(dir_yolo, 'custom', path=path, source='local', force_reload=True)

    def predict(self, image, focal_length, known_perimeter, conf = 0.60):    
        output = {}
        self.model.conf = conf

        results = self.model(image)
        crops = results.crop(save=False) # получение найденных объектов как отдельных картинок

        if len(crops) != 0:
            pandas_results = results.pandas().xyxy[0]        

            for index, row in pandas_results.iterrows():
                perimeter = 2*(row['xmax']-row['xmin']+row['ymax']-row['ymin'])            

                dist = distance_to_camera(known_perimeter, focal_length, perimeter)
                if row["name"] not in output:
                    output[row["name"]] = []
                output[row["name"]].append( dist )

        return output

def main():
    detector = DetectTrafficSign()

    cap = cv2.VideoCapture(0)      # -> BGR format
    cap.set(cv2.CAP_PROP_FPS,32)   

    while cap.isOpened(): 
        cv2.startWindowThread()
        cv2.namedWindow("preview")

        success, image = cap.read()  
        cv2.imshow('raw',image)  

        # image = cv2.imread( str( Path(__file__).parent / "image.png") )
        out = detector.predict( image, 50, 400 )
        print(out)

        condition = cv2.waitKey(10) & 0xFF
        if condition in {ord("q"), ord("Q"), 27}:                
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()