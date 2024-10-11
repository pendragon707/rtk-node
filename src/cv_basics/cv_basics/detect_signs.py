from pathlib import Path
import torch
import numpy as np
import cv2
import time

def distance_to_camera(knownWidth, focalLength, perWidth): 
 return (knownWidth * focalLength) / perWidth

class DetectTrafficSign:

    def __init__(self, 
                 dir_yolo = '/home/none/rtk/circut_ws/src/cv_basics/cv_basics/yolov5' , 
                 path = '/home/none/rtk/circut_ws/src/cv_basics/cv_basics/yolov5/weights/best.pt'):        
        self.load_model(dir_yolo, path)

    def load_model(self, dir_yolo, path):
        self.model = torch.hub.load(dir_yolo, 'custom', path=path, source='local', force_reload=True)

    def preprocess(self, image, target_size=320):
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert to RGB
        image = cv2.resize(image, (target_size, target_size))  # Resize to target size
        image = image / 255.0  # Normalize pixel values to [0, 1]

        return image

    def predict(self, image, focal_length, known_perimeter, conf = 0.75):    
        output = {}
        # self.model.conf = conf

        results = self.model(image)
        crops = results.crop(save=False) # получение найденных объектов как отдельных картинок

        if len(crops) != 0:
            pandas_results = results.pandas().xyxy[0]        

            for index, row in pandas_results.iterrows():
                perimeter = 2*(row['xmax']-row['xmin']+row['ymax']-row['ymin'])            

                dist = distance_to_camera(known_perimeter, focal_length, perimeter)
                if row["name"] in output:
                    temp = output[row["name"]]
                    output[row["name"]] = []
                    output[row["name"]].append(temp)
                    output[row["name"]].append( dist )
                else:                
                    output[row["name"]] = dist

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
