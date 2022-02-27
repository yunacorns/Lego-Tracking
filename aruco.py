import cv2
import cv2.aruco as aruco
import numpy as np
import os


def findArucoMarkers(frame, markerSize = 4, totalMarkers = 250, draw=True):
    imgGray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    key = getattr(aruco,f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bbox, ids, rejected = aruco.detectMarkers(imgGray,arucoDict, parameters=arucoParam)
    
  
    if draw:
        aruco.drawDetectedMarkers(frame,bbox)

    
  
    
    if ids is not None and [1] in ids and [2] in ids:
        
        ids_new = [ids[0][0],ids[1][0]]

        pos_1 = ids_new.index(1)
        pos_2 = ids_new.index(2)
        print(pos_1,pos_2)
        coord1 = bbox[pos_1][0][0]
        coord2 = bbox[pos_2][0][2]
        print(coord1)
        print(coord2)

        coord1_x = int(coord1[0])
        coord2_x = int(coord2[0])
        coord1_y = int(coord1[1])
        coord2_y = int(coord2[1])


     
        new_frame = frame[coord1_y:coord2_y,coord1_x:coord2_x]
        cv2.imshow('calibrated',new_frame)
   
        

  


def main():

    cap= cv2.VideoCapture(1)
    cap.set(3,640)
    cap.set(4,480)

    while True:
        _,frame = cap.read()
        findArucoMarkers(frame)
        cv2.imshow("result",frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    main()
