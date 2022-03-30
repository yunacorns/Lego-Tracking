import cv2
import cv2.aruco as aruco
import numpy as np
import os
import socket
import math
import time


 


def main():

    host, port = "127.0.0.1", 25001
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host,port))
    

    cap= cv2.VideoCapture(0)
    time.sleep(2)
    cap.set(3,640)
    cap.set(4,480)

    while True:
        _,frame = cap.read()
        markerSize = 4
        totalMarkers = 250
        imgGray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        key = getattr(aruco,f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
        arucoDict = aruco.Dictionary_get(key)
        arucoParam = aruco.DetectorParameters_create()
        bbox, ids, rejected = aruco.detectMarkers(imgGray,arucoDict, parameters=arucoParam)


        # cropping using aruco marker 1 and 2 
        if ids is not None:
            length_ids = len(ids)
            # aruco.drawDetectedMarkers(frame,bbox)
        if ids is not None and [1] in ids and [2] in ids and [3] in ids and [4] in ids:
        
            ids_formatted = []
            for i in range(length_ids):
                ids_formatted.append(ids[i][0])
           

            calibCoordsX = []
            calibCoordsY = []
            # find midpoint
            for i in range(1,5):
                pos = ids_formatted.index(i) 
                TL = bbox[pos][0][0]
                TR = bbox[pos][0][1]
                BR = bbox[pos][0][2]
                BL = bbox[pos][0][3]
                c = np.array([(TL[0],TL[1]),(TR[0],TR[1]),(BR[0],BR[1]),(BL[0],BL[1])])
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY= int(M["m01"] / M["m00"])
                calibCoordsX.append(cX)
                calibCoordsY.append(cY)
                


             

            # minus in front of y coords bc webcam origin flipped in x axis compared to unity origin
            # change these to 950 and 590 stuff 

            

            # calibration
            width, height = 950,590
            pts1 = np.float32([[calibCoordsX[0],calibCoordsY[0]],[calibCoordsX[1],calibCoordsY[1]],[calibCoordsX[2],calibCoordsY[2]],[calibCoordsX[3],calibCoordsY[3]]])
            pts2 = np.float32([[0,0],[width,0],[0,height],[width,height]])
            M = cv2.getPerspectiveTransform(pts1,pts2)
            dst = cv2.warpPerspective(frame,M,(width,height))
            bbox_c, ids_c, rejected_c = aruco.detectMarkers(dst,arucoDict, parameters=arucoParam)
            aruco.drawDetectedMarkers(dst,bbox_c)
            cv2.imshow('warped',dst)
            # positions 1 and 2 
            position1 = [0,0,0]
            position2 = [width,0,0]
            position3 = [0,height,0]
            position4 = [width,height,0]
            # convert to string - change this to a loop later when sending
            posString1 = ','.join(map(str,position1))
            posString2 = ','.join(map(str,position2))
            posString3 = ','.join(map(str,position3))
            posString4 = ','.join(map(str,position4))
            

       
            
            if ids_c is not None:
                length_ids_c = len(ids_c)
                ids_formatted_c = []
                for i in range(length_ids_c):
                    ids_formatted_c.append(ids_c[i][0])

                # central aruco markers
                for i in range(5,7):
                    if [i] in ids_c:
                        pos = ids_formatted_c.index(i)
                        TL = bbox_c[pos][0][0]
                        TR = bbox_c[pos][0][1] 
                        BR = bbox_c[pos][0][2]
                        BL = bbox_c[pos][0][3]
                        c = np.array([(TL[0],TL[1]),(TR[0],TR[1]),(BR[0],BR[1]),(BL[0],BL[1])])
                        M = cv2.moments(c)
                        cX = int(M["m10"] / M["m00"])
                        cY= int(M["m01"] / M["m00"])
                        position = [i,cX,-cY,0]
                        posString = ','.join(map(str,position))
                        print(posString)
                        sock.sendall(posString.encode("UTF-8"))
                        
        cv2.imshow("result",frame)
        cv2.waitKey(1)
        

if __name__ == "__main__":
    main()
