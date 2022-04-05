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
        # aruco.drawDetectedMarkers(frame,bbox)

        
        # cropping using aruco marker 1 and 2 
        if ids is not None:
            length_ids = len(ids)
            # aruco.drawDetectedMarkers(frame,bbox)
        if ids is not None and [1] in ids and [2] in ids and [3] in ids and [4] in ids:
        
            ids_formatted = []
            for i in range(length_ids):
                ids_formatted.append(ids[i][0])

            # find 1 is top left 2 is top right 3 is bottom left 4 is bottom right
            pos1 = ids_formatted.index(1) 
            pos2 = ids_formatted.index(2)
            pos3 = ids_formatted.index(3)
            pos4 = ids_formatted.index(4)
            TL1 = bbox[pos1][0][0]
            TR2 = bbox[pos2][0][1]
            BL3 = bbox[pos3][0][3]
            BR4 = bbox[pos4][0][2]

            calibCoordsX = [TL1[0],TR2[0],BL3[0],BR4[0]]
            calibCoordsY = [TL1[1],TR2[1],BL3[1],BR4[1]]
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

            # menu parameters
            # find middle section
            topOfMenu = int(height/4)
            bottomOfMenu = int((3*height)/4)
            numberOfSections = 3
            widthOfEachSection = (bottomOfMenu-topOfMenu)/numberOfSections
            firstSec = int(topOfMenu + widthOfEachSection)
            secondSec = int(topOfMenu + 2*widthOfEachSection)

       
            
            if ids_c is not None:
                length_ids_c = len(ids_c)
                ids_formatted_c = []
                for i in range(length_ids_c):
                    ids_formatted_c.append(ids_c[i][0])

                totalPosition = []
                # central aruco markers
                for i in range(5,8):
                    if [i] in ids_c:
                        pos = ids_formatted_c.index(i)
                        TL = bbox_c[pos][0][0]
                        TR = bbox_c[pos][0][1] 
                        BR = bbox_c[pos][0][2]
                        BL = bbox_c[pos][0][3]
                        c = np.array([(TL[0],TL[1]),(TR[0],TR[1]),(BR[0],BR[1]),(BL[0],BL[1])])
                        # find centre of aruco marker
                        M = cv2.moments(c)
                        cX = int(M["m10"] / M["m00"])
                        cY= int(M["m01"] / M["m00"])
                        # send coordinates to unity
                        totalPosition.append(i)
                        totalPosition.append(cX)
                        totalPosition.append(-cY)
                        totalPosition.append(0)
                    else:
                        totalPosition.append(i)
                        totalPosition.append(-100)
                        totalPosition.append(0)
                        totalPosition.append(0)
                        # position = [i,cX,-cY,0]
                        # posString = ','.join(map(str,position))
                        # print(posString)
                        # sock.sendall(posString.encode("UTF-8")

                # play button
                if [24] in ids_c:
                    pos = ids_formatted_c.index(24)
                    TL = bbox_c[pos][0][0]
                    TR = bbox_c[pos][0][1] 
                    BR = bbox_c[pos][0][2]
                    BL = bbox_c[pos][0][3]
                    c = np.array([(TL[0],TL[1]),(TR[0],TR[1]),(BR[0],BR[1]),(BL[0],BL[1])])
                    # find centre of aruco marker
                    M = cv2.moments(c)
                    cX = int(M["m10"] / M["m00"])
                    cY= int(M["m01"] / M["m00"])
                    # if in first box send "24,edit". if in second box send"24,animate". if third send "24,data"
                    totalPosition.append(24)
                    if cX>=825 and topOfMenu<cY<firstSec:
                        totalPosition.append(0)
                        totalPosition.append("edit")
                    elif cX>=835 and firstSec<cY<secondSec:
                        totalPosition.append(1)
                        totalPosition.append("animate")
                    elif cX>=835 and secondSec<cY<bottomOfMenu:
                        totalPosition.append(2)
                        totalPosition.append("data")
                    else:
                        totalPosition.append(10)
                        totalPosition.append("none")
                else:
                    totalPosition.append(24)
                    totalPosition.append(10)
                    totalPosition.append("none")
                    # make sure to send -cY
                    



                totalPosString = ','.join(map(str,totalPosition))
                print(totalPosString)
                sock.sendall(totalPosString.encode("UTF-8"))


                        
        cv2.imshow("result",frame)
        cv2.waitKey(1)
        

if __name__ == "__main__":
    main()
