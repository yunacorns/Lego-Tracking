import cv2
import cv2.aruco as aruco
import numpy as np
import os
import socket
import math

 


def main():

    host, port = "127.0.0.1", 25001
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host,port))
    

    cap= cv2.VideoCapture(0)
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


        # calibration using aruco marker 1 and 2 
        if ids is not None:
            length_ids = len(ids)
            aruco.drawDetectedMarkers(frame,bbox)
        if ids is not None and [1] in ids and [2] in ids:
        
            ids_formatted = []
            for i in range(length_ids):
                ids_formatted.append(ids[i][0])
   
        
            pos_1 = ids_formatted.index(1)
            pos_2 = ids_formatted.index(2)
            # get coordinates of top left and bottom right of 1 and 2 
            TLcoord1 = bbox[pos_1][0][0]
            BRcoord1 = bbox[pos_1][0][2]
            TLcoord2 = bbox[pos_2][0][0]
            BRcoord2 = bbox[pos_2][0][2]
     

            coord1_x = int(TLcoord1[0])+(int(BRcoord1[0])-int(TLcoord1[0]))/2
            coord1_y = int(TLcoord1[1])+(int(BRcoord1[1])-int(TLcoord1[1]))/2
            coord2_x = int(TLcoord2[0])+(int(BRcoord2[0])-int(TLcoord2[0]))/2
            coord2_y = int(TLcoord2[1])+(int(BRcoord2[1])-int(TLcoord2[1]))/2
            # minus in front of y coords bc webcam origin flipped in x axis compared to unity origin
            position1 = [coord1_x-coord1_x,-(coord1_y-coord1_y),0]
            position2 = [coord2_x-coord1_x,-(coord2_y-coord1_y),0]
            posString1 = ','.join(map(str,position1))
            posString2 = ','.join(map(str,position2))


            
            # aruco markers 3 and 4 for the joint coordinates
            if [3] in ids:
                pos_3 = ids_formatted.index(3)
                TLcoord3 = bbox[pos_3][0][0]
                BRcoord3 = bbox[pos_3][0][2]
                # find midpoints
                new_coord3_x = (int(TLcoord3[0])+(int(BRcoord3[0])-int(TLcoord3[0]))/2)-coord1_x
                new_coord3_y = (int(TLcoord3[1])+(int(BRcoord3[1])-int(TLcoord3[1]))/2)-coord1_y
                position3 = [new_coord3_x,-new_coord3_y,0]
                posString3 = ','.join(map(str,position3))
                


            if [4] in ids:
                pos_4 = ids_formatted.index(4)
                TLcoord4 = bbox[pos_4][0][0]
                BRcoord4 = bbox[pos_4][0][2]
                # find midpoints
                new_coord4_x = (int(TLcoord4[0])+(int(BRcoord4[0])-int(TLcoord4[0]))/2)-coord1_x 
                new_coord4_y = (int(TLcoord2[1])+(int(BRcoord2[1])-int(TLcoord2[1]))/2)-coord1_y
                position4 = [new_coord4_x,-new_coord4_y,0]
                posString4 = ','.join(map(str,position4))


            # use 3 and 4 to find length
            if [3] and [4] in ids:
                length3_4 = math.sqrt(math.pow(new_coord4_x-new_coord3_x,2)+math.pow(new_coord4_y-new_coord3_y,2))
                print(length3_4)
                
            # send data to unity
            if [1] and [2] and [3] and [4] in ids:
                posString1234 = posString1 +','+ posString2 + ','+ posString3 + ','+ posString4
                sock.sendall(posString1234.encode("UTF-8"))
                print(posString1234)



        

            #cv2.circle(frame,(coord1_x,coord1_y),7,(255,255,255),-1)


            new_frame = frame[int(coord1_y):int(coord2_y),int(coord1_x):int(coord2_x)]
            cv2.imshow('calibrated',new_frame)


        
        cv2.imshow("result",frame)
        cv2.waitKey(1)
        

if __name__ == "__main__":
    main()
