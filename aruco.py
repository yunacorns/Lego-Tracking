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
    

    cap= cv2.VideoCapture(1)
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
        if ids is not None and [1] in ids and [2] in ids:
        
            ids_formatted = []
            for i in range(length_ids):
                ids_formatted.append(ids[i][0])
   
        
            pos_1 = ids_formatted.index(1)
            pos_2 = ids_formatted.index(2)
            # get coordinates of top left and bottom right of 1 and 2 
            TL1 = bbox[pos_1][0][0]
            TR1 = bbox[pos_1][0][1]
            BR1 = bbox[pos_1][0][2]
            BL1 = bbox[pos_1][0][3]
            size1 = '50,50,0'
            # size1 = str(int(abs(TLcoord1[0]-BRcoord1[0])))+','+str(int(abs(TLcoord1[0]-BRcoord1[0])))+',0'
            TL2 = bbox[pos_2][0][0]
            TR2 = bbox[pos_2][0][1]
            BR2 = bbox[pos_2][0][2]
            BL2 = bbox[pos_2][0][3]
     
            # find midpoint
            c_1 = np.array([(TL1[0],TL1[1]),(TR1[0],TR1[1]),(BR1[0],BR1[1]),(BL1[0],BL1[1])])
            M_1 = cv2.moments(c_1)
            cX_1 = int(M_1["m10"] / M_1["m00"])
            cY_1= int(M_1["m01"] / M_1["m00"])
            c_2 = np.array([(TL2[0],TL2[1]),(TR2[0],TR2[1]),(BR2[0],BR2[1]),(BL2[0],BL2[1])])
            M_2 = cv2.moments(c_2)
            cX_2 = int(M_2["m10"] / M_2["m00"])
            cY_2= int(M_2["m01"] / M_2["m00"])

            # minus in front of y coords bc webcam origin flipped in x axis compared to unity origin
            # change these to 950 and 590 stuff 

            # show cropped image
            new_frame = frame[int(cY_1):int(cY_2),int(cX_1):int(cX_2)]
            cv2.imshow('calibrated',new_frame)

            # calibration
            width, height = 950,590
            pts1 = np.float32([[cX_1,cY_1],[cX_2,cY_1],[cX_2,cY_2],[cX_1,cY_2]])
            pts2 = np.float32([[0,0],[width,0],[width,height],[0,height]])
            M = cv2.getPerspectiveTransform(pts1,pts2)
            dst = cv2.warpPerspective(frame,M,(width,height))
            bbox_c, ids_c, rejected_c = aruco.detectMarkers(dst,arucoDict, parameters=arucoParam)
            aruco.drawDetectedMarkers(dst,bbox_c)
            cv2.imshow('warped',dst)
            # positions 1 and 2 
            position1 = [0,0,0]
            position2 = [width,-height,0]
            posString1 = ','.join(map(str,position1))
            posString2 = ','.join(map(str,position2))
            

       
            
            if ids_c is not None:
                length_ids_c = len(ids_c)
                ids_formatted_c = []
                for i in range(length_ids_c):
                    ids_formatted_c.append(ids_c[i][0])
                print(ids_formatted_c)


            
            # aruco markers 3 and 4 for the joint coordinates
                if [3] in ids_c:
                    pos_3 = ids_formatted_c.index(3)
                    TL3 = bbox_c[pos_3][0][0]
                    TR3 = bbox_c[pos_3][0][1] 
                    BR3 = bbox_c[pos_3][0][2]
                    BL3 = bbox_c[pos_3][0][3]

                    # find midpoints
                    c_3 = np.array([(TL3[0],TL3[1]),(TR3[0],TR3[1]),(BR3[0],BR3[1]),(BL3[0],BL3[1])])
                    M_3 = cv2.moments(c_3)
                    cX_3 = int(M_3["m10"] / M_3["m00"])
                    cY_3= int(M_3["m01"] / M_3["m00"])
                    # new_coord3_x = (int(TLcoord3[0])+(int(BRcoord3[0])-int(TLcoord3[0]))/2)
                    # new_coord3_y = (int(TLcoord3[1])+(int(BRcoord3[1])-int(TLcoord3[1]))/2)
                    position3 = [cX_3,-cY_3,0]
                    posString3 = ','.join(map(str,position3))
                else:
                    posString3 = '0,0,0'
                    

                


                if [4] in ids_c:
                    pos_4 = ids_formatted_c.index(4)
                    TL4 = bbox_c[pos_4][0][0]
                    TR4 = bbox_c[pos_4][0][1] 
                    BR4 = bbox_c[pos_4][0][2]
                    BL4 = bbox_c[pos_4][0][3]

                    # find midpoints
                    c_4 = np.array([(TL4[0],TL4[1]),(TR4[0],TR4[1]),(BR4[0],BR4[1]),(BL4[0],BL4[1])])
                    M_4 = cv2.moments(c_4)
                    cX_4 = int(M_4["m10"] / M_4["m00"])
                    cY_4= int(M_4["m01"] / M_4["m00"])
                    position4 = [cX_4,-cY_4,0]
                    posString4 = ','.join(map(str,position4))
                else:
                    posString4 = '0,0,0'

                if [5] in ids_c:
                    pos_5 = ids_formatted_c.index(5)
                    TL5 = bbox_c[pos_5][0][0]
                    TR5 = bbox_c[pos_5][0][1] 
                    BR5 = bbox_c[pos_5][0][2]
                    BL5 = bbox_c[pos_5][0][3]

                    # find midpoints
                    c_5 = np.array([(TL5[0],TL5[1]),(TR5[0],TR5[1]),(BR5[0],BR5[1]),(BL5[0],BL5[1])])
                    M_5 = cv2.moments(c_5)
                    cX_5 = int(M_5["m10"] / M_5["m00"])
                    cY_5= int(M_5["m01"] / M_5["m00"])
                    position5 = [cX_5,-cY_5,0]
                    posString5 = ','.join(map(str,position5))
                else:
                    posString5 = '0,0,0'

            # use 3 and 4 to find length
            # if [3] and [4] in ids:
            #     length3_4 = math.sqrt(math.pow(new_coord4_x-new_coord3_x,2)+math.pow(new_coord4_y-new_coord3_y,2))
            #     print(length3_4)
                
            # send data to unity


                if [1] and [2] in ids:
                    posStringTotal = posString1 +','+ posString2 + ','+ posString3 + ','+ posString4+','+ posString5+','+ size1
                    sock.sendall(posStringTotal.encode("UTF-8"))
                    print(posStringTotal)


            # homography test
            # source_corners = np.array([(coord1_x,coord1_y),(coord2_x,coord1_y),(coord2_x,coord2_y),(coord2_x,coord2_y)])
            # width, height = 95,59
            # scale = 4
            # target_corners = np.array([(0, 0), (width*scale, 0), (width*scale, height*scale), (0, height*scale)])
            # # Get matrix H that maps source_corners to target_corners
            # H, _ = cv2.findHomography(source_corners, target_corners, params=None)

            # # Apply matrix H to source image.
            # transformed_image = cv2.warpPerspective(
            # frame, H, (frame.shape[1], frame.shape[0]))
            # cv2.imshow('transformed image',transformed_image)
            
        

            #cv2.circle(frame,(coord1_x,coord1_y),7,(255,255,255),-1)




        
        cv2.imshow("result",frame)
        cv2.waitKey(1)
        

if __name__ == "__main__":
    main()
