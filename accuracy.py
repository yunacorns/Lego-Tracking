import cv2
import numpy as np
import imutils
import cv2.aruco as aruco
from pyzbar.pyzbar import decode

def nothing(x):
    # any operation
    pass

cap= cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)


font = cv2.FONT_HERSHEY_COMPLEX

while True:
     _,frame= cap.read()

     for barcode in decode(frame):
        print(barcode.data)
        myData = barcode.data.decode('utf-8')
        print(myData)
        pts = np.array([barcode.polygon], np.int32)
        pts = pts.reshape((-1,1,2))
        cv2.polylines(frame,[pts],True,(255,0,255),5)
        # gives the top left coordinates of barcode
        pts2 = barcode.rect


        TL = pts[0][0]
        BL = pts[1][0]
        BR = pts[2][0]
        TR = pts[3][0]

        contqr = np.array([(TL[0],TL[1]),(TR[0],TR[1]),(BR[0],BR[1]),(BL[0],BL[1])])
        Momqr = cv2.moments(contqr)
        cXqr = int(Momqr["m10"] / Momqr["m00"])
        cYqr = int(Momqr["m01"] / Momqr["m00"])
        print(cXqr,cYqr)
        cv2.circle(frame,(cXqr,cYqr),7,(255,255,255),-1)
        cv2.putText(frame,"("+str(cXqr)+","+str(cYqr)+")",(cXqr-20,cYqr-20),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),3)



     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

     markerSize = 4
     totalMarkers = 250
     imgGray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
     key = getattr(aruco,f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
     arucoDict = aruco.Dictionary_get(key)
     arucoParam = aruco.DetectorParameters_create()
     bbox, ids, rejected = aruco.detectMarkers(imgGray,arucoDict, parameters=arucoParam)
     aruco.drawDetectedMarkers(frame,bbox)
     if ids is not None:
        TL = bbox[0][0][0]
        TR = bbox[0][0][1]
        BR = bbox[0][0][2]
        BL = bbox[0][0][3]
        cont = np.array([(TL[0],TL[1]),(TR[0],TR[1]),(BR[0],BR[1]),(BL[0],BL[1])])
        # find centre of aruco marker
        Mom = cv2.moments(cont)
        cX = int(Mom["m10"] / Mom["m00"])
        cY= int(Mom["m01"] / Mom["m00"])
        cv2.circle(frame,(cX,cY),7,(255,255,255),-1)
        cv2.putText(frame,"("+str(cX)+","+str(cY)+")",(cX-20,cY-20),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),3)









     lower_red = np.array([161,155,84])
     upper_red = np.array([179,255,255])
     low_blue = np.array([38,86,0])
     high_blue = np.array([121,255,255])

     mask = cv2.inRange(hsv,lower_red,upper_red)
     blue_mask = cv2.inRange(hsv, low_blue, high_blue)

     cnts = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
     cnts = imutils.grab_contours(cnts)
     cnts_blue = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
     cnts_blue = imutils.grab_contours(cnts_blue)

     for c in cnts:
         area = cv2.contourArea(c)
         if area > 400:


             cv2.drawContours(frame,[c],-1,(0,255,0), 3)

             M = cv2.moments(c)

             cx = int(M["m10"]/ M["m00"])
             cy = int(M["m01"]/ M["m00"])

             cv2.circle(frame,(cx,cy),7,(255,255,255),-1)
             cv2.putText(frame,'red',(cx-20,cy-20),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),3)
             cv2.putText(frame,"("+str(cx)+","+str(cy)+")",(cx-40,cy-40),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),3)
    #  for c in cnts_blue:
    #      area = cv2.contourArea(c)
    #      if area > 500:


    #          cv2.drawContours(frame,[c],-1,(0,255,0), 3)

    #          M = cv2.moments(c)

    #          cx = int(M["m10"]/ M["m00"])
    #          cy = int(M["m01"]/ M["m00"])

    #          cv2.circle(frame,(cx,cy),7,(255,255,255),-1)
    #          cv2.putText(frame,'blue',(cx-20,cy-20),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),3)
    #          cv2.putText(frame,'x={}, y={}'.format(cx,cy),(cx-30,cy-30),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),3)



    #  kernel = np.ones((5, 5), np.uint8)
    #  mask = cv2.erode(mask, kernel)

     # Contours detection
     if int(cv2.__version__[0]) > 3:
        # Opencv 4.x.x
        contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
     else:
        # Opencv 3.x.x
        _, contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

     for cnt in contours:
        area = cv2.contourArea(cnt)
        approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
        # cv2.drawContours(frame,[cnt],-1,(0,255,0), 3)
        if area > 400:
            M = cv2.moments(cnt)

            cx_r = int(M["m10"]/ M["m00"])
            cy_r = int(M["m01"]/ M["m00"])
            x = approx.ravel()[0]
            y = approx.ravel()[1]

            if cy_r > 240 and cx_r<320:
                cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)

                if len(approx) == 3:
                    cv2.circle(frame,(cx_r,cy_r),7,(255,255,255),-1)
                    cv2.putText(frame, "Triangle", (x-20, y-20), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),3)
                    cv2.putText(frame,"("+str(cx_r)+","+str(cy_r)+")",(x-40,y-40),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),3)
                elif len(approx) == 4:
                    cv2.circle(frame,(cx_r,cy_r),7,(255,255,255),-1)
                    cv2.putText(frame, "Rectangle", (x-20, y-20), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),3)
                    cv2.putText(frame,"("+str(cx_r)+","+str(cy_r)+")",(x-40,y-40),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),3)
                elif 10 < len(approx) < 20:
                    cv2.circle(frame,(cx_r,cy_r),7,(255,255,255),-1)
                    cv2.putText(frame, "Circle", (x-20, y-20), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),3)
                    cv2.putText(frame,"("+str(cx_r)+","+str(cy_r)+")",(x-40,y-40),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),3)

     cv2.imshow("result",frame)
    #  cv2.imshow("Mask", mask)

     k = cv2.waitKey(5)
     if k == 27:
         break

cap.release()
cv2.destroyAllWindows()