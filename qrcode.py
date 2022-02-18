import cv2
import numpy as np
import imutils
from pyzbar.pyzbar import decode

cap= cv2.VideoCapture(1)
cap.set(3,640)
cap.set(4,480)

while True:
    _,frame= cap.read()
    # blurred_frame = cv2.GaussianBlur(frame,(5,5),0)

    # hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

    # lower_red = np.array([161,155,84])
    # upper_red = np.array([179,255,255])
    # mask_red = cv2.inRange(hsv,lower_red,upper_red)

    # cnts_red = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cnts_red = imutils.grab_contours(cnts_red)
    # font = cv2.FONT_HERSHEY_COMPLEX
    # for c in cnts_red:
    #     area = cv2.contourArea(c)
    #     if area > 500:
    #         approx = cv2.approxPolyDP(c, 0.009 * cv2.arcLength(c, True), True)
    #         cv2.drawContours(frame,[approx],-1,(0,255,0), 3)
    #         M = cv2.moments(c)

    #         cx = int(M["m10"]/ M["m00"])
    #         cy = int(M["m01"]/ M["m00"])

    #         cv2.circle(frame,(cx,cy),7,(255,255,255),-1)
    #         cv2.putText(frame,'red x={}, y={}'.format(cx,cy),(cx-20,cy-20),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),3)
    for barcode in decode(frame):
        print(barcode.data)
        myData = barcode.data.decode('utf-8')
        print(myData)
        pts = np.array([barcode.polygon], np.int32)
        pts = pts.reshape((-1,1,2))
        cv2.polylines(frame,[pts],True,(255,0,255),5)
        # gives the top left coordinates of barcode
        pts2 = barcode.rect
        cv2.putText(frame,myData,(pts2[0],pts2[1]),cv2.FONT_HERSHEY_SIMPLEX,0.9,(255,0,255),2)
        if myData == "1":
            first_left = pts2[0]
            first_top = pts2[1]
            print("first",first_left,first_top)
        elif myData == "2":
            second_right = pts2[0]+pts2[2]
            second_bottom = pts2[1]+pts2[3]
            print("second",second_right,second_bottom)    
        try: first_left
        except NameError: first_left = None
        try: second_right
        except NameError: second_right = None
        
        if first_left is not None and second_right is not None :
            new_frame = frame[first_top:second_bottom,first_left:second_right]
            blurred_frame = cv2.GaussianBlur(new_frame,(5,5),0)

            hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

            lower_red = np.array([161,155,84])
            upper_red = np.array([179,255,255])
            mask_red = cv2.inRange(hsv,lower_red,upper_red)

            cnts_red = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cnts_red = imutils.grab_contours(cnts_red)
            font = cv2.FONT_HERSHEY_COMPLEX
            for c in cnts_red:
                area = cv2.contourArea(c)
                if area > 500:
                    approx = cv2.approxPolyDP(c, 0.009 * cv2.arcLength(c, True), True)
                    cv2.drawContours(new_frame,[approx],-1,(0,255,0), 3)
                    M = cv2.moments(c)

                    cx = int(M["m10"]/ M["m00"])
                    cy = int(M["m01"]/ M["m00"])

                    cv2.circle(new_frame,(cx,cy),7,(255,255,255),-1)
                    cv2.putText(new_frame,'red x={}, y={}'.format(cx,cy),(cx-20,cy-20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),2)

            cv2.imshow('calibrated',new_frame)
            

        

        
    
    cv2.imshow("result",frame)

    k = cv2.waitKey(5)
    if k == 27:
         break

cap.release()
cv2.destroyAllWindows()