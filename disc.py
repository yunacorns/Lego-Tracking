import cv2
import numpy as np
import imutils

cap= cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)

while True:
     _,frame= cap.read()
     blurred_frame = cv2.GaussianBlur(frame,(5,5),0)

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
             cv2.drawContours(frame,[approx],-1,(0,255,0), 3)
             n = approx.ravel() 
             i = 0

             for j in n : 
                if(i % 2 == 0): 
                    x = n[i] 
                    y = n[i + 1] 

                    # String containing the co-ordinates. 
                    string = str(x) + " " + str(y) 

                    if(i == 0): 
                        # text on topmost co-ordinate. 
                        cv2.putText(frame, "Arrow tip", (x, y), 
                                font, 0.5, (255, 0, 0)) 
                    else: 
                        # text on remaining co-ordinates. 
                        cv2.putText(frame, string, (x, y), 
                            font, 0.5, (0, 255, 0)) 
                i = i + 1


             M = cv2.moments(c)

             cx = int(M["m10"]/ M["m00"])
             cy = int(M["m01"]/ M["m00"])

             cv2.circle(frame,(cx,cy),7,(255,255,255),-1)
             cv2.putText(frame,'red x={}, y={}'.format(cx,cy),(cx-20,cy-20),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),3)
    
     cv2.imshow("result",frame)

     k = cv2.waitKey(5)
     if k == 27:
         break

cap.release()
cv2.destroyAllWindows()