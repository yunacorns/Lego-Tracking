import cv2
import numpy as np
import imutils

cap= cv2.VideoCapture(1)
cap.set(3,640)
cap.set(4,480)

while True:
     _,frame= cap.read()
     blurred_frame = cv2.GaussianBlur(frame,(5,5),0)

     hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

     lower_red = np.array([161,155,84])
     upper_red = np.array([179,255,255])
     lower_blue = np.array([100,150,0],np.uint8)
     upper_blue = np.array([140,255,255],np.uint8)
     lower_yellow = np.array([20, 100, 100])
     upper_yellow = np.array([30, 255, 255])

     mask_red = cv2.inRange(hsv,lower_red,upper_red)
     mask_blue = cv2.inRange(hsv,lower_blue,upper_blue)

     cnts_red = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
     cnts_red = imutils.grab_contours(cnts_red)
     cnts_blue = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
     cnts_blue= imutils.grab_contours(cnts_blue)


     for c in cnts_red:
         area = cv2.contourArea(c)
         if area > 500:


             cv2.drawContours(frame,[c],-1,(0,255,0), 3)

             M = cv2.moments(c)

             cx = int(M["m10"]/ M["m00"])
             cy = int(M["m01"]/ M["m00"])

             cv2.circle(frame,(cx,cy),7,(255,255,255),-1)
             cv2.putText(frame,'red x={}, y={}'.format(cx,cy),(cx-20,cy-20),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),3)
     for c in cnts_blue:
         area = cv2.contourArea(c)
         if area > 500:


             cv2.drawContours(frame,[c],-1,(0,255,0), 3)

             M = cv2.moments(c)

             cx = int(M["m10"]/ M["m00"])
             cy = int(M["m01"]/ M["m00"])

             cv2.circle(frame,(cx,cy),7,(255,255,255),-1)
             cv2.putText(frame,'blue x={}, y={}'.format(cx,cy),(cx-20,cy-20),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),3)

     cv2.imshow("result",frame)

     k = cv2.waitKey(5)
     if k == 27:
         break

cap.release()
cv2.destroyAllWindows()