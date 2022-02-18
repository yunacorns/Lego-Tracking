import cv2
import numpy as np
import imutils

cap= cv2.VideoCapture(1)
cap.set(3,640)
cap.set(4,480)
dist_lego_from_center = 13

while True:
     _,frame= cap.read()
     frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
     frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

     blurred_frame = cv2.GaussianBlur(frame,(5,5),0)

     hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

     lower_red = np.array([161,155,84])
     upper_red = np.array([179,255,255])
     mask_red = cv2.inRange(hsv,lower_red,upper_red)

     cnts_red = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
     cnts_red = imutils.grab_contours(cnts_red)
     font = cv2.FONT_HERSHEY_COMPLEX
    
     x_coord = []
     y_coord = []
    #  x_coord_topright = []
    #  y_coord_topright = []
    #  x_coord_bottomleft = []
    #  y_coord_bottomleft = []
     for c in cnts_red:
         area = cv2.contourArea(c)
         if area > 500:

             approx = cv2.approxPolyDP(c, 0.009 * cv2.arcLength(c, True), True)
             cv2.drawContours(frame,[approx],-1,(0,255,0), 3)

             


             M = cv2.moments(c)

             cx = int(M["m10"]/ M["m00"])
             cy = int(M["m01"]/ M["m00"])

             cv2.circle(frame,(cx,cy),7,(255,255,255),-1)
            #  cv2.circle(frame,(cx+dist_lego_from_center,cy-dist_lego_from_center),7,(255,255,255),-1)
            #  cv2.circle(frame,(cx-dist_lego_from_center,cy+dist_lego_from_center),7,(255,255,255),-1)

             cv2.putText(frame,'red x={}, y={}'.format(cx,cy),(cx-20,cy-20),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),3)
            #  x_coord_topright.append(cx+dist_lego_from_center)
            #  y_coord_topright.append(cy-dist_lego_from_center)
            #  x_coord_bottomleft.append(cx-dist_lego_from_center)
            #  y_coord_bottomleft.append(cy+dist_lego_from_center)
             x_coord.append(cx)
             y_coord.append(cy)
             

        #  print(x_coord_topright,y_coord_topright)
         print(x_coord,y_coord)

         
         if len(x_coord)==2 and len(y_coord)==2:
            x_val = []
            for x in range(10):
                x_value = x_coord[0]+((x_coord[1]-x_coord[0])/19)
                x_val.append(x_value)


            #  x_val = []
            #  for x in range(19):
            #     x_value = x_coord_topright[0]+((x_coord_topright[1]-x_coord_bottomleft[0]+30)/19)*x
            #     x_val.append(x_value)


            # #  for x in x_val:
            # # make 15 a variable 
            # #  discy = (y_coord_topright[0]-y_coord_topright[1]+4*dist_lego_from_center)/(2*dist_lego_from_center)
            # #  print(discy)
            #  for x in x_val:
                # for y in range(15):
                #     y_val = y_coord_topright[1]+((y_coord_bottomleft[0]-y_coord_topright[1]+30)/15)*y
                #     cv2.circle(frame,(x,y_val),7,(255,255,255),-1)
            
 

     
     cv2.imshow("result",frame)

     k = cv2.waitKey(5)
     if k == 27:
         break

cap.release()
cv2.destroyAllWindows()