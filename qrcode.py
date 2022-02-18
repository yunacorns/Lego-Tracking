import cv2
import numpy as np
from pyzbar.pyzbar import decode

cap= cv2.VideoCapture(1)
cap.set(3,640)
cap.set(4,480)

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
            cv2.imshow('new',new_frame)


        

        
    
    cv2.imshow("result",frame)

    k = cv2.waitKey(5)
    if k == 27:
         break

cap.release()
cv2.destroyAllWindows()