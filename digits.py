# -*- coding: utf-8 -*-
import cv2
import numpy as np
import matplotlib.pyplot as plt
import numpy as np



if __name__ == '__main__':
    img = cv2.imread('3.jpg')
    cv2.namedWindow("original")
    cv2.imshow("original", img)

    blur = cv2.GaussianBlur(img, (9, 9), 0)
    
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    lower_green = np.array([50, 100, 80])
    upper_green = np.array([80, 240, 220])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    res_green = cv2.bitwise_and(hsv, hsv, mask=mask_green)
    res_green = cv2.cvtColor(res_green, cv2.COLOR_BGR2GRAY)
    ret, res_green = cv2.threshold(res_green, 80, 255, 0)
    _, contours, _ = cv2.findContours(res_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    green_count = 0

    contours = [cnt for cnt in contours if cv2.contourArea(cnt)>80] # вычисление площади и очистка
    cv2.drawContours(img,contours,0,(0,255,0),1)
    
    kostyl_number = 0
    kostyl_size = cv2.contourArea(contours[0])/(cv2.arcLength (contours[0], True )**2)
    if (kostyl_size > 0.040):
        kostyl_number = 0
    elif(kostyl_size > 0.020):
        kostyl_number = 1
    elif(kostyl_size > 0.016):
        kostyl_number = 2
    else:
        kostyl_number = 3

    print kostyl_number

    # for cnt in contours:
    #     print (str(cv2.contourArea(cnt)) + " " + str(cv2.arcLength (cnt, True )) + " " + str(cv2.contourArea(cnt)/(cv2.arcLength (cnt, True )**2)))
	
    cv2.namedWindow("green")
    cv2.imshow("green", res_green)


    

    cv2.imshow("original2", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


