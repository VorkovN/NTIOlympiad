# -*- coding: utf-8 -*-
import cv2
import numpy as np
import matplotlib.pyplot as plt
import numpy as np



if __name__ == '__main__':
    img = cv2.imread('1.jpg')
    cv2.namedWindow("original")
    cv2.imshow("original", img)

    blur = cv2.GaussianBlur(img, (5, 5), 0)
    
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    lower_green = np.array([50, 70, 90])
    upper_green = np.array([70, 250, 240])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    res_green = cv2.bitwise_and(hsv, hsv, mask=mask_green)
    res_green = cv2.cvtColor(res_green, cv2.COLOR_BGR2GRAY)
    ret, res_green = cv2.threshold(res_green, 100, 255, 0)
    _, green_contours, _ = cv2.findContours(res_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    green_count = 0
    for cnt in green_contours:
        area = cv2.contourArea(cnt) # вычисление площади
        if area > 10:
            green_count += 1
        else:
            continue
        rect = cv2.minAreaRect(cnt) # пытаемся вписать прямоугольник
        box = cv2.boxPoints(rect) # поиск четырех вершин прямоугольника
        box = np.int0(box) # округление координат
        cv2.drawContours(img,[box],0,(0,255,0),-1) # рисуем прямоугольник
    print ("green: " + str(green_count))
	
    cv2.namedWindow("green")
    cv2.imshow("green", res_green)


    lower_yellow = np.array([25, 70, 90])
    upper_yellow = np.array([35, 250, 240])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    res_yellow = cv2.bitwise_and(hsv, hsv, mask=mask_yellow)
    res_yellow = cv2.cvtColor(res_yellow, cv2.COLOR_BGR2GRAY)
    ret, res_yellow = cv2.threshold(res_yellow, 100, 255, 0)
    _, yellow_contours, _ = cv2.findContours(res_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    yellow_count = 0
    for cnt in yellow_contours:
        area = cv2.contourArea(cnt) # вычисление площади
        if area > 10:
            yellow_count += 1
        else:
            continue
        rect = cv2.minAreaRect(cnt) # пытаемся вписать прямоугольник
        box = cv2.boxPoints(rect) # поиск четырех вершин прямоугольника
        box = np.int0(box) # округление координат
        cv2.drawContours(img,[box],0,(0,255,255),-1) # рисуем прямоугольник
    print ("yellow: " + str(yellow_count))

    cv2.namedWindow("yellow")
    cv2.imshow("yellow", res_yellow)

    lower_blue = np.array([95, 70, 90])
    upper_blue = np.array([145, 250, 240])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    res_blue = cv2.bitwise_and(hsv, hsv, mask=mask_blue)
    res_blue = cv2.cvtColor(res_blue, cv2.COLOR_BGR2GRAY)
    ret, res_blue = cv2.threshold(res_blue, 100, 255, 0)
    _, blue_contours, _ = cv2.findContours(res_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    blue_count = 0
    for cnt in blue_contours:
        area = cv2.contourArea(cnt) # вычисление площади
        if area > 10:
            blue_count += 1
        else:
            continue
        rect = cv2.minAreaRect(cnt) # пытаемся вписать прямоугольник
        box = cv2.boxPoints(rect) # поиск четырех вершин прямоугольника
        box = np.int0(box) # округление координат
        cv2.drawContours(img,[box],0,(255,0,0),-1) # рисуем прямоугольник
    print ("blue: " + str(blue_count))

    cv2.namedWindow("blue")
    cv2.imshow("blue", res_blue)


    lower_red1 = np.array([0, 70, 90])
    upper_red1 = np.array([10, 250, 240])

    lower_red2 = np.array([170, 70, 90])
    upper_red2 = np.array([179, 250, 240])

    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    res_red1 = cv2.bitwise_and(hsv, hsv, mask=mask_red1)
    res_red2 = cv2.bitwise_and(hsv, hsv, mask=mask_red2)
    res_red = cv2.bitwise_or(res_red1, res_red2)
    res_red = cv2.cvtColor(res_red, cv2.COLOR_BGR2GRAY)
    ret, res_red = cv2.threshold(res_red, 100, 255, 0)
    _, red_contours, _ = cv2.findContours(res_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    red_count = 0
    for cnt in red_contours:
        area = cv2.contourArea(cnt) # вычисление площади
        if area > 10:
            red_count += 1
        else:
            continue
        rect = cv2.minAreaRect(cnt) # пытаемся вписать прямоугольник
        box = cv2.boxPoints(rect) # поиск четырех вершин прямоугольника
        box = np.int0(box) # округление координат
        cv2.drawContours(img,[box],0,(0,0,255),-1) # рисуем прямоугольник
    print ("red: " + str(red_count))

    cv2.namedWindow("red")
    cv2.imshow("red", res_red)

    cv2.imshow("original2", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


