# -*- coding: utf-8 -*-
import math
import rospy
import cv2
import numpy as np
from clover import srv
from std_srvs.srv import Trigger
from pyzbar import pyzbar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import time

cur_col = ""
file = open("results.txt", "w")
arr = [[2.0, 2.0], [2.0, 5.0]]
colours = ["", "", "", ""]
blue = 0
red = 0
green = 0
yellow = 0

bridge = CvBridge()

rospy.init_node('task2')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)


def scan_cargo(img):
    global cur_col
    global red
    global blue
    global green
    global yellow

	
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

    cv2.imwrite("AENS_Robotech/Contours"+str(time.time())+".jpg", img)


def navigate_wait(x=0.0, y=0.0, z=1.7, yaw=float('nan'), speed=0.2, frame_id='aruco_map', auto_arm=False, tolerance=0.2):

    navigate(x=x, y=y, z=z, yaw=yaw,
	         speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.1)

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.1)

def take_off():
	navigate_wait(x=0, y=0, z=1.7, yaw=0.0, speed=0.2, frame_id='body',  auto_arm=True, tolerance=0.1)
	rospy.sleep(2)	

def nextMark(xc, yc):
	navigate_wait(x=float(xc), y=float(yc), z=1.7, yaw=0.0, speed=0.2, frame_id='aruco_map',  auto_arm=True, tolerance=0.1)
	rospy.sleep(1)

def home():
	navigate_wait(x=0.0, y=0.0, z=1.7, yaw=0.0, speed=0.2, frame_id='aruco_map',  auto_arm=True, tolerance=0.1)
	rospy.sleep(1)

def image_callback(data):
	global callback_scan

	cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
	cv2.imwrite("AENS_Robotech/"+str(time.time())+".jpg", cv_image)
	callback_scan = cv_image
	time.sleep(0.3)
	"""
	global cur_col
	global blue
	global red
	global green
	global yellow

	cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
	cv2.imwrite("AENS_Robotech/"+str(time.time())+".jpg", cv_image)
	time.sleep(0.3)

	

	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

	lower_green = np.array([55, 100, 0])
        upper_green = np.array([65, 255, 255])
	mask_green = cv2.inRange(hsv, lower_green, upper_green)
	res_green = cv2.bitwise_and(hsv, hsv, mask=mask_green)
	res_green = cv2.cvtColor(res_green, cv2.COLOR_BGR2GRAY)
	ret, res_green = cv2.threshold(res_green, 127, 255, 0)

	lower_yellow = np.array([25, 100, 0])
        upper_yellow = np.array([35, 255, 255])
	mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
	res_yellow = cv2.bitwise_and(hsv, hsv, mask=mask_yellow)
	res_yellow = cv2.cvtColor(res_yellow, cv2.COLOR_BGR2GRAY)
	ret, res_yellow = cv2.threshold(res_yellow, 127, 255, 0)

	lower_blue = np.array([115, 100, 0])
        upper_blue = np.array([125, 255, 255])
	mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
	res_blue = cv2.bitwise_and(hsv, hsv, mask=mask_blue)
	res_blue = cv2.cvtColor(res_blue, cv2.COLOR_BGR2GRAY)
	ret, res_blue = cv2.threshold(res_blue, 127, 255, 0)

	lower_red1 = np.array([0, 100, 0])
        upper_red1 = np.array([5, 255, 255])

	lower_red2 = np.array([175, 100, 0])
        upper_red2 = np.array([179, 255, 255])
	mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
	mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
	res_red1 = cv2.bitwise_and(hsv, hsv, mask=mask_red1)
	res_red2 = cv2.bitwise_and(hsv, hsv, mask=mask_red2)
	res_red = cv2.bitwise_or(res_red1, res_red2)
	res_red = cv2.cvtColor(res_red, cv2.COLOR_BGR2GRAY)
	ret, res_red = cv2.threshold(res_red, 127, 255, 0)

	_, green_contours, _ = cv2.findContours(res_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	if len(green_contours)>green:
		green = len(green_contours)
		
# -*- coding: utf-8 -*-
	_, yellow_contours, _ = cv2.findContours(res_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	if len(yellow_contours)>yellow:
		yellow = len(yellow_contours)
		

	_, red_contours, _ = cv2.findContours(res_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	if len(red_contours)>red:
		red = len(red_contours)


	_, blue_contours, _ = cv2.findContours(res_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	if len(blue_contours)>blue:
		blue = len(blue_contours)
	"""

	
def main():
	image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)

	take_off()
	nextMark(0.0, 0.0)
	nextMark(1.8, 0.0)
	nextMark(1.8, 4.7)
	time.sleep(0.3)
	scan_cargo(callback_scan)   
	time.sleep(5)

	home()
	land_wait()
	file.write("green = " + str(green) + "\n")
	file.write("yellow = " + str(yellow) + "\n")
	file.write("red = " + str(red) + "\n")
	file.write("blue = " + str(blue) + "\n")

	cv2.destroyAllWindows()
	file.close()

main()
