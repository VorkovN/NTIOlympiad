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
import os

cur_col = ""
arr = [[2.0, 2.0], [2.0, 5.0]]
colours = ["", "", "", ""]

COLOR = [
    'yellow',
    'green',
    'blue',
    'red'
]

CARGO_TYPE = {
    'yellow': 'products',
    'green': 'clothes',
    'blue': 'fragile packaging',
    'red': 'correspondence'
}

cargo_amount = {
    'blue': 0,
    'red': 0,
    'green': 0,
    'yellow': 0
}

cur_cargo_amount = {
    'blue': 0,
    'red': 0,
    'green': 0,
    'yellow': 0
}

report = []

points = [[3.6, 3.6], [2.7, 3.6], [1.8, 3.6], [0.9, 3.6], [0.0, 3.6], [0.0, 2.7], [0.9, 2.7], [1.8, 2.7], [2.7, 2.7], [3.6, 2.7], [3.6, 1.8], [2.7, 1.8], [1.8, 1.8], [0.9, 1.8], [0.0, 1.8], [0.0, 0.9], [0.9, 0.9], [1.8, 0.9], [2.7, 0.9], [3.6, 0.9], [3.6, 0.0], [2.7, 0.0], [1.8, 0.0], [0.9, 0.0], [0.0, 0.0]]
# points = [[3.6, 3.6], [2.7, 3.6], [1.8, 3.6], [0.9, 3.6], [0.0, 3.6], [0.0, 2.7], [0.9, 2.7], [1.8, 2.7], [2.7, 2.7]]

ALTITUDE = 1.7
INVENTARIZATION_ALTITUDE = 0.6
INVENTARIZATION_ENABLE = False
INVENTARIZATION_CROP_IMAGE = 5
INVENTARIZATION_TIME = 2
INVENTARIZATION_SPEED = 0.2
AREA_TOLERANCE = 50
SPEED = 0.5
DIR_PATH = 'AENS_Robotech/'

DRONPOINTS_DETECTION_ENABLE = False
DETECTED_DRONPOINT = -1
DRONPOINTS_LANDING = False


log_file = None

bridge = CvBridge()

rospy.init_node('AENS_Robotech')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', srv.SetLEDEffect)

def get_cur_position():
    telem = get_telemetry()
    return (telem.x, telem.y, telem.z)

def logging(s, inconsole=True):
    log_file.write(s + "\n")
    if inconsole:
        print s

def color_blinking(color, duration=5):
    r,g,b = 255, 255, 255
    if color == 'red':
        r, g, b = 255, 0, 0
    elif color == 'green':
        r, g, b = 0, 255, 0
    elif color == 'blue':
        r, g, b = 0, 0, 255
    elif color == 'yellow':
        r, g, b = 255, 255, 0
    
    set_effect(effect='fill', r=r, g=g, b=b)
    rospy.sleep(duration)
    set_effect(r=0, g=0, b=0)

def total_cargo():
    return sum(cargo_amount.values())

def scan_cargo(img):
    global cur_cargo_amount

    blur = cv2.GaussianBlur(img, (3, 3), 0)
    
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    lower_green = np.array([45, 30, 70])
    upper_green = np.array([75, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    res_green = cv2.bitwise_and(hsv, hsv, mask=mask_green)
    res_green = cv2.cvtColor(res_green, cv2.COLOR_BGR2GRAY)
    ret, res_green = cv2.threshold(res_green, 100, 255, 0)
    _, green_contours, _ = cv2.findContours(res_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    green_count = 0
    for cnt in green_contours:
        area = cv2.contourArea(cnt) # вычисление площади
        if area > AREA_TOLERANCE:
            green_count += 1
        else:
            continue
        rect = cv2.minAreaRect(cnt) # пытаемся вписать прямоугольник
        box = cv2.boxPoints(rect) # поиск четырех вершин прямоугольника
        box = np.int0(box) # округление координат
        cv2.drawContours(img,[box],0,(0,255,0),-1) # рисуем прямоугольник
    if green_count > cur_cargo_amount['green']:
        cur_cargo_amount['green'] = green_count
    

    lower_yellow = np.array([20, 50, 90])
    upper_yellow = np.array([40, 250, 240])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    res_yellow = cv2.bitwise_and(hsv, hsv, mask=mask_yellow)
    res_yellow = cv2.cvtColor(res_yellow, cv2.COLOR_BGR2GRAY)
    ret, res_yellow = cv2.threshold(res_yellow, 90, 255, 0)
    _, yellow_contours, _ = cv2.findContours(res_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    yellow_count = 0
    for cnt in yellow_contours:
        area = cv2.contourArea(cnt) # вычисление площади
        if area > AREA_TOLERANCE:
            yellow_count += 1
        else:
            continue
        rect = cv2.minAreaRect(cnt) # пытаемся вписать прямоугольник
        box = cv2.boxPoints(rect) # поиск четырех вершин прямоугольника
        box = np.int0(box) # округление координат
        cv2.drawContours(img,[box],0,(0,255,255),-1) # рисуем прямоугольник
    if yellow_count > cur_cargo_amount['yellow']:
        cur_cargo_amount['yellow'] = yellow_count

    lower_blue = np.array([95, 70, 90])
    upper_blue = np.array([145, 250, 240])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    res_blue = cv2.bitwise_and(hsv, hsv, mask=mask_blue)
    res_blue = cv2.cvtColor(res_blue, cv2.COLOR_BGR2GRAY)
    ret, res_blue = cv2.threshold(res_blue, 100, 255, 0)#110
    _, blue_contours, _ = cv2.findContours(res_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    blue_count = 0
    for cnt in blue_contours:
        area = cv2.contourArea(cnt) # вычисление площади
        if area > AREA_TOLERANCE:
            blue_count += 1
        else:
            continue
        rect = cv2.minAreaRect(cnt) # пытаемся вписать прямоугольник
        box = cv2.boxPoints(rect) # поиск четырех вершин прямоугольника
        box = np.int0(box) # округление координат
        cv2.drawContours(img,[box],0,(255,0,0),-1) # рисуем прямоугольник
    if blue_count > cur_cargo_amount['blue']:
        cur_cargo_amount['blue'] = blue_count

    lower_red1 = np.array([0, 20, 90])
    upper_red1 = np.array([15, 250, 240])

    lower_red2 = np.array([165, 20, 90])
    upper_red2 = np.array([179, 250, 240])

    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    res_red1 = cv2.bitwise_and(hsv, hsv, mask=mask_red1)
    res_red2 = cv2.bitwise_and(hsv, hsv, mask=mask_red2)
    res_red = cv2.bitwise_or(res_red1, res_red2)
    res_red = cv2.cvtColor(res_red, cv2.COLOR_BGR2GRAY)
    ret, res_red = cv2.threshold(res_red, 90, 255, 0)
    _, red_contours, _ = cv2.findContours(res_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    red_count = 0
    for cnt in red_contours:
        area = cv2.contourArea(cnt) # вычисление площади
        if area > AREA_TOLERANCE:
            red_count += 1
        else:
            continue
        rect = cv2.minAreaRect(cnt) # пытаемся вписать прямоугольник
        box = cv2.boxPoints(rect) # поиск четырех вершин прямоугольника
        box = np.int0(box) # округление координат
        cv2.drawContours(img,[box],0,(0,0,255),-1) # рисуем прямоугольник

    if red_count > cur_cargo_amount['red']:
        cur_cargo_amount['red'] = red_count
    
    return img

def dronpoints_detection(img):
    img = img[90:150, 115:195]
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
    if len(contours) == 0:
        return -1
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

    return kostyl_number

def navigate_wait(x=0.0, y=0.0, z=ALTITUDE, yaw=float('nan'), speed=SPEED, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw,
                speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        #logging('[Pos] x = %.3f, y = %.3f, z = %.3f' % get_cur_position())
        telem = get_telemetry(frame_id='navigate_target')
        #logging('[Nav pos] x = %.3f, y = %.3f, z = %.3f' % (telem.x, telem.y, telem.z))
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.1)

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.1)

def take_off():
    navigate_wait(x=0, y=0, z=ALTITUDE, yaw=0.0, speed=SPEED, frame_id='body',  auto_arm=True, tolerance=0.1)
    rospy.sleep(0.5)

def nextMark(xc, yc, zc=ALTITUDE, speed=SPEED, tolerance=0.1):
    navigate_wait(x=float(xc), y=float(yc), z=float(zc), yaw=0.0, speed=speed, frame_id='aruco_map',  auto_arm=True, tolerance=tolerance)
    rospy.sleep(0.5)

def home():
    navigate_wait(x=0.0, y=0.0, z=ALTITUDE, yaw=0.0, speed=SPEED, frame_id='aruco_map',  auto_arm=True, tolerance=0.1)
    navigate_wait(x=0, y=0, z=0.5, yaw=0.0, speed=SPEED/2, frame_id='aruco_map',  auto_arm=True, tolerance=0.05)
    rospy.sleep(0.5)

def image_callback(data):
    global DETECTED_DRONPOINT

    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    cv2.imwrite(DIR_PATH+str(time.time())+".jpg", cv_image)

    if INVENTARIZATION_ENABLE:
        height = cv_image.shape[0]
        min_height, max_height = int(height//2 - height//(INVENTARIZATION_CROP_IMAGE*2)),\
                                 int(height//2 + height//(INVENTARIZATION_CROP_IMAGE*2))
        cv_image = cv_image[min_height:max_height, :]
        cnt_img = scan_cargo(cv_image)
        cv2.imwrite(DIR_PATH+str(time.time())+"contours.jpg", cnt_img)

    if DRONPOINTS_DETECTION_ENABLE:
        result = dronpoints_detection(cv_image)
        if result != -1:
            DETECTED_DRONPOINT = result

def print_inventarization(inconsole=True):
    logging('Balance ' + str(total_cargo()) + ' cargo', inconsole)
    logging('Type 0: ' + str(cargo_amount['yellow']) + ' cargo', inconsole)
    logging('Type 1: ' + str(cargo_amount['green']) + ' cargo', inconsole)
    logging('Type 2: ' + str(cargo_amount['blue']) + ' cargo', inconsole)
    logging('Type 3: ' + str(cargo_amount['red']) + ' cargo', inconsole)
  
def main():
    global log_file, cargo_amount, cur_cargo_amount, INVENTARIZATION_ENABLE, DRONPOINTS_DETECTION_ENABLE, DETECTED_DRONPOINT

    if not os.path.isdir(DIR_PATH):
        os.mkdir(DIR_PATH)

    # Логирование
    log_file = open(DIR_PATH+"logs"+str(time.time())+".txt", "w")

    # Взлет
    take_off()
    nextMark(0.0, 0.0)

    nextMark(0.0, 4.95) # Перемещение на склад     
    image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)
    for i in range(9):
        nextMark(0.45*i, 4.95, zc=INVENTARIZATION_ALTITUDE, speed=INVENTARIZATION_SPEED, tolerance=0.05)
        logging('INV on')
        INVENTARIZATION_ENABLE = True
        time.sleep(INVENTARIZATION_TIME)
        INVENTARIZATION_ENABLE = False
        logging('INV off')

        for k,v in cur_cargo_amount.iteritems():
            cargo_amount[k] += v
            cur_cargo_amount[k] = 0
        print_inventarization(True)
        
    time.sleep(1)
    print_inventarization() # Печатаем отчет об инвентаризации
    image_sub.unregister()

    # Облет всех маркеров для поиска дронпоинтов
    nextMark(points[0][0], points[0][1])
    image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)
    for point in points:
        DETECTED_DRONPOINT = -1
        nextMark(point[0], point[1])
        DRONPOINTS_DETECTION_ENABLE = True
        time.sleep(2)
        DRONPOINTS_DETECTION_ENABLE = False

        if DETECTED_DRONPOINT != -1:  # Нашли дронпоинт
            logging('Find dronpoint D%d at (%.2f, %.2f)' % (DETECTED_DRONPOINT, point[0], point[1]))
            if cargo_amount[COLOR[DETECTED_DRONPOINT]] == 0:
                continue

            color_blinking(COLOR[DETECTED_DRONPOINT], 5)
            time.sleep(5)
            # Посадка на дронпоинт (out of use)
            if DRONPOINTS_LANDING and cargo_amount[COLOR[DETECTED_DRONPOINT]] > 0:
                nextMark(point[0], point[1], 0.5) # TODO: А что делать, если дронпоинт на большой высоте?
                land()
                logging('D%d_delivered %s' % (DETECTED_DRONPOINT, CARGO_TYPE[COLOR[DETECTED_DRONPOINT]]), True)
                take_off()
                time.sleep(1)
            report.append('D%d_delivered to %d cargo' % (DETECTED_DRONPOINT, cargo_amount[COLOR[DETECTED_DRONPOINT]]))
            cargo_amount[COLOR[DETECTED_DRONPOINT]] = 0

        if total_cargo() == 0:
            print("Going home")
            break
    image_sub.unregister()

    home()
    land_wait()

    # Формируем автоматический отчет
    print ''
    report.append('Balance: %d cargo' % total_cargo())
    for line in report:
        logging(line, True)

    log_file.close()

if __name__ == '__main__':
    main()
