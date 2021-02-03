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
file = open("results.txt", "w")
arr = [[2.0, 2.0], [2.0, 5.0]]
colours = ["", "", "", ""]

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

ALTITUDE = 1.7
INVENTARIZATION_ALTITUDE = 0.5
INVENTARIZATION_ENABLE = False
INVENTARIZATION_CROP_IMAGE = 5
INVENTARIZATION_TIME = 2
INVENTARIZATION_SPEED = 0.2
AREA_TOLERANCE = 50
SPEED = 0.5
DIR_PATH = 'AENS_Robotech/'

KMEANS_N_COLORS = 5
COLOR_TOLERANCE = 60

REFERENCE_COLORS = {
    'green': [87, 128, 77],
    'red': [102, 103, 194],
    'yellow': [91, 214, 216],
    'blue': [130, 117, 73]
}

DEBUG_COLORS = {
    'green': [0, 255, 0],
    'red': [0, 0, 255],
    'yellow': [0, 255, 255],
    'blue': [255, 0, 0]
}

bridge = CvBridge()

rospy.init_node('AENS_Robotech')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)


def scan_cargo(img):
    global cur_cargo_amount
    
    # Поиск контуров
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray_img, 100, 255, cv2.THRESH_BINARY)
    _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    # Поиск прямоугольников
    rects, shapes = [], []
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.025*cv2.arcLength(cnt, True), True)
        area = cv2.contourArea(approx)
        if len(approx) == 4 and area > 100:
            rects.append(np.reshape(approx, (4, 2)))
            shapes.append(cv2.minAreaRect(cnt))

    cargo_idx = {
        'green': [],
        'red': [],
        'yellow': [],
        'blue': []
    }
    cur_cur_cargo_amount = {
        'blue': 0,
        'red': 0,
        'green': 0,
        'yellow': 0
    }

    # Выделение доминантного цвета в прямоугольнике и сравнение с референсными цветами посылок
    for i in range(len(shapes)):
        box = cv2.boxPoints(shapes[i]).astype(np.uint8)
        coords_min = np.min(box, axis=0)
        coords_max = np.max(box, axis=0)
        crop_img = img[coords_min[1]:coords_max[1], coords_min[0]:coords_max[0]]

        # Применяем метод k-средних
        pixels = np.float32(crop_img.reshape(-1, 3))
        _, labels, palette = cv2.kmeans(pixels, KMEANS_N_COLORS, None,
                                        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, .1), 10, cv2.KMEANS_RANDOM_CENTERS)
        _, counts = np.unique(labels, return_counts=True)

        dominant = palette[np.argmax(counts)]

        min_diff = 255
        min_color = ''
        for color in ['green', 'red', 'yellow', 'blue']:
            diff = cv2.norm(dominant.astype(np.uint8) - REFERENCE_COLORS[color])
            if diff < min_diff:
                min_color = color
                min_diff = diff
        
        if min_diff < COLOR_TOLERANCE:
            cur_cur_cargo_amount[min_color] += 1
            cargo_idx[min_color].append(i)

    # Стремимся зафиксировать в одном кадре все грузы
    for k, v in cur_cur_cargo_amount.iteritems():
        if cur_cargo_amount[k] < v:
            cur_cargo_amount[k] = v

    for color, cargo in cargo_idx.iteritems():
        img = cv2.drawContours(img, [rects[i] for i in range(len(rects)) if i in cargo], -1, DEBUG_COLORS[color], 2)

    return img


def navigate_wait(x=0.0, y=0.0, z=ALTITUDE, yaw=float('nan'), speed=SPEED, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
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
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    cv2.imwrite(DIR_PATH+str(time.time())+".jpg", cv_image)

    if INVENTARIZATION_ENABLE:
        height = cv_image.shape[0]
        min_height, max_height = int(height//2 - height//(INVENTARIZATION_CROP_IMAGE*2)),\
                                 int(height//2 + height//(INVENTARIZATION_CROP_IMAGE*2))
        cv_image = cv_image[min_height:max_height, :]
        cnt_img = scan_cargo(cv_image)
        cv2.imwrite(DIR_PATH+str(time.time())+"contours.jpg", cnt_img)

def print_inventarization():
    total = sum(cargo_amount.values())

    print 'Balance', total, 'cargo'
    print 'Type 0:', cargo_amount['yellow'], 'cargo'
    print 'Type 1:', cargo_amount['green'], 'cargo'
    print 'Type 2:', cargo_amount['blue'], 'cargo'
    print 'Type 3:', cargo_amount['red'], 'cargo'
  
def main():
    global cargo_amount, cur_cargo_amount, INVENTARIZATION_ENABLE

    if not os.path.isdir(DIR_PATH):
        os.mkdir(DIR_PATH)

    take_off()
    nextMark(0.0, 0.0)
    nextMark(0.0, 4.9)

    image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)
    for i in range(9):
        nextMark(0.45*i, 4.95, zc=INVENTARIZATION_ALTITUDE, speed=INVENTARIZATION_SPEED, tolerance=0.05)
        print 'INV on'
        INVENTARIZATION_ENABLE = True
        time.sleep(INVENTARIZATION_TIME)
        INVENTARIZATION_ENABLE = False
        print 'INV off'

        for k,v in cur_cargo_amount.iteritems():
            cargo_amount[k] += v
            cur_cargo_amount[k] = 0
        print_inventarization()
        
    time.sleep(1)
    print_inventarization()
    image_sub.unregister()
    nextMark(3.6, 5.4)

    # TODO: Облет всех маркеров для поиска дронпоинтов
    # ...

    home()
    land_wait()

if __name__ == '__main__':
    main()
