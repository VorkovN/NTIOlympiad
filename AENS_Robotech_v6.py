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

arr = [[1.0, 1.0], [4.0, 1.0], [4.0, 5.0], [1.0, 5.0], [1.0, 1.0]]
colours = ["", "", "", ""]

bridge = CvBridge()

rospy.init_node('task2')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

def navigate_wait(x=0.0, y=0.0, z=1.4, yaw=float('nan'), speed=0.4, frame_id='map', auto_arm=False, tolerance=0.3):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

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
	navigate_wait(x=0.0, y=0.0, z=1.4, yaw=0.0, speed=0.15, frame_id='map',  auto_arm=True, tolerance=0.1)
	rospy.sleep(2)	

def nextMark(xc, yc):
	navigate_wait(x=float(xc), y=float(yc), z=1.4, yaw=0.0, speed=0.15, frame_id='map',  auto_arm=True, tolerance=0.1)
	rospy.sleep(1)

def home():
	navigate_wait(x=0.0, y=0.0, z=1.4, yaw=0.0, speed=0.15, frame_id='map',  auto_arm=True, tolerance=0.1)
	rospy.sleep(1)

def image_callback(data):

	global cur_col
	cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
	cv2.imwrite("AENS_Robotech/"+str(time.time())+".jpg", cv_image)
	time.sleep(0.5)


	
def main():
    navigate(x=0, y=0, z=1.5, yaw=0, speed=0.1, frame_id='body', auto_arm=True)

main()
