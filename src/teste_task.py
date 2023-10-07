#! /usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Empty
from theta_face_recognition.srv import FaceRec, FaceRecResponse
import time

import rospkg
import face_recognition as fr
import os.path

def img_kinect(data):
	if take_photo:
		# cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
		current_img = data
	

PACK_DIR = rospkg.RosPack().get_path("theta_face_recognition")
OPERADOR_DIR = os.path.join(PACK_DIR, "dataset/operador.png")
COMPARADOR_DIR = os.path.join(PACK_DIR, "dataset/comparador.png")

rospy.init_node('teste_task', anonymous=True)  
rospy.Subscriber("camera/rgb/image_raw", Image, img_kinect) 
operador_pub = rospy.Publisher('/face_detection/operador_take', Image, queue_size = 1)
comparador_pub = rospy.Publisher('/face_detection/comparador_take', Image, queue_size = 1)

current_img = None

bridge = CvBridge()



take_photo = True
time.sleep(1)
take_photo = False

operador_pub.publish(current_img)
rospy.rosinfo('first photo taken')

take_photo = True
time.sleep(1)
take_photo = False

comparador_pub.publish(current_img)
rospy.rosinfo('second photo taken')






