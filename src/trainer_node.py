#! /usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os.path

def detect(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    if os.path.exists('/home/dallagnol/work_ws/src/theta_face_recognition/dataset/Comparador.png'):
        return
    else:
        cv2.imwrite("/home/dallagnol/work_ws/src/theta_face_recognition/dataset/Comparador.png", cv_image)
    
rospy.init_node('trainer_node', anonymous=True)
rospy.Subscriber("bridge/original_image", Image, detect, queue_size=1)
rospy.spin()