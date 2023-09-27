#!/usr/bin/env python3
import rospkg
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from cv_bridge import CvBridge
import os.path


rospy.init_node('face_detection_node', anonymous=True)

PACK_DIR = rospkg.RosPack().get_path("theta_face_recognition")
OPERADOR_DIR = os.path.join(PACK_DIR, "dataset/operador.png")
COMPARADOR_DIR = os.path.join(PACK_DIR, "dataset/comparador.png")

bridge = CvBridge()

def operador(req):
    webcam = cv2.VideoCapture(0)
    if webcam.isOpened():
        validacao, frame = webcam.read()
        while validacao:
            validacao, frame = webcam.read()
            if os.path.exists(OPERADOR_DIR):
                break
            else:
                cv2.imwrite(OPERADOR_DIR, frame)

def comparador(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    
    if os.path.exists(COMPARADOR_DIR):
        return
    else:
        cv2.imwrite(COMPARADOR_DIR, cv_image)

if __name__ == '__main__':
    try:
        subOperador = rospy.Subscriber('/operador_take', Empty, operador)
        subComparador = rospy.Subscriber('/comparador_take', Empty, comparador)
        rospy.Subscriber("bridge/original_image", Image, comparador, queue_size=1)
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass
