#! /usr/bin/env python3
import rospy
import cv2
import os.path
import rospkg
from std_msgs.msg import Empty

rospy.init_node('face_detection_node', anonymous=True)

PACK_DIR = rospkg.RosPack().get_path("theta_face_recognition")
OPERADOR_DIR = os.path.join(PACK_DIR,"dataset/operador.png")
COMPARADOR_DIR = os.path.join(PACK_DIR,"dataset/comparador.png")

def operador():
    webcam = cv2.VideoCapture(0)
    if webcam.isOpened():
        validacao, frame = webcam.read()
        while validacao:
            validacao, frame = webcam.read()
            if os.path.exists(OPERADOR_DIR):
                break
            else:   
                cv2.imwrite(OPERADOR_DIR, frame)

def comparador():
    webcam = cv2.VideoCapture(0)
    if webcam.isOpened():
        validacao, frame = webcam.read()
        while validacao:
            validacao, frame = webcam.read()
            if os.path.exists(COMPARADOR_DIR):
                break
            else:   
                cv2.imwrite(COMPARADOR_DIR, frame)

if __name__ == '__main__':
    try:
        subOperador = rospy.Subscriber('/operador_take', Empty ,operador)
        subComparador = rospy.Subscriber('/comparador_take', Empty, comparador)
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass  