#! /usr/bin/env python3
import rospy
import cv2
import os.path
import rospkg

rospy.init_node('face_detection_node', anonymous=True)

PACK_DIR = rospkg.RosPack().get_path("theta_face_recognition")
OPERADOR_DIR = os.path.join(PACK_DIR,"dataset/operador.png")
COMPARADOR_DIR = os.path.join(PACK_DIR,"dataset/comparador.png")

def detection():
    webcam = cv2.VideoCapture(0)
    if webcam.isOpened():
        validacao, frame = webcam.read()
        while validacao:
            validacao, frame = webcam.read()
            if os.path.exists(OPERADOR_DIR):
                break
            else:   
                cv2.imwrite(COMPARADOR_DIR, frame)

if __name__ == "__main__":
    try:
        detection()
    except rospy.ROSInterruptException:
        pass 