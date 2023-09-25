#! /usr/bin/env python3
import rospy
import cv2
import os.path

rospy.init_node('trainer_node', anonymous=True)

def detection():
    webcam = cv2.VideoCapture(0)
    if webcam.isOpened():
        validacao, frame = webcam.read()
        while validacao:
            validacao, frame = webcam.read()
            if os.path.exists('/home/dallagnol/work_ws/src/theta_face_recognition/dataset/Comparador.png'):
                break
            else:   
                cv2.imwrite("/home/dallagnol/work_ws/src/theta_face_recognition/dataset/Comparador.png", frame)

if __name__ == "__main__":
    try:
        detection()
    except rospy.ROSInterruptException:
        pass 
