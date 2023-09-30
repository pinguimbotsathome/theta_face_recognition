#! /usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs import Empty

import rospkg
import face_recognition as fr
import os.path


PACK_DIR = rospkg.RosPack().get_path("theta_face_recognition")
OPERADOR_DIR = os.path.join(PACK_DIR, "dataset/operador.png")
COMPARADOR_DIR = os.path.join(PACK_DIR, "dataset/comparador.png")

bridge = CvBridge()

img_depth = []

def operador(data):
  
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    if os.path.exists(OPERADOR_DIR):
        breakpoint
    else:
        cv2.imwrite(OPERADOR_DIR, cv_image)

def comparador(data):
  
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    if os.path.exists(COMPARADOR_DIR):
        breakpoint
    else:
        cv2.imwrite(COMPARADOR_DIR, cv_image)

def recogniton():
    sub_depth = rospy.Subscriber('/camera/depth/image', Image, depth_image)

    imgOperador = fr.load_image_file(OPERADOR_DIR)
    imgOperador = cv2.cvtColor(imgOperador, cv2.COLOR_BGR2RGB)

    imgComparador = fr.load_image_file(COMPARADOR_DIR)
    imgComparador = cv2.cvtColor(imgComparador, cv2.COLOR_BGR2RGB)

    faceLoc = fr.face_locations(imgComparador)[0]
    cv2.rectangle(imgComparador, (faceLoc[3], faceLoc[0]), (faceLoc[1], faceLoc[2]), (0, 255, 0), 2)

    encondeOperador = fr.face_encodings(imgOperador)[0]
    encodeComparador = fr.face_encodings(imgComparador)[0]

    comparacao = fr.compare_faces([encondeOperador], encodeComparador)
    distancia = fr.face_distance([encondeOperador], encodeComparador)
    x1, y1, x2, y2 = faceLoc

    rospy.loginfo('\nResultados:')
    rospy.loginfo(f'Comparação: {comparacao}')
    rospy.loginfo(f'Distância: {distancia}')

    rospy.loginfo(f'Coordenadas: x1={x1}, y1={y1}, x2={x2}, y2={y2}')

    cv2.imshow('Operador', imgOperador)
    cv2.imshow('Comparador', imgComparador)

    cv2.waitKey()
    
    while not rospy.is_shutdown():
        pass
    

def depth_image(data):
    img_depth = bridge.imgmsg_to_cv2(data, "bgr8") # change encode



if __name__ == '__main__':
    rospy.init_node('kinect_image_node', anonymous=True)  

    rospy.Subscriber("camera/rgb/image_raw", Image, image_callback, queue_size=1) 

    sub_operador = rospy.Subscriber('/face_detection/operador_take', Empty, operador)
    sub_comparador = rospy.Subscriber('/face_detection/comparador_take', Empty, comparador)
    sub_recognition = rospy.Subscriber('/face_detection/recognition', Empty,recogniton)

    rospy.spin()

    while not rospy.is_shutdown():
        pass