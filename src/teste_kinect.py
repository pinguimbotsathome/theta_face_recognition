#! /usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Empty
from theta_face_recognition.srv import FaceRec, FaceRecResponse

import rospkg
import face_recognition as fr
import os.path


PACK_DIR = rospkg.RosPack().get_path("theta_face_recognition")
OPERADOR_DIR = os.path.join(PACK_DIR, "dataset/operador.jpeg")
COMPARADOR_DIR = os.path.join(PACK_DIR, "dataset/comparador.jpg")

bridge = CvBridge()

latest_img = None
latest_depth = None
take_photo = False

def operador(data):
    global latest_img
    # cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # if os.path.exists(OPERADOR_DIR):
    #     breakpoint
    # else:
    cv2.imwrite(OPERADOR_DIR, latest_img)
    rospy.loginfo("operador photo taken")

def comparador(data):
    global latest_img
    # cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # if os.path.exists(COMPARADOR_DIR):
    #     breakpoint
    # else:
    cv2.imwrite(COMPARADOR_DIR, latest_img)
    rospy.loginfo("comparator photo taken")

def recogniton(req):
    global latest_depth
    # sub_depth = rospy.Subscriber('/camera/depth/image', Image, depth_image)
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

    # cv2.imshow('Operador', imgOperador)
    # cv2.imshow('Comparador', imgComparador)
    distance = latest_depth[round((x2+x1)/2)][round((y2+y1)/2)]
    # cv2.waitKey()
    return FaceRecResponse(x1=x1,y1=y1,x2=x2,y2=y2, distance=distance)

def change_flag_photo(data):
    global take_photo
    take_photo = not take_photo
    if take_photo:
        rospy.loginfo("enabling photo")
    else:
        rospy.loginfo("disabling photo")

def img_kinect_rgb(data):
    global take_photo
    global latest_img
    if take_photo:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        latest_img = cv_image
        rospy.loginfo('image saved')

def img_kinect_depth(data):
    global take_photo
    global latest_depth
    if take_photo:
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        latest_depth = cv_image
        rospy.loginfo('image saved')

if __name__ == '__main__':
    rospy.init_node('kinect_image_node', anonymous=True)  

    rospy.Subscriber("camera/rgb/image_raw", Image, img_kinect_rgb) 

    rospy.Subscriber("camera/depth/image_raw", Image, img_kinect_depth) 

    rospy.Subscriber('/face_detection/operador_take', Empty, operador)
    rospy.Subscriber('/face_detection/comparador_take', Empty, comparador)
    rospy.Subscriber('/face_detection/recognition', Empty,recogniton)
    rospy.Subscriber('/face_detection/enable_disable_camera', Empty, change_flag_photo)

    rospy.Service('services/faceRecognition', FaceRec, recogniton)

    while not rospy.is_shutdown():
        rospy.spin()