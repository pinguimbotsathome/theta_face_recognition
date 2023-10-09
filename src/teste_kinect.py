#! /usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Empty
from theta_face_recognition.srv import FaceRec, FaceRecResponse
import pathlib

import rospkg
import face_recognition as fr
import os.path

from cv_bridge import CvBridge


PACK_DIR = rospkg.RosPack().get_path("theta_face_recognition")
OPERADOR_DIR = os.path.join(PACK_DIR, "dataset/operador.jpeg")
COMPARADOR_DIR = os.path.join(PACK_DIR, "dataset/comparador.jpg")

bridge = CvBridge()

latest_img = None
latest_depth = None
take_photo = False

pub_image = rospy.Publisher('face_detection/camera', Image)

class FaceRecognition:
    face_location = []
    face_encodings = []
    face_names = []
    known_face_encodings = []
    known_face_names = []
    

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
    

def recogniton(self):
    global latest_depth
    # sub_depth = rospy.Subscriber('/camera/depth/image', Image, depth_image)
    imgOperador = fr.load_image_file(OPERADOR_DIR)
    imgOperador = cv2.cvtColor(imgOperador, cv2.COLOR_BGR2RGB)

    # face_cascade = cv2.CascadeClassifier('haarcascade_frontaface_default.xml')
    imgComparador = fr.load_image_file(COMPARADOR_DIR)
    imgComparador = cv2.cvtColor(imgComparador, cv2.COLOR_BGR2RGB)
    # faces = face_cascade.detectMultiScale(imgComparador,1,1,4)

    # for(x,y,w,h) in faces:
    #     cv2.rectangle(imgComparador,(x,y),(x+w, y+h),(255,0,0),2)

    # cv2.imshow(imgComparador)
    

    # detector_face_hog = dlib.get_frontal_face_detector()
    # deteccoes = detector_face_hog(comparador,1)
    # deteccoes, len(deteccoes)

    # for face in(deteccoes):
    #     print(face)



    for face in fr.face_locations(imgComparador):
        faceLoc = face
        cv2.rectangle(imgComparador, (faceLoc[3], faceLoc[0]), (faceLoc[1], faceLoc[2]), (255, 0, 0), 2)


    faceLoc = fr.face_locations(imgComparador)[0]
    cv2.rectangle(imgComparador, (faceLoc[3], faceLoc[0]), (faceLoc[1], faceLoc[2]), (0, 255, 0), 2)
    encondeOperador = fr.face_encodings(imgOperador)[0]
    encodeComparador = fr.face_encodings(imgComparador)[0]

    FaceRecognition.known_face_encodings.append(encondeOperador)
    FaceRecognition.known_face_names.append(imgOperador)

    rospy.loginfo(FaceRecognition.known_face_names)


    comparacao = fr.compare_faces([encondeOperador], encodeComparador)
    distancia = fr.face_distance([encondeOperador], encodeComparador)
    x1, y1, x2, y2 = faceLoc

    rospy.loginfo('\nResultados:')
    rospy.loginfo(f'Comparação: {comparacao}')
    rospy.loginfo(f'Distância: {distancia}')

    rospy.loginfo(f'Coordenadas: x1={x1}, y1={y1}, x2={x2}, y2={y2}')

    # cv2.imwrite('Operador.png', imgOperador)
    cv2.imwrite('Final.png', imgComparador)

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

def image_passing(data):
    webcam = cv2.VideoCapture(0) #theta usar VideoCapture(0)
    if webcam.isOpened():
        validacao, frame = webcam.read()
        # while validacao:
        #     validacao, frame = webcam.read()
        # (IMAGE_DIR, frame)
        latest_img = frame
        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(frame, encoding="passthrough")

        pub_image.publish(image_message)
        webcam.release()

if __name__ == '__main__':
    rospy.init_node('kinect_image_node', anonymous=True)  

    rospy.Subscriber("face_detection/camera", Image, img_kinect_rgb) 

    rospy.Subscriber("camera/depth/image_raw", Image, img_kinect_depth) 

    rospy.Subscriber('/face_detection/operador_take', Empty, operador)
    rospy.Subscriber('/face_detection/comparador_take', Empty, comparador)
    rospy.Subscriber('/face_detection/recognition', Empty,recogniton)
    rospy.Subscriber('/face_detection/enable_disable_camera', Empty, change_flag_photo)

    rospy.Service('services/faceRecognition', FaceRec, recogniton)

    rospy.Subscriber('/face_detection/webcam_camera', Empty, image_passing)

    while not rospy.is_shutdown():
        rospy.spin()