#! /usr/bin/env python3
import rospy
import cv2
import face_recognition as fr

def recogniton():
    imgOperador = fr.load_image_file("/home/dallagnol/work_ws/src/theta_face_recognition/dataset/Operador.png")
    imgOperador = cv2.cvtColor(imgOperador, cv2.COLOR_BGR2RGB)

    imgComparador = fr.load_image_file("/home/dallagnol/work_ws/src/theta_face_recognition/dataset/Comparador.png")
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

if __name__ == '__main__':
    try:
        rospy.init_node('image_receive_node', anonymous=True)
        recogniton()
    except rospy.ROSInterruptException:
        pass