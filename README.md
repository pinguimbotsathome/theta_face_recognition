# Theta_face_recognition 

## Overview
The theta_face_recognition package uses Freenect_stack and Oppeni_camera to start the Kinect and be able to use its image topics. We use Dlib to perform face recognition and comparison in the images.

## Nodos
* **`kinect_image_node:`** : This node uses the topic provided by the Kinect and converting between ROS images and OpenCV images, thus functioning as a bridge.
  
* **`face_detection_node:`** : This node uses the topic provided by the kinect_image_node and saves a frame into a .jpg image, being a picture of the Operator.
  
* **`face_recognition_node.:`** : This node also uses the topic provided by the kinect_image_node and saves a frame into a image, with the photo to be compared as Comparator.png.
  
* **`trainer_node.:`** : This node compares the image of the Operator with the Comparator and returns whether the Operator was found in the other image.
