#! /usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def image_callback(data):
    original_image_pub = rospy.Publisher("bridge/original_image", Image, queue_size=1) 

    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        original_image_msg = bridge.cv2_to_imgmsg(cv_image, "bgr8")  
        original_image_pub.publish(original_image_msg)

    except CvBridgeError as e:
        print(e)

rospy.init_node('kinect_image_node', anonymous=True)  

rospy.Subscriber("camera/rgb/image_raw", Image, image_callback, queue_size=1) 
rospy.spin()