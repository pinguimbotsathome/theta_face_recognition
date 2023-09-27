#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    image_msg = bridge.cv2_to_imgmsg(cv_image, "bgr8")
    image_pub.publish(image_msg)


if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    rospy.Subscriber("camera/rgb/image_raw", Image, callback, queue_size=1)
    image_pub = rospy.Publisher("kinect/converted_image", Image, queue_size=1)
