import roslib
import rospy
import sys
import cv2.cv as cv
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def ToOpenCV(ros_image):
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        return cv_image
    except CvBridgeError, e:
        print e
        raise Exception("Failed to convert to OpenCV image")

    def ToRos(cv_image):
        try:
            ros_image = CvBridge().bridge.cv2_to_imgmsg(cv_image, desired_encoding="passthrough")
            return ros_image
        except CvBridgeError, e:
            print e
            raise Exception("Failed to convert to ROS image")
