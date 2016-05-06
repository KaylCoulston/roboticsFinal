#! /usr/bin/env python

import roslib; roslib.load_manifest('roboassistant')
import rospy
import actionlib
import numpy as np
import cv2
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from roboassistant.msg import FindHumanAction, FindHumanFeedback, FindHumanResult

bridge = CvBridge()
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')

def ToOpenCV(ros_image):
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    return cv_image
  except CvBridgeError, e:
    print(e)
    raise Exception("Failed to convert to OpenCV image")

class MoveCloseToTagServer:
    # create messages that are used to publish feedback/result
    __feedback = FindHumanFeedback()
    __result = FindHumanResult()

    def __init__(self, name):
        self.__action_name = name
	self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
        self.speach_cmd = rospy.Publisher('/'
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
	rospy.sleep(2)

        self.server = actionlib.SimpleActionServer(self.__action_name, FindHumanAction, execute_cb=self.execute, auto_start=False)
        self.server.start()

    def image_callback(self, data):
        self.image = np.asarray(ToOpenCV(data)) #convert image
        self.height, self.width, _ = image.shape

    def execute(self, goal):
        print "Executing"
        
        person_found = 0

        while(1):
            gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)

            for (x,y,w,h) in faces:
                cv2.rectangle(self.image,(x,y),(x+w,y+h),(255,0,0),2)
                roi_gray = gray[y:y+h, x:x+w]
                roi_color = self.image[y:y+h, x:x+w]
                eyes = eye_cascade.detectMultiScale(roi_gray)
                for (ex,ey,ew,eh) in eyes:
                    person_found = 1
                    cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)

            cv2.imshow('img',img)
            cv2.waitKey(2)

            if person_found == 1:
                




if __name__ == '__main__':
    rospy.init_node('find_human_server')
    server = MoveCloseToTagServer(rospy.get_name())
    rospy.spin()
