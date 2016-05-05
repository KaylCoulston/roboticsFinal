#! /usr/bin/env python

import roslib; roslib.load_manifest('roboassistant')
import rospy
import actionlib
import numpy as np
import cv2

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from roboassistant.msg import MoveCloseToTagAction, \
                              MoveCloseToTagFeedback, \
                              MoveCloseToTagResult
# from chores.msg import DoDishesAction, DoDishesServer

bridge = CvBridge()

def ToOpenCV(ros_image):
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    return cv_image
  except CvBridgeError, e:
    print(e)
    raise Exception("Failed to convert to OpenCV image")

class MoveCloseToTagServer:
    # create messages that are used to publish feedback/result
    __feedback = MoveCloseToTagFeedback()
    __result = MoveCloseToTagResult()

    def __init__(self, name):
        self.__action_name = name
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        rospy.sleep(2)

        self.server = actionlib.SimpleActionServer(self.__action_name, MoveCloseToTagAction, execute_cb=self.execute, auto_start=False)
        self.server.start()

    def image_callback(self, data):
        image = np.asarray(ToOpenCV(data)) #convert image
        self.height, self.width, _ = image.shape
        #show the image
        cv2.imshow('image', image)
        cv2.waitKey(1)

    def execute(self, goal):
        print "Executing"

        # helper variables
        r = rospy.Rate(1)
        success = True

        # append the seeds for the
        self.__feedback.distance = 10.

        # publish info to the console for the user
        rospy.loginfo('%s: Searching/Moving toward AR Tag ID %i' % (self.__action_name, goal.ar_tag_id))

        # start executing the actionlib
        for i in xrange(10):
            # check that preempt has not been requested by the client
            if self.server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.__action_name)
                self.server.set_preempted()
                success = False
                break

            # TODO: Put code here to center turtlebot
            self.__feedback.distance = 10. - i
            # publish the feedback
            self.server.publish_feedback(self.__feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
            print "distance: ", self.__feedback.distance

        if success:
            if self.__feedback.distance == 1.:
                self.__result.completed = True
                rospy.loginfo('%s: Succeeded' % self.__action_name)
            else:
                self.__result.completed = False
                rospy.loginfo('%s: Failed' % self.__action_name)
            self.server.set_succeeded(self.__result)

if __name__ == '__main__':
    rospy.init_node('move_to_tag')
    server = MoveCloseToTagServer(rospy.get_name())
    rospy.spin()
