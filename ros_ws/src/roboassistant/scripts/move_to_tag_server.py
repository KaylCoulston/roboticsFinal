#! /usr/bin/env python

import roslib; roslib.load_manifest('roboassistant')
import rospy
import actionlib
import numpy as np
import cv2

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
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
        self._id = -1
        self._x = -100.
        self._y = 0.
        self._distance = 0.
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.alvar_callback)
        rospy.sleep(2)

        self.server = actionlib.SimpleActionServer(self.__action_name, MoveCloseToTagAction, execute_cb=self.execute, auto_start=False)
        self.server.start()

    def alvar_callback(self, data):
        try:
            print "ID: ", data.markers[0].id
            print "x: ", data.markers[0].pose.pose.position.x
            print "y: ", data.markers[0].pose.pose.position.y
            print "z: ", data.markers[0].pose.pose.position.z
            self._id = data.markers[0].id
            self._x = data.markers[0].pose.pose.position.x
            self._y = data.markers[0].pose.pose.position.y
            self._distance = data.markers[0].pose.pose.position.z
            #print "Orientation: ", data.markers[0].pose.pose.orientation
            #print "x: ", data.markers.pose.pose.position.x
        except:
            self._x = -100.
            self._id = -1

    def image_callback(self, data):
        pass
        # image = np.asarray(ToOpenCV(data)) #convert image
        # # self.height, self.width, _ = image.shape
        # # print "H: ", self.height
        # # print "W: ", self.width
        #
        # #show the image
        # # h = int((self._y / self._distance * 480) + 480/2.)
        # # w = int((self._x / self._distance * 640) + 640/2.)
        # # cv2.line(image, (0, h), (639, h), 0)  #draw horizontal line
        # # cv2.line(image, (w, 0), (w, 479), 0)  #draw vertical line
        #
        # cv2.imshow('image', image)
        # cv2.waitKey(1)

        # self._x = self._y = self._distance = 0.

    def execute(self, goal):
        rospy.loginfo("Executing move towards AR Tag ID %i" % goal.ar_tag_id)
        self._id = -1
        self._x = -100.
        # helper variables
        r = rospy.Rate(1)
        success = True

        # append the seeds for the
        self.__feedback.distance = 10.

        # publish info to the console for the user
        rospy.loginfo('%s: Searching/Moving toward AR Tag ID %i' % (self.__action_name, goal.ar_tag_id))

        # start executing the actionlib
        cmd = Twist()
        cmd.linear.x = 0.
        cmd.linear.y = 0.
        cmd.linear.z = 0.

        condition = self._x < -0.05 or \
                    self._x > 0.05
        while condition:
            # check that preempt has not been requested by the client
            if self.server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.__action_name)
                self.server.set_preempted()
                success = False
                break

            if self._id == goal.ar_tag_id:
                #print "ID FOUND!"
                if self._x < -0.05 or self._x > 0.05:
                    cmd.linear.z = 0.6 * -self._x /0.3
                    self.cmd_vel.publish(cmd)
                    print "CENTERING TO TAG"
                else:
                    cmd.linear.z = 0.0
                    self.cmd_vel.publish(cmd)
                    #rospy.sleep(.1)
                    break
            else:
                #print "ID NOT FOUND!"
                cmd.angular.z = -0.4
                self.cmd_vel.publish(cmd)

        cmd = Twist()
        cmd.linear.x = 0.05
        cmd.linear.y = 0.
        cmd.linear.z = 0.
        while self._distance > 0.18:
            # check that preempt has not been requested by the client
            if self.server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.__action_name)
                self.server.set_preempted()
                success = False
                break
            self.cmd_vel.publish(cmd)

        cmd = Twist()
        cmd.linear.x = 0.
        cmd.linear.y = 0.
        cmd.linear.z = 0.
        self.cmd_vel.publish(cmd)

        self.__feedback.distance = self._distance
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
