#!/usr/bin/env python

import roslib
import rospy

from sensor_msgs.msg import Image   # for receiving the video feed
import cv2
import numpy as np
from std_msgs.msg import String
from roboassistant.msg import BinPosition
import os
import time
from cv_bridge import CvBridge, CvBridgeError
from imageConverter import ToOpenCV


#NOTE
KINECT_MAX_DEPTH = 5.55
KINECT_MIN_DEPTH = 0.35

erodeKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
dilateKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))

class Kinect_Ros ():
    def __init__ (self):
        rospy.init_node('Kinect_Ros', anonymous=True)
        self.bridge = CvBridge ()
        self.publish = rospy.Publisher('/binPosition', BinPosition, queue_size = 1)
        #TODO; Fix depth on this (for some reason it gets larger as you move closer to something
        self.depthSub = rospy.Subscriber('/camera/depth/image_raw', Image, self.saveDepthImage)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        rospy.spin()

    def image_callback(self, ros_image):
        self.image = np.asarray(ToOpenCV(ros_image))

        #Green
        self.lower_target_range_one = (48, 55, 84)
        self.upper_target_range_one = (90, 148, 156)

        #Red
        self.lower_target_range_two =  (145, 71, 168)
        self.upper_target_range_two = (179, 156, 214)

        self.process_image()

    def saveDepthImage(self, data):
        depth_image = self.bridge.imgmsg_to_cv2(data, '32FC1')
        self.depth_array = np.array(depth_image, dtype=np.float32)

    def getDepth(self,x,y):
        try:
            return self.depth_array[y][x]
        except:
            self.logfile.write("Error occured in getDepth...\n")
            return -1

    def process_image(self):
        images = dict()

        images["Src"] = self.image    #deepcopy (image)
        images["HSV"] = cv2.cvtColor (self.image, cv2.COLOR_BGR2HSV)

        images["OuterColor"] = cv2.inRange (images["HSV"], self.lower_target_range_one, self.upper_target_range_one)
        images["InnerColor"] = cv2.inRange (images["HSV"], self.lower_target_range_two, self.upper_target_range_two)

        #clear up images
        images["OuterColor"] = cv2.erode(images["OuterColor"], erodeKernel, iterations = 2)
        images["OuterColor"] = cv2.dilate(images["OuterColor"], dilateKernel, iterations = 1)
        images["InnerColor"] = cv2.erode(images["InnerColor"], erodeKernel, iterations = 2)
        images["InnerColor"] = cv2.dilate(images["InnerColor"], dilateKernel, iterations = 1)



        #Show images (for testing)
        #cv2.imshow ("Color One Processed", images["OuterColor"])
        #cv2.imshow ("Color Two Processed", images["InnerColor"])
        #cv2.waitKey(25)

        #find targets
        outer_x, outer_y, outer_w, outer_h = self.find_target(images["OuterColor"])
        inner_x, inner_y, inner_w, inner_h = self.find_target(images["InnerColor"])

        center_inner_x = inner_x + inner_w / 2
        center_inner_y = inner_y + inner_h / 2

        #Check if center_inner is inside outer square
        if((center_inner_x > outer_x and center_inner_x < (outer_x + outer_w)) and (center_inner_y > outer_y and center_inner_y < (outer_y + outer_h))):
            #get distance
            depth = self.getDepth(center_inner_x, center_inner_y)

            #declare position variable
            binPosition = BinPosition();

            #set data
            binPosition.x = center_inner_x
            binPosition.y = center_inner_y
            binPosition.distance = depth

            #publish
            self.publish.publish(binPosition)

    def find_target(self, image):
        # Target Location
        contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Find the index of the largest contour
        areas = [cv2.contourArea(c) for c in contours]

        #Initialize for use
        x = y = w = h = -1

        if len(areas) >= 1:
            max_index = np.argmax(areas)
            cnt = contours[max_index]

            # Bounding rectangle
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(image, (x,y), (x+w, y+h), (0,0,255), 2)

        return (x, y, w, h)

kinect = Kinect_Ros()
