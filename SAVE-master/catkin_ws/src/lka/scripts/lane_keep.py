#!/usr/bin/env python
import cv2 as cv
import numpy as np

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import lane_detect

def processImage(img):
    bridge  = CvBridge()
    steering = 0.0
    height = img.height
    width  = img.width
    if img != None:
        try:
            cv_img = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")

            result = lane_detect.lane_detect(cv_img, (height, width))
            if result != None:
                left_line, right_line = result
                rospy.loginfo(result)

                if left_line != [] and right_line != []:
                    x_intersection = (left_line[1] - right_line[1])/(right_line[0] - left_line[0])
                elif left_line == []: # Probably too close to the right side of the lane
                    x_intersection = ((height/2) - right_line[1])/(right_line[0] - 0)
                elif right_line == []: # Probably too close to the left side of the lane
                    x_intersection = (left_line[1] - (height/2))/(0 - left_line[0])

                steering = (x_intersection / (width / 2)) - 1
                rospy.loginfo(steering)
                steeringPub.publish(steering)
            else: # An error
                rospy.loginfo('Unable to detect lines...')
                

        except CvBridgeError as e:
            rospy.loginfo(e)

def listener():
    global steeringPub
    rospy.init_node('lka', anonymous=True)
    steeringPub = rospy.Publisher('control/steering', Float64, queue_size=1)
    rospy.Subscriber('airsim/image_raw', Image, processImage)
    rospy.spin()

if __name__ == "__main__":
    listener()