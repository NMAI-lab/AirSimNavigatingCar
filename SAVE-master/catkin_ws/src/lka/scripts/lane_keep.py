#!/usr/bin/env python
import cv2 as cv
import numpy as np

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import lane_detect

def margin_correction(left_line, right_line, height, width):
    left_margin = (width/2) - ((height - left_line[1])/left_line[0])
    right_margin = ((height - right_line[1])/right_line[0]) - (width/2)
    return (right_margin - left_margin)/(width/2)

def processImage(img):
    bridge  = CvBridge()
    steering = 0.0
    height = img.height
    width  = img.width
    margin_steer = 0.0
    x_intersection = None

    if img != None:
        try:
            cv_img = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")

            result = lane_detect.lane_detect(cv_img, (height, width))
            if result != None:
                left_line, right_line = result
                # rospy.loginfo(result)

                if left_line != [] and right_line != []: # There are both left and right lane lines
                    x_intersection = (left_line[1] - right_line[1])/(right_line[0] - left_line[0])
                    margin_steer = margin_correction(left_line, right_line, height, width)
                elif left_line == [] and right_line != []: # Probably too close to the right side of the lane
                    x_intersection = ((height/2) - right_line[1])/(right_line[0] - 0)
                elif right_line == [] and left_line != []: # Probably too close to the left side of the lane
                    x_intersection = (left_line[1] - (height/2))/(0 - left_line[0])

                if x_intersection != None:
                    x_steering = (x_intersection / (width / 2)) - 1
                else:
                    x_steering = steering

                if (margin_steer != 0):
                    rospy.loginfo('margin correction: {}'.format(margin_steer))
                    steering = (0.6 * x_steering) + (0.4 * margin_steer)
                else:
                    steering = x_steering

                rospy.loginfo(steering)
                steeringPub.publish(steering)
            else: # An error
                rospy.loginfo('Unable to detect lines...')
                

        except CvBridgeError as e:
            rospy.loginfo(e)

def listener():
    global steeringPub
    rospy.init_node('lka', anonymous=True)
    steeringPub = rospy.Publisher('lka/steering', Float64, queue_size=1)
    rospy.Subscriber('airsim/image_raw', Image, processImage)
    rospy.spin()

if __name__ == "__main__":
    listener()