#!/usr/bin/env python
import cv2 as cv
import numpy as np

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from lka.msg import Lane
from lka.msg import Lanes
from lka.msg import Margins
from cv_bridge import CvBridge, CvBridgeError

import lane_detect

class LaneKeepAssist:

    def __init__(self, metrics):
        self.steeringPub = rospy.Publisher('lka/steering', Float64, queue_size=1)
        self.lanePub = rospy.Publisher('lka/lanes', Lanes, queue_size=1)
        self.marginsPub = rospy.Publisher('lka/margins', Margins, queue_size=1)

        self.steering = 0.0
        self.metrics = metrics

    def margin_correction(self, left_line, right_line, height, width):
        left_margin = (width/2) - ((height - left_line[1])/left_line[0])
        right_margin = ((height - right_line[1])/right_line[0]) - (width/2)
        # If metrics are enabled, then send margins to metrics node for analysis
        if self.metrics:
            margins = Margins()
            margins.margin_diff = right_margin - left_margin
            self.marginsPub.publish(margins)

        return (right_margin - left_margin)/(width/2)

    def construct_lane_msg(self, lane, values):
        lane.exists = True
        lane.slope = values[0]
        lane.y_cept = values[1]

    def processImage(self, img):
        bridge  = CvBridge()
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

                    lanes_msg = Lanes() # Left lane is first element and right lane is second.

                    if left_line != [] and right_line != []: # There are both left and right lane lines
                        x_intersection = (left_line[1] - right_line[1])/(right_line[0] - left_line[0])
                        margin_steer = self.margin_correction(left_line, right_line, height, width)
                        self.construct_lane_msg(lanes_msg.lane_lines[0], left_line)
                        self.construct_lane_msg(lanes_msg.lane_lines[1], right_line)
                    elif left_line == [] and right_line != []: # Probably too close to the right side of the lane
                        x_intersection = ((height/2) - right_line[1])/(right_line[0] - 0)
                        self.construct_lane_msg(lanes_msg.lane_lines[1], right_line)
                    elif right_line == [] and left_line != []: # Probably too close to the left side of the lane
                        x_intersection = (left_line[1] - (height/2))/(0 - left_line[0])
                        self.construct_lane_msg(lanes_msg.lane_lines[0], left_line)

                    if x_intersection != None:
                        x_steering = (x_intersection / (width / 2)) - 1
                    else:
                        x_steering = self.steering

                    if (margin_steer != 0):
                        rospy.loginfo('margin correction: {}'.format(margin_steer))
                        self.steering = (0.6 * x_steering) + (0.4 * margin_steer)
                    else:
                        self.steering = x_steering

                    rospy.loginfo(self.steering)

                    self.steeringPub.publish(self.steering)
                    self.lanePub.publish(lanes_msg)
                else: # An error
                    rospy.loginfo('Unable to detect lines...')
                    

            except CvBridgeError as e:
                rospy.loginfo(e)

def listener():
    rospy.init_node('lka', anonymous=True)
    lka = LaneKeepAssist(True)
    rospy.Subscriber('airsim/image_raw', Image, lka.processImage)
    rospy.spin()

if __name__ == "__main__":
    listener()