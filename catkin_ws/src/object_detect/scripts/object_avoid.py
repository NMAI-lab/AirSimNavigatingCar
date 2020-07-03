#!/usr/bin/env python
import cv2 as cv
import numpy as np

import rospy

import threading

from std_msgs.msg import Empty
from std_msgs.msg import Float64
from lka.msg import Lane
from lka.msg import Lanes

class AvoidPedestrians():

    def __init__(self):
        self.left_line_info = Lane()
        self.right_line_info = Lane()
        # Used to ensure getting line info is an atomic instructions
        # due to the time required for object detection 
        self.sem = threading.Semaphore() 
        self.throttlePub = rospy.Publisher('object_avoid/throttle', Float64, queue_size=1)
        self.brakePub = rospy.Publisher('object_avoid/brake', Float64, queue_size=1)
        self.clearPub = rospy.Publisher('object_avoid/clear', Empty, queue_size=1)

    def objects_in_road(self, rectangles):
        self.sem.acquire()
        left_exists = self.left_line_info.exists
        left_slope = self.left_line_info.slope
        left_yint = self.left_line_info.y_cept
        right_exists = self.right_line_info.exists
        right_slope = self.right_line_info.slope
        right_yint = self.right_line_info.y_cept
        self.sem.release()

        left_in_lane = False
        left_in_road = False
        right_in_lane = False 
        right_in_road = False

        if left_exists:
            left_in_lane, left_in_road = self.in_left_lane(left_slope, left_yint, rectangles)
            rospy.loginfo("Left lane exists")
        else: 
            left_in_road = False
            rospy.loginfo("Left lane does not exist")

        if right_exists:
            right_in_lane, left_in_road = self.in_right_lane(right_slope, right_yint, rectangles)
            rospy.loginfo("Right lane exists")
        else:
            right_in_road = False
            rospy.loginfo("Right lane does not exist")

        if (right_in_road and left_in_road):
            # stop car
            self.brakePub.publish(1.0)
            self.throttlePub.publish(0.0)
            rospy.loginfo("Found person in road -> stopping")
        elif (left_in_lane):
            # slow down
            self.brakePub.publish(0.5)
            self.throttlePub.publish(0.0)
            rospy.loginfo("Found person overlapping left lane -> slowing down")
        elif (right_in_lane):
            # slow down
            self.brakePub.publish(0.5)
            self.throttlePub.publish(0.0)
            rospy.loginfo("Found person overlapping right lane -> slowing down")
        else:
            self.clearPub.publish()
            rospy.loginfo("No person in road")

        return 

    def in_left_lane(self, slope, yint, rectangles):
        in_lane = False
        on_road = False

        # rectangles have the following parameters
        # xAyA---------
        # |           |
        # |           |
        # |           |
        # ---------xByB
        for (xA, yA, xB, yB) in rectangles:
            # bottom right corner inside left line -> pedestrian is partly in the road 
            if(yB > (slope*xB + yint)):
                # pedestrian is in lane
                # keep searching to make sure a pedstrian is not in the road
                in_lane = True
                # top left corner inside left lane -> pedestrian is completly on the road 
                if(yA > (slope*xA + yint)):
                    on_road = True
                    break
        return (in_lane, on_road)

    def in_right_lane(self, slope, yint, rectangles):
        in_lane = False
        on_road = False

        # for rectangles found in object detect determine if they're between the lines and therefore on the road 
        # rectangles have the following parameters
        # xAyA---------
        # |           |
        # |           |
        # |           |
        # ---------xByB
        for (xA, yA, xB, yB) in rectangles:
            # bottom left corner inside right line -> pedestrian is partly in the road
            if(yB > (slope*xA + yint)):
                # pedestrian is in lane
                # keep searching to make sure a pedstrian is not in the road
                in_lane = True

            # top right corner inside left lane -> pedestrian is completly on the road
            if(yA > (slope*xB + yint)):
                on_road = True
                break

        return (in_lane, on_road)

    def get_lines(self, lines):
        self.sem.acquire()
        self.left_line_info = lines.lane_lines[0]
        self.right_line_info = lines.lane_lines[1]
        self.sem.release()

        


