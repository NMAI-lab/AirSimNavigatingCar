#!/usr/bin/env python
import cv2 as cv
import numpy as np

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from imutils.object_detection import non_max_suppression
import imutils 

from object_avoid import AvoidPedestrians

from lka.msg import Lane
from lka.msg import Lanes

"""
Class for pedestrian detection 
"""      
class DetectPedestrians():

    def __init__(self, avoid_class):
        self.hog = cv.HOGDescriptor()
        self.hog.setSVMDetector(cv.HOGDescriptor_getDefaultPeopleDetector())
        self.avoid = avoid_class
        
    def getImage(self, img):
        bridge  = CvBridge()

        try:
            cv_img = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.loginfo(e)
        
        return cv_img

    def processImage(self, image):
        image_grey = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        image_grey = cv.equalizeHist(image_grey)
        return image_grey
         
    def detectAndDisplay(self, img):
        image = self.getImage(img)
        # resize image to improve the accuracy and decrease detection time 
        image = imutils.resize(image, width=min(400, image.shape[1]))
        # detect people in the image
        # winStride indicates the step size for the boxes that search the image
        # padding indicates the number of pixels added to pad the sliding boxes 
        # scale indicates the scale of the image 
        (rects, weights) = self.hog.detectMultiScale(image, winStride=(4, 4),
            padding=(8, 8), scale=1.05)

        # apply non-maxima suppression to the bounding boxes using a
        # fairly large overlap threshold to try to maintain overlapping
        # boxes that are still people
        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
        # # draw the final bounding boxes
        # for (xA, yA, xB, yB) in pick:
        #     cv.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
        # # display image
        # cv.imshow("After NMS", image)
        # cv.waitKey(1)

        self.avoid.objects_in_road(pick)

## ==============================================================================================================================================================================

def listener():
    #global detect_ped 
    #global avoid_ped 
    rospy.init_node('object_detect', anonymous=True)
    avoid_ped = AvoidPedestrians()
    detect_ped = DetectPedestrians(avoid_ped)
    rospy.Subscriber('airsim/image_raw', Image, detect_ped.detectAndDisplay)
    rospy.Subscriber('lka/lanes', Lanes, detect_ped.avoid.get_lines)
    rospy.spin()

if __name__ == "__main__":
    listener()