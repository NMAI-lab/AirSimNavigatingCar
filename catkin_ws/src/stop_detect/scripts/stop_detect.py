#!/usr/bin/env python
import cv2 as cv
import numpy as np
import pytesseract
import rospy
from controls.msg import Control
import re
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import random
from imutils.object_detection import non_max_suppression
import imutils 
import airsim

from std_msgs.msg import Float64

from object_avoid import AvoidPedestrians

from lka.msg import Lane
from lka.msg import Lanes
from std_msgs.msg import String
# CLass for Stop Sign Detection

Done = False
pub = rospy.Publisher("request/msg",String,queue_size=10)

def cll(data):
    global Done
    b=DetectStopSign()
    if data == String("5"):
        b.brakePub.publish(1)
        time.sleep(5)
        Done = True

def control():
    sub = rospy.Subscriber("controller",String,cll)

class DetectStopSign():

    def __init__(self):
        self.brakePub = rospy.Publisher('object_avoid/brake',Float64,queue_size=1)

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

        # converting from BGR to HSV color space
        img_hsv=cv.cvtColor(image, cv.COLOR_BGR2HSV)

        #image = cv.Canny(img_hsv,100,200)
        mask1 = cv.inRange(img_hsv, (0,50,20), (5,255,255))
        mask2 = cv.inRange(img_hsv, (175,50,20), (180,255,255))

        mask = cv.bitwise_or(mask1, mask2)
        
        # Making the Boxes around Area of Interest
        bluecnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)[-2]
        blue_area = max(bluecnts, key=cv.contourArea)
        (x,y,w,h) = cv.boundingRect(blue_area)
        cv.rectangle(image,(x,y),(x+w, y+h),(0,255,0),2)
    
        # Canny on region of image
        text = image[y:y+h, x:x+w]
        a = imutils.resize(text, width=min(85, image.shape[1]))
        l = cv.Canny(a,100,295)
        # For Detecting the TEXT from the stop sign
        cv.waitKey(5)
        # TEMOVE THE COMMENT #
        text = pytesseract.image_to_string(l) 
        rospy.loginfo(text)
        print (text)
        #if re.search('[a-zA-Z]', text):
        if text:
            # Show Image Processing Results
            cv.imshow("Text",l)
            cv.imshow("img_hsv",image)
            # New Subsumption Code
            #rospy.init_node('obstacle_avoid')
            pub = rospy.Publisher("request",String,queue_size=1)
            print("REAHCED HERE ___________")
            requestClearance = "3"
            pub.publish(requestClearance)
            control()
            print("---------------",Done)
            if Done == True:
                reset = str("13")
                pub.publish(reset)
                print("RESET SET")

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
    detect_stop = DetectStopSign()
    #rospy.Subscriber('airsim/image_raw', Image, detect_ped.detectAndDisplay)
    rospy.Subscriber('airsim/image_raw', Image, detect_stop.detectAndDisplay)
    #rospy.Subscriber('lka/lanes', Lanes, detect_ped.avoid.get_lines)
    rospy.spin()


    while True:
        data_car1 = client.getDistanceSensorData(vehicle_name="Car1")
        data_car2 = client.getDistanceSensorData(vehicle_name="Car2")
        rospy.loginfo("Distance sensor data: Car1: {data_car1.distance}, Car2: {data_car2.distance}")
    client = airsim.CarClient()
    client.confirmConnection()

if __name__ == "__main__":
    listener()
