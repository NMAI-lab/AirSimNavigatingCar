#!/usr/bin/env python
import cv2 as cv
import numpy as np

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Canny edge detection with Hough transform provided by https://towardsdatascience.com/tutorial-build-a-lane-detector-679fd8953132#127c =============================================
def do_canny(frame):
    # Converts frame to grayscale because we only need the luminance channel for detecting edges - less computationally expensive
    gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
    # Applies a 5x5 gaussian blur with deviation of 0 to frame - not mandatory since Canny will do this for us
    blur = cv.GaussianBlur(gray, (5, 5), 0)
    # Applies Canny edge detector with minVal of 50 and maxVal of 150
    canny = cv.Canny(blur, 50, 150)
    return canny

def do_segment(frame):
    # Since an image is a multi-directional array containing the relative intensities of each pixel in the image, we can use frame.shape to return a tuple: [number of rows, number of columns, number of channels] of the dimensions of the frame
    # frame.shape[0] give us the number of rows of pixels the frame has. Since height begins from 0 at the top, the y-coordinate of the bottom of the frame is its height
    height = frame.shape[0]
    # Creates a pentagon polygon for the mask defined by three (x, y) coordinates
    polygons = np.array([
                            [(0, height - 100), (0, height), (640, height), (640, height - 100), (320, 150)]
                        ])
    # Creates an image filled with zero intensities with the same dimensions as the frame
    mask = np.zeros_like(frame)
    # Allows the mask to be filled with values of 1 and the other areas to be filled with values of 0
    cv.fillPoly(mask, polygons, 255)
    # A bitwise and operation between the mask and frame keeps only the triangular area of the frame
    segment = cv.bitwise_and(frame, mask)
    return segment

def calculate_lines(frame, lines):
    # Empty arrays to store the coordinates of the left and right lines
    left = []
    right = []
    # Loops through every detected line
    for line in lines:
        # Reshapes line from 2D array to 1D array
        x1, y1, x2, y2 = line.reshape(4)
        # Fits a linear polynomial to the x and y coordinates and returns a vector of coefficients which describe the slope and y-intercept
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        y_intercept = parameters[1]
        # If slope is negative, the line is to the left of the lane, and otherwise, the line is to the right of the lane
        if slope < 0:
            left.append((slope, y_intercept))
        else:
            right.append((slope, y_intercept))
    # Averages out all the values for left and right into a single slope and y-intercept value for each line
    left_avg = np.average(left, axis = 0)
    right_avg = np.average(right, axis = 0)
    return left_avg, right_avg

## ==============================================================================================================================================================================


def processImage(img):
    bridge  = CvBridge()
    steering = 0.0

    try:
        cv_img = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
    except CvBridgeError as e:
        rospy.loginfo(e)
    if img != None:
        canny = do_canny(cv_img)
        segment = do_segment(canny)
        hough = cv.HoughLinesP(segment, 2, np.pi / 180, 100, np.array([]), minLineLength = 100, maxLineGap = 50)
        # Averages multiple detected lines from hough into one line for left border of lane and one line for right border of lane
        left_line, right_line = calculate_lines(cv_img, hough)
        x_intersection = (left_line[1] - right_line[1])/(right_line[0] - left_line[0])
        rospy.loginfo(x_intersection)
        steering = (x_intersection / 320) - 1
        rospy.loginfo(steering)
        steeringPub.publish(steering)

        # cv.imshow("seg", segment)
        # cv.waitKey(1)



def listener():
    global steeringPub
    rospy.init_node('lka', anonymous=True)
    steeringPub = rospy.Publisher('control/steering', Float64, queue_size=1)
    rospy.Subscriber('airsim/image_raw', Image, processImage)
    rospy.spin()

if __name__ == "__main__":
    listener()