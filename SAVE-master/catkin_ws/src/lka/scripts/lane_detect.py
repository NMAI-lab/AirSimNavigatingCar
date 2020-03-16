#!/usr/bin/env python
import cv2 as cv
import numpy as np

import rospy

# Canny edge detection with Hough transform provided by https://towardsdatascience.com/tutorial-build-a-lane-detector-679fd8953132#127c =============================================
def do_canny(frame):
    # Converts frame to grayscale because we only need the luminance channel for detecting edges - less computationally expensive
    gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
    # Applies a 5x5 gaussian blur with deviation of 0 to frame - not mandatory since Canny will do this for us
    blur = cv.GaussianBlur(gray, (5, 5), 0)
    # Applies Canny edge detector with minVal of 50 and maxVal of 150
    canny = cv.Canny(blur, 25, 100)
    return canny

def do_segment(frame, size):
    # Since an image is a multi-directional array containing the relative intensities of each pixel in the image, we can use frame.shape to return a tuple: [number of rows, number of columns, number of channels] of the dimensions of the frame
    # frame.shape[0] give us the number of rows of pixels the frame has. Since height begins from 0 at the top, the y-coordinate of the bottom of the frame is its height
    height = size[0]
    width  = size[1]
    # Creates a pentagon polygon for the mask defined by three (x, y) coordinates
    polygons = np.array([
                            [(0, height - 100), (0, height), (width, height), (width, height - 100), (width/2, 150)]
                        ])
    # Creates an image filled with zero intensities with the same dimensions as the frame
    mask = np.zeros_like(frame)
    # Allows the mask to be filled with values of 1 and the other areas to be filled with values of 0
    cv.fillPoly(mask, polygons, 255)
    # A bitwise and operation between the mask and frame keeps only the triangular area of the frame
    segment = cv.bitwise_and(frame, mask)
    return segment

def calculate_lines(frame, lines):
    left_avg  = []
    right_avg = []
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
        abs_slope = abs(slope)
        y_intercept = parameters[1]
        # If slope is negative, the line is to the left of the lane, and otherwise, the line is to the right of the lane
        if abs_slope > 0.3:
            if slope < 0:
                left.append((slope, y_intercept))
            else:
                right.append((slope, y_intercept))
    # Averages out all the values for left and right into a single slope and y-intercept value for each line
    if len(left) != 0:
        left_avg = np.average(left, axis = 0)
    if len(right) != 0:
        right_avg = np.average(right, axis = 0)

    return left_avg, right_avg

## ==============================================================================================================================================================================

def lane_detect(cv_img, size):
    canny = do_canny(cv_img)
    segment = do_segment(canny, size)
    hough = cv.HoughLinesP(segment, 1, np.pi/180, 100, np.array([]), minLineLength = 40, maxLineGap = 10)
    
    # Debug to show Hough Transform
    # for l in hough:
    #     line = l[0]
    #     cv.line(cv_img, (line[0],line[1]), (line[2],line[3]), (0,0,255), 2)

    # cv.imshow("hough", cv_img)
    # cv.waitKey(1)
    
    if not (hough is None):
        if len(hough) > 0:
            # Averages multiple detected lines from hough into one line for left border of lane and one line for right border of lane
            ret = calculate_lines(cv_img, hough)
    else:
        ret = None

    return ret