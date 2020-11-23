#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Float64
#from navigation.msg import GPS
import nvector as nv
import re

# Globals for the current location and direction of the car
compass = 0
position = (0,0)
wgs84 = nv.FrameE(name='WGS84')
    
def updatePosition(data):
    global position
    global wgs84
    position = wgs84.GeoPoint(latitude=data.latitude, longitude=data.longitude, degrees = True)

def updateCompass(data):
    global compass
    compass = data.data
    
def calculateSteering(courseCorrection):

    if abs(courseCorrection) < 20:
        return courseCorrection/180
    else:
        if courseCorrection > 0:
            return 1
        else:
            return -1

def directTo(steeringPub, speedPub, targetPosition):
    global currentPosition
    global currentBearing
    global speed
    
    delta = currentPosition.delta_to(targetPosition)
    targetRange = delta.length[0]
    
    speedSetting = 6
 
    # While range to target > 5
    while targetRange > 10:
        delta = currentPosition.delta_to(targetPosition)
        targetRange = delta.length[0]
        targetBearing = delta.azimuth_deg[0]
        
        courseCorrection = targetBearing - currentBearing        
        steering = calculateSteering(courseCorrection)
        rospy.loginfo("course correction: " + str(courseCorrection))
        rospy.loginfo("steering: " + str(steering))
         
        steeringPub.publish(Float64(steering))
        
        
        if speed <= 0:
            speedPub.publish(Float64(speedSetting)) 
            rospy.loginfo("Setting speed: " + str(speedSetting))
            
            
def turnToward(steeringPub, speedPub, targetPosition):
    global currentPosition
    global currentBearing
    global speed
    
    startPosition = currentPosition
    
    startDelta = currentPosition.delta_to(startPosition)
    startRange = startDelta.length[0]
    
    targetDelta = currentPosition.delta_to(targetPosition)
    targetRange = targetDelta.length[0]
    targetBearing = targetDelta.azimuth_deg[0]
    
    speedSetting = 6
 
    # Perform the turn
    while ((abs(startRange) < 10) and (abs(targetBearing) > 10)) or (abs(targetRange) < 10):
        targetDelta = currentPosition.delta_to(targetPosition)
        targetBearing = targetDelta.azimuth_deg[0]
        
        startDelta = currentPosition.delta_to(startPosition)
        startRange = startDelta.length[0]
        
        courseCorrection = targetBearing - currentBearing        
        steering = calculateSteering(courseCorrection)
        rospy.loginfo("course correction: " + str(courseCorrection))
        rospy.loginfo("steering: " + str(steering))
         
        steeringPub.publish(Float64(steering))
        
        if speed <= 0:
            speedPub.publish(Float64(speedSetting)) 
            rospy.loginfo("Setting speed: " + str(speedSetting))
        
    rospy.loginfo("On track")


def performTurn(command, lkaEnablePub, steeringPub, speedPub):
    rospy.loginfo("Command: " + str(command))
    
    # Command structure: turn(AT, TO)
    # AT: The location where the turn needs to happen
    # TO: A location that we need to turn toward
    parameter = re.search('\((.*)\)', command).group(1)
    at = parameter.split(',')[0]
    to = parameter.split(',')[1]
    
    # Assume control over speed and steering: disable LKA
    lkaEnablePub.publish(Bool(False))
    
    # Approach: Direct to the intersection
    directTo(steeringPub, speedPub, at)
    
    # Perform the turn at the intersection
    turnToward(steeringPub, speedPub, to)
    
    # Release control once clear
    lkaEnablePub.publish(True)
