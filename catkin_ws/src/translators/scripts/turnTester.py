#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool, Float64
from navigation.msg import GPS
import nvector as nv
from time import sleep
import math

# Globals for the current location and direction of the car
currentBearing = 0
currentPosition = (0,0)
speed = 0
wgs84 = nv.FrameE(name='WGS84')


def updateSpeed(data):
    global speed
    speed = data.data


def updatePosition(data):
    global currentPosition
    global wgs84
    currentPosition = wgs84.GeoPoint(latitude=data.latitude, longitude=data.longitude, degrees = True)


def updateCompass(data):
    global currentBearing
    currentBearing = data.data


def setAngleRange180(angle):
    while angle < -180:
        angle = angle + 360
            
    while angle > 180:
        angle = angle - 360
    
    return angle


def getSteeringAngle(targetBearing, currentBearing):
    
    targetBearing = setAngleRange180(targetBearing)
    currentBearing = setAngleRange180(currentBearing)
    
    diff = currentBearing - targetBearing
    
    
    
    # Return in radians
    steeringAngle = math.radians(diff)   # See what happens (likely unstable)
    return steeringAngle

    
    

# Main execution
def directTo(steeringPub, speedPub, targetPosition):
    global currentPosition
    global currentBearing
    global speed
    
    delta = targetPosition.delta_to(currentPosition)
    targetRange = delta.length[0]
    
    steeringAngle = 0
    speedSetting = 6
    steeringPub.publish(Float64(steeringAngle))
    speedPub.publish(Float64(speedSetting))  
    
    # While range to target > 5
    while targetRange > 10:
        delta = targetPosition.delta_to(currentPosition)
        targetRange = delta.length[0]
        targetBearing = delta.azimuth_deg[0]
        steeringAngle = getSteeringAngle(targetBearing, currentBearing)        

        rospy.loginfo("Steering angle (RAD): " + str(steeringAngle))
        rospy.loginfo("Steering angle (DEG): " + str(math.degrees(steeringAngle)))

        steeringAngle = 0
        steeringPub.publish(Float64(steeringAngle))
        
        
        if speed <= 0:
            speedPub.publish(Float64(speedSetting)) 
            rospy.loginfo("Setting speed: " + str(speedSetting))
        
        
    rospy.loginfo("Arrived")
    # Stop the car
    for i in range(1,20):
        steeringPub.publish(Float64(0))
        speedPub.publish(Float64(0))
        
    rospy.loginfo("Stopped")


def rosMain():
    #lkaEnablePub = rospy.Publisher('lka/enable', Bool, queue_size=1)
    steeringPub = rospy.Publisher('action/steering', Float64, queue_size=1)
    speedPublisher = rospy.Publisher('action/setSpeed', Float64, queue_size=1)
    rospy.init_node('turner', anonymous=True)
    #rospy.Subscriber('action/turn', String, performTurn, (lkaEnablePub, steeringPub, speedPublisher))
    rospy.Subscriber('sensor/compass', Float64, updateCompass)
    rospy.Subscriber('sensor/gps', GPS, updatePosition)
    rospy.Subscriber('sensor/speed', Float64, updateSpeed)

    sleep(5) 

    # Test the method for directing the car somewhere using a GPS coordinate
    
    targetPosition = wgs84.GeoPoint(latitude=47.6426242556, longitude=-122.140354517, degrees = True)
    directTo(steeringPub, speedPublisher, targetPosition)

    rospy.loginfo("Second leg")
    targetPosition = wgs84.GeoPoint(latitude=47.642632115856806, longitude=-122.13892325834075, degrees = True)    
    directTo(steeringPub, speedPublisher, targetPosition)

# Start things up
if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
