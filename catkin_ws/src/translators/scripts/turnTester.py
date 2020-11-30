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
    declanation = 10.3881839942
    compassReading = data.data
    currentBearing = compassReading + declanation


def setAngleRange180(angle):
    while angle < -180:
        angle = angle + 360
            
    while angle > 180:
        angle = angle - 360
    
    return angle


def calculateSteering(courseCorrection):

    if abs(courseCorrection) < 20:
        return courseCorrection/180
    else:
        if courseCorrection > 0:
            return 1
        else:
            return -1


# Main execution
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
        
        
    rospy.loginfo("Arrived")
    # Stop the car
    while speed > 0:
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

    global currentPosition
    while currentPosition == (0,0):   # Wait for the first GPS update
        sleep(1)

    # Test the method for directing the car somewhere using a GPS coordinate
    post1 = wgs84.GeoPoint(latitude=47.641482370883864, longitude=-122.14036499180827, degrees = True)
    post2 = wgs84.GeoPoint(latitude=47.6426242556, longitude=-122.140354517, degrees = True)
    post3 = wgs84.GeoPoint(latitude=47.642632115856806, longitude=-122.13892325834075, degrees = True)
    post4 = wgs84.GeoPoint(latitude=47.642634517703016, longitude=-122.14203898419318, degrees = True)
    
    targetPosition = wgs84.GeoPoint(latitude=47.6426242556, longitude=-122.140354517, degrees = True)
    directTo(steeringPub, speedPublisher, targetPosition)

    rospy.loginfo("Second leg")
    targetPosition = post3
    directTo(steeringPub, speedPublisher, targetPosition)

def testMath():
    compassReading = -10.3881839942
    declanation = 10.3881839942
    currentBearing = compassReading + declanation
    
    wgs84 = nv.FrameE(name='WGS84')
    
    post1 = wgs84.GeoPoint(latitude=47.641482370883864, longitude=-122.14036499180827, degrees = True)
    post2 = wgs84.GeoPoint(latitude=47.6426242556, longitude=-122.140354517, degrees = True)
    post3 = wgs84.GeoPoint(latitude=47.642632115856806, longitude=-122.13892325834075, degrees = True)
    post4 = wgs84.GeoPoint(latitude=47.642634517703016, longitude=-122.14203898419318, degrees = True)
        
   
    currentPosition = post1
    targetPosition = post2

    
    
    delta = currentPosition.delta_to(targetPosition)
    targetRange = delta.length[0]
    targetBearing = delta.azimuth_deg[0]
    
    courseCorrection = targetBearing - currentBearing
    
    print("Compass angle: " + str(currentBearing))
    print("Target bearing: " + str(targetBearing))
    print("Target range: " + str(targetRange))
    
    steering = calculateSteering(courseCorrection)
    
    print("Sample steering angle: " + str(steering))
    
    
    # Straight ahead:
    # -10.3881839942

    # Need to turn to the left:
    # 60

    # Need to turn to the right:
    # -60

# Start things up
if __name__ == '__main__':
    try:
        rosMain()
    #testMath()
    except rospy.ROSInterruptException:
        pass



