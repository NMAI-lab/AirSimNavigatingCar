#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool, Float64
from navigation.msg import GPS
import nvector as nv

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

def directTo(location):
    print("direct to")

def performTurn(data, args):
    (lkaEnablePub, steeringPub, speedPublisher) = args
    command = data.data
    rospy.loginfo("Command: " + str(command))
    
    # Assume control over speed and steering: disable LKA
    lkaEnablePub.publish(Bool(False))
    
    # Approach: Slow and direct to the intersection
    speedPublisher.publish(Float64(1.0))
    # directTo(intersectionLocation)
    
    # Perform the turn at the intersection
    # turn(intersectionLocation, nextLocation) # Need some sort of proximity check here, or hard code the turn
    
    # Drive away from intersection
    speedPublisher.publish(Float64(8.0))        # Correct this from a hard coded value
    
    # Release control once clear
    lkaEnablePub.publish(True)
    
    
def rosMain():
    print("Setup turner")
    
    lkaEnablePub = rospy.Publisher('lka/enable', Bool, queue_size=1)
    steeringPub = rospy.Publisher('action/steering', Float64, queue_size=1)
    speedPublisher = rospy.Publisher('action/setSpeed', Float64, queue_size=1)
    rospy.init_node('turner', anonymous=True)
    rospy.Subscriber('action/turn', String, performTurn, (lkaEnablePub, steeringPub, speedPublisher))
    rospy.Subscriber('sensor/compass', Float64, updateCompass)
    rospy.Subscriber('sensor/gps', GPS, updatePosition)
    
if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass