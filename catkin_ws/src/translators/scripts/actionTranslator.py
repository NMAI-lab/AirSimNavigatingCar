#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String, Bool
from navigation.msg import GPS
import re
import nvector as nv
import json

enable = True
currentBearing = 0
speedSetting = 0
speed = 0
wgs84 = nv.FrameE(name='WGS84')
currentPosition = wgs84.GeoPoint(latitude=0, longitude=0, degrees = True)
nodeLocations = 0

def updatePosition(data):
    global currentPosition
    global wgs84
    currentPosition = wgs84.GeoPoint(latitude=data.latitude, longitude=data.longitude, degrees = True)

def updateCompass(data):
    global currentBearing
    declanation = 7.5#10.3881839942
    compassReading = data.data
    currentBearing = compassReading + declanation
    
def updateSpeed(data):
    global speed
    speed = data.data
    
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
    while targetRange > 5:
        delta = currentPosition.delta_to(targetPosition)
        targetRange = delta.length[0]
        targetBearing = delta.azimuth_deg[0]
        
        courseCorrection = targetBearing - currentBearing        
        steering = calculateSteering(courseCorrection)
        #rospy.loginfo("course correction: " + str(courseCorrection))
        #rospy.loginfo("steering: " + str(steering))
         
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
    while ((abs(startRange) < 50) and (abs(targetBearing) > 5)) or (abs(targetRange) < 10):
        targetDelta = currentPosition.delta_to(targetPosition)
        targetBearing = targetDelta.azimuth_deg[0]
        
        startDelta = currentPosition.delta_to(startPosition)
        startRange = startDelta.length[0]
        
        courseCorrection = targetBearing - currentBearing        
        steering = calculateSteering(courseCorrection)
        #rospy.loginfo("course correction: " + str(courseCorrection))
        #rospy.loginfo("steering: " + str(steering))
         
        steeringPub.publish(Float64(steering))
        
        if speed <= 0:
            speedPub.publish(Float64(speedSetting)) 
            #rospy.loginfo("Setting speed: " + str(speedSetting))
        
    rospy.loginfo("On track")


def performTurn(command, lkaEnablePub, steeringPub, speedPub):
    rospy.loginfo("Command: " + str(command))
    
    # Command structure: turn(AT, TO)
    # AT: The location where the turn needs to happen
    # TO: A location that we need to turn toward
    parameter = re.search('\((.*)\)', command).group(1)
    at = parameter.split(',')[0]
    to = parameter.split(',')[1]
    
    # Validate locations
    global nodeLocations
    if at not in nodeLocations.keys():
        rospy.loginfo("AT LOCATION NOT FOUND")
        return
    if to not in nodeLocations.keys():
        rospy.loginfo("TO LOCATION NOT FOUND")
        return
    
    # Assume control over speed and steering: disable LKA
    lkaEnablePub.publish(Bool(False))
    rospy.loginfo("LKA Disabled")
    
    # Approach: Direct to the intersection
    directTo(steeringPub, speedPub, nodeLocations[at])
    
    # Perform the turn at the intersection
    turnToward(steeringPub, speedPub, nodeLocations[to])
    
    # Release control once clear
    lkaEnablePub.publish(True)
    rospy.loginfo("LKA Enabled")



# Decode and execute the action
def decodeAction(data, args):
    global enable
    
    if enable:              # Simple mutex
        enable = False

        global speedSetting

        # Get the parameters
        action = str(data.data)
        (lkaEnablePub, steeringPub, speedPublisher, destinationPublisher) = args
        
        rospy.loginfo('Action: ' + action)
    
        # Handle the docking station cases
        if 'setSpeed' in action:
            # Extract the action parameter between the brackets
            parameter = re.search('\((.*)\)', action).group(1)
    
            # Set the new speed    
            speedPublisher.publish(Float64(float(parameter)))
            rospy.loginfo('Setting speed: ' + parameter)

            speedSetting = float(parameter)
        
        elif 'turn' in action:
            performTurn(action, lkaEnablePub, steeringPub, speedPublisher)
            rospy.loginfo('Turn complete')           
        
        elif 'setDestination' in action:
            # Extract the action parameter between the brackets
            parameter = re.search('\((.*)\)', action).group(1)
            destinationPublisher.publish(str(parameter))
            rospy.loginfo('Setting destination: ' + str(parameter))

        else:
            rospy.loginfo("Received unsupported action: " + str(action))
            
        enable = True   # Release the mutex
        rospy.loginfo("Action mutex released")
    else:
        action = str(data.data)
        

def setupLocationLibrary():
    # Load node locations
    f = open('nodeLocations.json')
    global nodeLocations
    nodeLocations = json.load(f)
    
    # Update to use GPS objects for the locations in the dict
    global wgs84
    locations = nodeLocations.keys()
    for location in locations:
        latitude = nodeLocations[location][0]
        longitude = nodeLocations[location][1]
        nodeLocations[location] = wgs84.GeoPoint(latitude=latitude, longitude=longitude, degrees = True)

# Main execution
def rosMain():
    setupLocationLibrary()
    lkaEnablePub = rospy.Publisher('lka/enable', Bool, queue_size=1)
    steeringPub = rospy.Publisher('action/steering', Float64, queue_size=1)    # Hack into the steering angle (for now)
    speedPublisher = rospy.Publisher('action/setSpeed', Float64, queue_size=1)
    destinationPublisher = rospy.Publisher('setDestination', String, queue_size=1)
    rospy.init_node('actionTranslator', anonymous=True)
    rospy.Subscriber('actions', String, decodeAction, (lkaEnablePub, steeringPub, speedPublisher, destinationPublisher))
    rospy.Subscriber('sensor/compass', Float64, updateCompass)
    rospy.Subscriber('sensor/gps', GPS, updatePosition)
    rospy.Subscriber('sensor/speed', Float64, updateSpeed)

    rospy.spin()

# Start things up
if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
