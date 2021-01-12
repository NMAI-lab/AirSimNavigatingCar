#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String
import re

enable = True
'''
def updateCompass(data):
    global currentBearing
    declanation = 7.5#10.3881839942
    compassReading = data.data
    currentBearing = compassReading + declanation
    

    
def calculateSteering(courseCorrection):
    if abs(courseCorrection) < 20:
        return courseCorrection/180
    else:
        if courseCorrection > 0:
            return 1
        else:
            return -1
'''
# Decode and execute the action
def decodeAction(data, args):
    global enable
    
    if enable:              # Simple mutex
        enable = False

        # Get the parameters
        action = str(data.data)
        (steeringPublisher, speedPublisher, positionPublisher, destinationPublisher) = args
        
        rospy.loginfo('Action: ' + action)

        # Extract the action parameter between the brackets
        parameter = re.search('\((.*)\)', action).group(1)
        while "(" in parameter:
            parameter = re.search('\((.*)\)', action).group(1)
    
        # Handle the docking station cases
        if 'setSpeed' in action:
    
            # Set the new speed    
            speedPublisher.publish(Float64(float(parameter)))
            rospy.loginfo('Setting speed: ' + parameter)
        
        elif 'steering' in action:
            steeringPublisher.publish(Float64(float(parameter)))
            rospy.loginfo('Setting steering: ' + parameter)    
            
            # Action format: getPath(Current,Destination)
        elif "getPath" in action:
            parameter = parameter.replace(" ","")
            parameterList = parameter.split(",")
        
            current = parameterList[0]
            rospy.loginfo("Position: " + str(current))
            positionPublisher.publish(current)
        
            destination = parameterList[1]
            rospy.loginfo("Destination: " + str(destination))
            destinationPublisher.publish(destination)
        
        else:
            rospy.loginfo("Received unsupported action: " + str(action))
            
        enable = True   # Release the mutex
        rospy.loginfo("Action mutex released")
    else:
        # Clear the buffer
        action = str(data.data)
        
# Main execution
def rosMain():
    # Setup publishers
    positionPublisher = rospy.Publisher('navigation/position', String, queue_size=10)    
    destinationPublisher = rospy.Publisher('navigation/destination', String, queue_size=10)
    steeringPublisher = rospy.Publisher('action/steering', Float64, queue_size=1)    # Hack into the steering angle (for now)
    speedPublisher = rospy.Publisher('action/setSpeed', Float64, queue_size=1)

    rospy.init_node('actionTranslator', anonymous=True)
    rospy.Subscriber('actions', String, decodeAction, (steeringPublisher, speedPublisher, positionPublisher, destinationPublisher))

    rospy.spin()

# Start things up
if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
