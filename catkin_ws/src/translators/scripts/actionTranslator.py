#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String, Bool
import re

enable = True
compassAngle = 0

# Decode and execute the action
def decodeAction(data, args):
    global enable
    if enable:              # Simple mutex
        enable = False
        
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
        
        elif 'turn' in action:
            # Extract the action parameter between the brackets
            parameter = re.search('\((.*)\)', action).group(1)

            # Disable LKA
            lkaEnablePub.publish(Bool(False))
            global compassAngle
            initialCompassAngle = compassAngle

            if ('left' in parameter) or ('right' in parameter):
                turnAngle = 0.69    # RAD (40 deg)
                
                if 'left' in parameter:
                    turnAngle = turnAngle * -1


                # Send the turn action --- SWAP THIS FOR IMPLEMENTING THE TURN
                speedPublisher.publish(Float64(8.0))
                while not turnComplete(initialCompassAngle, compassAngle):
                    steeringPub.publish(Float64(turnAngle))
                steeringPub.publish(Float64(0.0))
                speedPublisher.publish(Float64(0.0))
                
                rospy.loginfo('Turning ' + parameter)           
            
            else:
                rospy.loginfo('Bad turn direction: ' + parameter)
        
            # Enable LKA
            lkaEnablePub.publish(Bool(True))
        
        elif 'setDestination' in action:
            # Extract the action parameter between the brackets
            parameter = re.search('\((.*)\)', action).group(1)
            destinationPublisher.publish(str(parameter))
            rospy.loginfo('Setting destination: ' + str(parameter))

        else:
            rospy.loginfo("Received unsupported action: " + str(action))
            
        enable = True   # Release the mutex
        
def updateCompass(data):
    global compassAngle
    compassAngle = data.data  

def turnComplete(initialAngle, newAngle):
    delta = abs(newAngle - initialAngle)
    if delta > 80:
        return True
    else:
        return False
        
# Main execution
def rosMain():
    lkaEnablePub = rospy.Publisher('lka/enable', Bool, queue_size=1)
    steeringPub = rospy.Publisher('action/steering', Float64, queue_size=1)    # Hack into the steering angle (for now)
    speedPublisher = rospy.Publisher('action/setSpeed', Float64, queue_size=1)
    destinationPublisher = rospy.Publisher('setDestination', String, queue_size=1)
    rospy.init_node('actionTranslator', anonymous=True)
    rospy.Subscriber('actions', String, decodeAction, (lkaEnablePub, steeringPub, speedPublisher, destinationPublisher))
    rospy.Subscriber('sensor/compass', Float64, updateCompass)

    rospy.spin()

# Start things up
if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass