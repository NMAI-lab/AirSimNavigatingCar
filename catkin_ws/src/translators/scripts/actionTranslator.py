#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String, Bool
import re
from time import sleep

enable = True
compassAngle = 0
speedSetting = 0

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
            # Extract the action parameter between the brackets
            parameter = re.search('\((.*)\)', action).group(1)

            # Disable LKA
            lkaEnablePub.publish(Bool(False))
            global compassAngle
            initialCompassAngle = compassAngle

            if ('left' in parameter) or ('right' in parameter):
                
                if 'left' in parameter:
                    turnAngle = -0.5
                else:
                    turnAngle = 0.4
                        

                rospy.loginfo('Turning ' + parameter)  

                # Send the turn action --- SWAP THIS FOR IMPLEMENTING THE TURN
                speedPublisher.publish(Float64(0.0))
                sleep(1)
                speedPublisher.publish(Float64(8.0))
                rospy.loginfo("compasAngle start: " + str(initialCompassAngle))
                
                while not turnComplete(initialCompassAngle, compassAngle):
                    steeringPub.publish(Float64(turnAngle))
                
                rospy.loginfo("compasAngle finish: " + str(initialCompassAngle))
                steeringPub.publish(Float64(0.0))
                
                speedPublisher.publish(Float64(speedSetting))
                sleep(3)
                
                rospy.loginfo('Turn complete')           
            
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
    proximityLimit = 60
    delta = (newAngle - initialAngle + 180) % 360 - 180
    
    if delta > proximityLimit:
        rospy.loginfo("delta " + str(delta))
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
