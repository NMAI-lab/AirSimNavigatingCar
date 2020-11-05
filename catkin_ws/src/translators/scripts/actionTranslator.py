#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String
import re


# Decode and execute the action
def decodeAction(data, args):
        
    # Get the parameters
    action = str(data.data)
    
    (speedPublisher, turnPublisher, destinationPublisher) = args
    rospy.loginfo('Action: ' + action)
    
    # Handle the docking station cases
    if 'setSpeed' in action:
        # Extract the action parameter between the brackets
        parameter = re.search('\((.*)\)', action).group(1)
    
        # Set the new speed    
        speedPublisher.publish(float(parameter))
        rospy.loginfo('Setting speed: ' + parameter)
        
    elif 'turn' in action:
        # Extract the action parameter between the brackets
        parameter = re.search('\((.*)\)', action).group(1)

        if 'left' in parameter:
             # Send the turn action --- SWAP THIS FOR IMPLEMENTING THE TURN
            turnPublisher.publish(str(parameter))
            rospy.loginfo('Turning ' + parameter)           
            
        elif 'right' in parameter:
            # Send the turn action --- SWAP THIS FOR IMPLEMENTING THE TURN
            turnPublisher.publish(str(parameter))
            rospy.loginfo('Turning ' + parameter)
        else:
            rospy.loginfo('Bad turn direction: ' + parameter)
        
    elif 'setDestination' in action:
        # Extract the action parameter between the brackets
        parameter = re.search('\((.*)\)', action).group(1)
        destinationPublisher.publish(str(parameter))
        rospy.loginfo('Setting destination: ' + str(parameter))

    else:
        rospy.loginfo("Received unsupported action: " + str(action))
        
        
# Main execution
def rosMain():
    speedPublisher = rospy.Publisher('action/setSpeed', Float64, queue_size=1)
    destinationPublisher = rospy.Publisher('setDestination', String, queue_size=1)
    turnPublisher = rospy.Publisher('action/turn', String, queue_size=1)
    rospy.init_node('actionTranslator', anonymous=True)
    rospy.Subscriber('actions', String, decodeAction, (speedPublisher, turnPublisher, destinationPublisher))

    rospy.spin()

# Start things up
if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
