#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String
import re


# Decode and execute the action
def decodeAction(data, args):
        
    # Get the parameters
    action = str(data.data)
    
    (speedPublisher, turnPublisher) = args
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

        if ('left' in parameter) or ('right' in parameter):
            # Send the turn action
            turnPublisher.publish(str(parameter))
            rospy.loginfo('Turning ' + parameter)
        else:
            rospy.loginfo('Bad turn direction: ' + parameter)
        
    else:
        rospy.loginfo("Received unsupported action: " + str(action))
        
        
# Main execution
def rosMain():
    speedPublisher = rospy.Publisher('sensor/setSpeed', Float64, queue_size=1)
#    destinationPublisher = rospy.Publisher('setDestination', String, queue_size=1)
    turnPublisher = rospy.Publisher('action/turn', String, queue_size=1)
    rospy.init_node('actionTranslator', anonymous=True)
    rospy.Subscriber('actions', String, decodeAction, (speedPublisher, turnPublisher))

    rospy.spin()

# Start things up
if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
