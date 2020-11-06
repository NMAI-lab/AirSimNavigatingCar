#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String

# Set speed as a global
speed = 0.0


# Callback function for updatign the global speed
def updateSpeed(data):
    global speed
    speed = data.data

# Callback function for updating the navigation message. Will also grap the 
# global speed and publish all as perceptions
def updateNav(data, args):
    (publisher) = args
    navMessage = data.data
    
    global speed
    message = navMessage + " speed(" + str(speed) + ")"
    
    publisher.publish(message)
    rospy.loginfo("Perception: " + message)
    

# Main execution
def rosMain():
    perceptionPub = rospy.Publisher('perceptions', String, queue_size=1)
    

    rospy.init_node('perceptionTranslator', anonymous=True)
    
    rospy.Subscriber('sensor/navigation', String, updateNav, (perceptionPub))
    rospy.Subscriber('sensor/speed', Float64, updateSpeed)
    
    rospy.spin()

# Start things up
if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass