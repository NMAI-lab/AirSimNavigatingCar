#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from navigation.msg import GPS
    
# Global variables to remember current and previous qr codes
previous = (0,0)
current = (0,0)

# Translate the line sensor data into a perception and publish
def translatePerception(data, args):
    # Extract the publisher and the message data
    (perceptionPublisher, postPointPublisher) = args
    position = (data.latitude, data.longitude)
    
    # Get access to the global variables (a bit hacky)
    global previous
    global current
    
    # Check if the post point changed, update history if necessary
    if position != current:
        previous = current
        current = position

    # Publish the perception
    postPoint = "gps({},{})".format(current, previous)
    rospy.loginfo(postPoint)
    #postPointPublisher.publish(postPoint)
    perceptionPublisher.publish(postPoint)

# Initialize the node, setup the publisher and subscriber
def rosMain():
    rospy.init_node('positionTranslator', anonymous=True)
    perceptionPublisher = rospy.Publisher('perceptions', String, queue_size=10)
    postPointPublisher = rospy.Publisher('postPoint', String, queue_size=10)
    rospy.Subscriber('sensor/gps', GPS, translatePerception, (perceptionPublisher, postPointPublisher))
    rospy.spin()

if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
