#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from navigation.msg import GPS
    
# Global variables to remember current and previous qr codes
position = (0,0)

# Translate the line sensor data into a perception and publish
def translatePerception(data, args):
    # Extract the publisher and the message data
    (perceptionPublisher) = args
    
    global position
    position = (data.latitude, data.longitude)
    

    # Publish the perception
    gpsPerception = "gps({})".format(position)
    rospy.loginfo(gpsPerception)
    #postPointPublisher.publish(postPoint)
    perceptionPublisher.publish(gpsPerception)

# Initialize the node, setup the publisher and subscriber
def rosMain():
    rospy.init_node('positionTranslator', anonymous=True)
    perceptionPublisher = rospy.Publisher('perceptions', String, queue_size=10)
    rospy.Subscriber('sensor/gps', GPS, translatePerception, (perceptionPublisher))
    rospy.spin()

if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
