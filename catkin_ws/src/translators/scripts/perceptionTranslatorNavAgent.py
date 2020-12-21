#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from navigation.msg import GPS
from std_msgs.msg import Float64
    
#position = (0,0)
#compass = 0
#speed = 0

# Translate the line sensor data into a perception and publish
def translateGPS(data, args):
    # Extract the publisher and the message data
    (perceptionPublisher) = args
    
    #global position
    position = (data.latitude, data.longitude)
    

    # Publish the perception
    gpsPerception = "gps{}".format(position)
    rospy.loginfo(gpsPerception)
    perceptionPublisher.publish(gpsPerception)

def translateCompass(data, args):
    # Extract the publisher and the message data
    (perceptionPublisher) = args
    
    #global position
    compass = data.data    

    # Publish the perception
    compassPerception = "compass({})".format(compass)
    rospy.loginfo(compassPerception)
    perceptionPublisher.publish(compassPerception)

def translateSpeed(data, args):
    # Extract the publisher and the message data
    (perceptionPublisher) = args
    
    #global position
    speed = data.data    

    # Publish the perception
    speedPerception = "speed({})".format(speed)
    rospy.loginfo(speedPerception)
    perceptionPublisher.publish(speedPerception)
    

# Initialize the node, setup the publisher and subscriber
def rosMain():
    rospy.init_node('positionTranslator', anonymous=True)
    perceptionPublisher = rospy.Publisher('perceptions', String, queue_size=10)
    rospy.Subscriber('sensor/gps', GPS, translateGPS, (perceptionPublisher))
    rospy.Subscriber('sensor/compass', Float64, translateCompass, (perceptionPublisher))
    rospy.Subscriber('sensor/speed', Float64, translateSpeed, (perceptionPublisher))
    rospy.spin()

if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
