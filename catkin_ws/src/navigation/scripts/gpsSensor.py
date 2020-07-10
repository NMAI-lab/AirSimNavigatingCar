#!/usr/bin/env python

# Created on 3 July 2020
# @author: Patrick Gavigan 

#import setup_path 
import airsim

#import numpy as np
#import os
#import tempfile
#import pprint
#import cv2

import rospy
from navigation.msg import GPS

def gpsSensor():
    # connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()


    pub = rospy.Publisher('sensor/gps', GPS, queue_size=1)
    rospy.init_node('gpsSensor', anonymous=True)
    rate = rospy.Rate(2) # 2 Hz

    while not rospy.is_shutdown():

        data = client.getGpsData()
        message = GPS()
    
        # altitude = data.gnss.geo_point.altitude
        message.latitude = data.gnss.geo_point.latitude
        message.longitude = data.gnss.geo_point.longitude
        message.bearing = 0
        message.distance = 0

        #velocity = (data.gnss.velocity.x_val,
        #            data.gnss.velocity.y_val,
        #            data.gnss.velocity.z_val)

        #print("(altitude, latitude, longitude): " + str(position))
        #print("velocity: " + str(velocity))
        
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        gpsSensor()
    except rospy.ROSInterruptException:
        pass