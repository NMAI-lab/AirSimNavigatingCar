#!/usr/bin/env python

# Created on 3 July 2020
# @author: Patrick Gavigan 

#import setup_path 
import airsim
import rospy
from navigation.msg import GPS

#from Calculations import getBearing, getDistance#, getMagneticBearing, getCompassAngle, getBearing

#Location,	Name,					Latitude,			Longitude,
#A,			Eary Approach, 			47.641482370883864,	-122.14036499180827,
#B,			Approach intersection,	47.64254900577103,	-122.14036358759651,
#C,			Turn right,				47.64261369134199,	-122.14022863966349,
#D,			Right, End of Street,	47.642632115856806,	-122.13892325834075,
#E,			Turn Left,				47.642635167315994,	-122.14049925386175,
#F, 		Left, end of street,	47.642634517703016,	-122.14203898419318,



def gpsSensor():
    # connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()


    #destination = (47.64254900577103, -122.14036358759651)
    
    pub = rospy.Publisher('sensor/gps', GPS, queue_size=1)
    rospy.init_node('gpsSensor', anonymous=True)
    rate = rospy.Rate(60) # Hz

    while not rospy.is_shutdown():

        data = client.getGpsData()

        message = GPS()
    
        # altitude = data.gnss.geo_point.altitude
        message.latitude = data.gnss.geo_point.latitude
        message.longitude = data.gnss.geo_point.longitude
        
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        gpsSensor()
    except rospy.ROSInterruptException:
        pass