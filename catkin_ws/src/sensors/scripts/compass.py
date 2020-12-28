#!/usr/bin/env python
 
import airsim

import rospy
from std_msgs.msg import Float64
from numpy import arctan2
import math

def compass():
    pub = rospy.Publisher('sensor/compass', Float64, queue_size=1)
    rospy.init_node('compass', anonymous=True)
    rate = rospy.Rate(60) # Hz

    # connect to the AirSim simulator 
    client = airsim.CarClient()
    client.confirmConnection()

    while not rospy.is_shutdown():
        # Get the compass data from the car
        magData = client.getMagnetometerData()
        compassAngle = getCompassAngle(magData.magnetic_field_body.x_val, magData.magnetic_field_body.y_val)
        
        #rospy.loginfo(magData)
        rospy.loginfo(compassAngle)
        pub.publish(compassAngle)
              
        rate.sleep()


# THIS FUNCTION HAD THE POLARITY BACKWARDS!!!!!
# https://cdn-shop.adafruit.com/datasheets/AN203_Compass_Heading_Using_Magnetometers.pdf
# Direction (y>0) = 90 - [arcTAN(x/y)]*180/pi
# Direction (y<0) = 270 - [arcTAN(x/y)]*180/pi
# Direction (y=0, x<0) = 180.0
# Direction (y=0, x>0) = 0.0  
#def getCompassAngle(x, y):
#    if y > 0:
#        return  90 - (arctan(x/y)) * 180 / pi   # Direction (y>0) = 90 - [arcTAN(x/y)]*180/pi
#    elif y < 0:
#        return 270 - (arctan(x/y)) * 180 / pi   # Direction (y<0) = 270 - [arcTAN(x/y)]*180/pi
#    elif ((y == 0) and (x < 0)):    # Direction (y=0, x<0) = 180.0
#        return 180.0
#    else:
#        return 0.0  # Direction (y=0, x>0) = 0.0
    
# THIS COMES FROM AVIONICS NAVIGATION SYSTEMS SECOND EDITION p. 443, (9.17)
# Assumes that x = H_forward.
# Avionics Navigation Systems, Second Edition
# Editor(s): Myron Kayton Ph.D., P.E., Walter R. Fried M.S.,
# First published:18 April 1997
# Print ISBN:9780471547952 |Online ISBN:9780470172704 |DOI:10.1002/9780470172704
# Copyright 1997 John Wiley & Sons, Inc.
def getCompassAngle(x,y):
    # https://stackoverflow.com/questions/35600583/how-do-i-convert-raw-xyz-magnetometer-data-to-a-heading
    # Gives compass angle that has the wrong sign for some reason (INVESTIGATE)
   
    compass = 0 - math.degrees(arctan2(y, x))
    
    return compass

if __name__ == '__main__':
    try:
        compass()
    except rospy.ROSInterruptException:
        pass
    
