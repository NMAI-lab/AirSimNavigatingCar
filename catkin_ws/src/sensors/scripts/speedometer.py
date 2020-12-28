#!/usr/bin/env python

import setup_path 
import airsim

import rospy
from sensors.msg import Speed
from std_msgs.msg import Float64

def speedometer():
    pub = rospy.Publisher('sensor/speed', Float64, queue_size=1)
    rospy.init_node('speedometer', anonymous=True)
    rate = rospy.Rate(60) # 2 Hz

    # connect to the AirSim simulator 
    client = airsim.CarClient()
    client.confirmConnection()

    while not rospy.is_shutdown():
        # Get the speed of the car
        car_speed = client.getCarState().speed
        rospy.loginfo(car_speed)
        pub.publish(car_speed)
        rate.sleep()

if __name__ == '__main__':
    try:
        speedometer()
    except rospy.ROSInterruptException:
        pass