#!/usr/bin/env python

import setup_path 
import airsim

import rospy
from sensors.msg import Speed
from std_msgs.msg import Float64

# Connect to the AirSim simulator and get controls object
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True) # Enable API controls
car_controls = airsim.CarControls()

def brake(data):
    val = data.data
    rospy.loginfo('brake: {}'.format(val))
    car_controls.brake = val
    client.setCarControls(car_controls)

def throttle(data):
    val = data.data
    rospy.loginfo('throttle: {}'.format(val))
    car_controls.throttle = val
    client.setCarControls(car_controls)

def controlListener():
    rospy.init_node('control', anonymous=True)

    # Subscribe to the different types of controls
    rospy.Subscriber('control/brake', Float64, brake)
    rospy.Subscriber('control/throttle', Float64, throttle)

    # Repeat the listener
    rospy.spin()

if __name__ == '__main__':
    controlListener()