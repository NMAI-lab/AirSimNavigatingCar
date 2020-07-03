#!/usr/bin/env python
from controls.msg import Control
from threading import Semaphore

import rospy
import airsim

# Connect to the AirSim simulator and get controls object
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True) # Enable API controls
car_controls = airsim.CarControls()
control_sem = Semaphore()

def update_controls(controls):
    global car_controls
    rospy.loginfo(controls)
    car_controls.brake = controls.brake
    car_controls.throttle = controls.throttle
    car_controls.steering = controls.steering
    client.setCarControls(car_controls)

def controlListener():
    rospy.init_node('control', anonymous=True)

    # Subscribe to the different types of controls
    rospy.Subscriber('control/control', Control, update_controls)

    # Repeat the listener
    rospy.spin()

if __name__ == '__main__':
    controlListener()