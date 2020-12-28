#!/usr/bin/env python
from std_msgs.msg import Float64
from threading import Semaphore

import rospy
import airsim


def setup():
    # Connect to the AirSim simulator and get controls object
    global carControls, controlSem, simulator
    simulator = airsim.CarClient()
    simulator.confirmConnection()
    simulator.enableApiControl(True) # Enable API controls
    carControls = airsim.CarControls()
    controlSem = Semaphore()


def updateSteering(data):
    rospy.loginfo(data)
    global carControls, controlSem, simulator
    controlSem.acquire()
    carControls.steering = data.data
    simulator.setCarControls(carControls)
    controlSem.release()    
    
def updateBrake(data):
    rospy.loginfo(data)
    global carControls, controlSem, simulator
    controlSem.acquire()
    carControls.brake = data.data
    simulator.setCarControls(carControls)
    controlSem.release() 
      
def updateThrottle(data):
    rospy.loginfo(data)
    global carControls, controlSem, simulator
    controlSem.acquire()
    carControls.throttle = data.data
    simulator.setCarControls(carControls) 
    controlSem.release() 


def controlListener():
    rospy.init_node('control', anonymous=True)

    setup()

    # Subscribe to the different types of controls
    rospy.Subscriber('action/steering', Float64, updateSteering)
    rospy.Subscriber('acc/throttle', Float64, updateThrottle)
    rospy.Subscriber('acc/brake', Float64, updateBrake)

    # Repeat the listener
    rospy.spin()


if __name__ == '__main__':
    controlListener()