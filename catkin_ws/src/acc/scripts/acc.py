#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

import time
import math

import matplotlib.pyplot as plt


class ACC:
    """
    Automated cruise control class
    Controls the speed of the car 
    """

    def __init__(self, speed):
        rospy.init_node('acc', anonymous=True)

        # set speed for ACC algorithm
        self.max_speed = speed
        self.set_speed = speed
        self.throttle = 0
        self.brake = 0
        self.throttlePub = rospy.Publisher('acc/throttle', Float64, queue_size=1)
        self.brakePub = rospy.Publisher('acc/brake', Float64, queue_size=1)

        self.integral = 0
        self.derivative = 0
        self.last_time = time.time()
        self.last_error = 0
        
        # TODO fine tune the pid controller
        self.Kp = 1.7
        self.Ki = 0.001
        self.Kd = 1.0

        # for tunning pid
        """
        self.plot_xaxis=[]
        self.count = 0
        self.plot_speed = []
        self.plot_output = []
        self.plot_set_speed = []
        """
        return

    """
    Bounds the input between 0 and 1 using the function {1-e^(-x)}
    """
    def _bound(self, inp):
        out = 1 - math.exp(-1 * inp)
        return out

    """
    Controls the speed of the car and keeps it around the control speed.    
    """
    def update_speed(self, speed_data):
        speed = speed_data.data
        rospy.loginfo('Speed: {}'.format(speed))

        # If we are stuck, then don't continue to increase integral variable
        if speed == 0:
            self.integral = 0

        dt = time.time() - self.last_time

        error = self.set_speed - speed

        self.integral = self.integral + (error * dt)

        self.derivative = (error - self.last_error)/dt

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * self.derivative)

        rospy.loginfo('Output: {}'.format(output))

        self.last_error = error
        self.last_time  = time.time()

        # add information to the graph for tuning pid
        """
        self.plot_xaxis.append(self.count)
        self.count = self.count + 1
        self.plot_output.append(output)
        self.plot_speed.append(speed)
        self.plot_set_speed.append(self.set_speed)
        

        if self.count > 200 :
            self.plot_graph()
        """

        if output > 0:
            self.throttle = self._bound(output)
            self.brake = 0.0
        elif output < 0:
            self.throttle = 0.0 
            self.brake = self._bound(-1 * output)
        else:
            self.throttle = 0.0
            self.brake = 0.0
            
        # If speed is zero, hit the brakes
        if self.max_speed <= 0.0:
            self.throttle = 0.0
            self.brake = 1.0

        rospy.loginfo('Brake: {} Throttle: {}'.format(self.brake, self.throttle))

        self.throttlePub.publish(self.throttle)
        self.brakePub.publish(self.brake)

    #def adjust_for_turn_lka(self, steering_data):
    #    steering = abs(steering_data.data)
    #    # Simple conditional for now. Needs to be updated: TODO
    #    if steering > 1:
    #        self.set_speed = 5.0 # 5 m/s -> take turn slow
    #    elif steering > 0.5:
    #        self.set_speed = 8.0
    #    elif steering > 0.3:
    #        self.set_speed = self.max_speed - 5.0
    #    else:
    #        self.set_speed = self.max_speed
            
    def adjust_for_turn_corner(self, steering_data):
        steering = abs(steering_data.data)
        # Simple conditional for now. Needs to be updated: TODO
        if steering >= 1:
            self.set_speed = self.max_speed * 0.15
        elif steering > 0.5:
            self.set_speed = self.max_speed * 0.3
        elif steering > 0.3:
            self.set_speed = self.max_speed * 0.5
        else:
            self.set_speed = self.max_speed
        
    def updateSetSpeed(self, speed):
        speed = speed.data
        self.max_speed = speed
        self.set_speed = speed
        rospy.loginfo('Updating speed set point: {}'.format(speed))
        
    def control_speed(self):
        rospy.Subscriber('sensor/speed', Float64, self.update_speed)
        #rospy.Subscriber('lka/steering', Float64, self.adjust_for_turn_lka)
        rospy.Subscriber('action/steering', Float64, self.adjust_for_turn_corner)
        rospy.Subscriber('action/setSpeed', Float64, self.updateSetSpeed)
        rospy.spin()

    """
    Function for plotting speed, output and set_speed
    Used for tuning the pid 
    """
    """
    def plot_graph(self):
        plt.plot(self.plot_xaxis, self.plot_speed, 'r' )
        plt.plot(self.plot_xaxis, self.plot_output, 'g')
        plt.plot(self.plot_xaxis, self.plot_set_speed, 'b')
        plt.show()

        return 
    """

# test ACC
acc = ACC(0)#8 # start with the car stopped.
acc.control_speed()