#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

import time

import matplotlib.pyplot as plt

class Distance:
    LOW = 1
    MEDIUM = 2
    HIGH = 3


class ACC:
    """
    Automated cruise control class
    Controls the speed of the car 
    """

    def __init__(self, distance, speed):
        rospy.init_node('acc', anonymous=True)

        # set distance and speed for ACC algorithm
        self.min_distance = distance
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
    Controls the speed of the car and keeps it around the control speed.    
    """
    def update_speed(self, speed_data):
        speed = speed_data.data
        rospy.loginfo('Speed: {}'.format(speed))

        dt = time.time() - self.last_time

        error = self.set_speed - speed

        self.integral = self.integral + (error * dt)

        self.derivative = (error - self.last_error)/dt

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * self.derivative)

        rospy.loginfo('Output: {}'.format(output))

        self.last_error = error

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
            self.throttle = output
            self.brake = 0.0
        elif output < 0:
            self.throttle = 0.0 
            self.brake = -1 * output 
        else:
            self.throttle = 0.0
            self.brake = 0.0

        rospy.loginfo('Brake: {} Throttle: {}'.format(self.brake, self.throttle))

        self.throttlePub.publish(self.throttle)
        self.brakePub.publish(self.brake)

    def adjust_for_turn(self, steering_data):
        steering = abs(steering_data.data)
        # Simple conditional for now. Needs to be updated: TODO
        if steering > 1:
            self.set_speed = 5.0 # 5 m/s -> take turn slow
        elif steering > 0.4:
            self.set_speed = 8.0
        elif steering > 0.2:
            self.set_speed = self.max_speed - 5.0
        else:
            self.set_speed = self.max_speed
        
    def control_speed(self):
        rospy.Subscriber('sensor/speed', Float64, self.update_speed)
        rospy.Subscriber('control/steering', Float64, self.adjust_for_turn)
        rospy.spin()

    """
    Controls the distance of the car from the leading vehicle. Keeps the distance 
    above the set distance by updating the speed of the vehicle.  
    """

    def control_distance(self):
        distance = 0

        print("Distance: \n" + str(distance))
        return

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

    # """
    # Updates the set_speed of the car
    # """

    # def update_speed(self, speed):
    #     if speed > self.max_speed:
    #         self.set_speed = self.max_speed
    #     elif speed < 0:
    #         self.set_speed = 0
    #     else:
    #         self.set_speed = speed

    #     return

    # """
    # Updates the distance of the car
    # """

    # def update_distance(self, distance):
    #     if distance in Distance:
    #         self.min_distance = distance
    #     else:
    #         print("Input distance is not a valid option! Setting to moderate distance.")
    #         self.min_distance = Distance.MEDIUM

    #     return


# test ACC
acc = ACC(Distance.MEDIUM, 8)
acc.control_speed()