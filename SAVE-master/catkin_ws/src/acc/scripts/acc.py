#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

class Distance:
    LOW = 1
    MEDIUM = 2
    HIGH = 3


class ACC:
    """
    """

    def __init__(self, distance, speed):
        rospy.init_node('acc', anonymous=True)

        # set distance and speed for ACC algorithm
        self.min_distance = distance
        self.max_speed = speed
        self.set_speed = speed
        self.throttle = 0
        self.brake = 0
        self.throttlePub = rospy.Publisher('control/throttle', Float64, queue_size=5)
        self.brakePub = rospy.Publisher('control/brake', Float64, queue_size=5)
        return

    """
    Controls the speed of the car and keeps it around the control speed.    
    """
    def update_speed(self, speed_data):
        speed = speed_data.data
        rospy.loginfo('Speed: {}'.format(speed))

        if speed < (self.set_speed - 5):
            self.throttle = 1.0
            self.brake = 0
        elif speed < self.set_speed:
            self.throttle = 0.5
            self.brake = 0
        elif speed > (self.set_speed + 5):
            self.throttle = 0.0
            self.brake = 1
        else:
            self.throttle = 0.0
            self.brake = 0

        self.throttlePub.publish(self.throttle)
        self.brakePub.publish(self.brake)

    def control_speed(self):
        rospy.Subscriber('sensor/speed', Float64, self.update_speed)
        rospy.spin()

    """
    Controls the distance of the car from the leading vehicle. Keeps the distance 
    above the set distance by updating the speed of the vehicle.  
    """

    def control_distance(self):
        distance = 0

        print("Distance: \n" + str(distance))
        return

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
acc = ACC(Distance.MEDIUM, 15)

while True:
    acc.control_speed()