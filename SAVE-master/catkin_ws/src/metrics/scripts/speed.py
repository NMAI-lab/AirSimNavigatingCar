#!/usr/bin/env python
from std_msgs.msg import Float64

import rospy

from metric import Metric

class SpeedMetric(Metric):

    def __init__(self):
        super(SpeedMetric, self).__init__(100)
        rospy.Subscriber('sensor/speed', Float64, self.add)

    def add(self, data):
        self.addElement(data.data)