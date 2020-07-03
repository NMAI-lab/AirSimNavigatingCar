#!/usr/bin/env python
from lka.msg import Margins
from metric import Metric

import rospy

class LaneMarginMetric(Metric):

    def __init__(self):
        super(LaneMarginMetric, self).__init__(100)
        rospy.Subscriber('lka/margins', Margins, self.add)

    def add(self, data):
        self.addElement(data.margin_diff)