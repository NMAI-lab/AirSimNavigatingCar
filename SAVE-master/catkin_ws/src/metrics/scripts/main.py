#!/usr/bin/env python

import rospy
import time

from speed import SpeedMetric
from lane_margin import LaneMarginMetric

def run():
    running = True

    rospy.init_node('metrics', anonymous=True)
    speed_m  = SpeedMetric()
    margin_m = LaneMarginMetric()
    rospy.loginfo("Metrics collection started")

    while running:
        rospy.loginfo('Speed ===========')
        rospy.loginfo(speed_m.getStats())
        rospy.loginfo('Lane Margin ===========')
        rospy.loginfo(margin_m.getStats())
        time.sleep(1)

    rospy.spin()


if __name__ == "__main__":
    run()