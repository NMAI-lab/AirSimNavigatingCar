#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64#, String


# Main execution
def rosMain():
    
    rospy.init_node('turnTester', anonymous=True)

    rospy.loginfo('Try turning')
    
    #steeringAngle = 0.69    # Max steering angle in RAD
    
    while not rospy.is_shutdown():
        steeringPub = rospy.Publisher('lka/steering', Float64, queue_size=1)    # Hack into the steering angle (for now)
        speedPublisher = rospy.Publisher('action/setSpeed', Float64, queue_size=1)

        steeringPub.publish(Float64(-0.69))
        speedPublisher.publish(Float64(1.0))



# Start things up
if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
