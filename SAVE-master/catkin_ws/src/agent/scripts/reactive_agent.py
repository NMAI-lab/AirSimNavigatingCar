from task import Task
from layer import Layer

import rospy

from std_msgs.msg import Float64


def start_agent():
    rospy.init_node('agent', anonymous=True)
    # Create publishers to control
    steeringPub = rospy.Publisher('control/steering', Float64, queue_size=1) # Queue size one due to hard real-time deadline
    throttlePub = rospy.Publisher('control/throttle', Float64, queue_size=1)
    brakePub    = rospy.Publisher('control/brake', Float64, queue_size=1)

    # Create Layers
    lka_layer = Layer(10, 'lka', [('steering', Float64)])
    acc_layer = Layer(5, 'acc', [('throttle', Float64),('brake', Float64)])

    # Create Tasks
    highway_driving = Task(
        {
            'steering' : steeringPub,
            'throttle' : throttlePub,
            'brake'    : brakePub
        },
        [
            acc_layer, # highest priority
            lka_layer
        ]
    )

    highway_driving.run()

    rospy.spin()

if __name__ == '__main__':
    start_agent()