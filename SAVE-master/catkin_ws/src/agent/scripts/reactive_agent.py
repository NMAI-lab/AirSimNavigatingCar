from task import Task
from layer import Layer
from controls.msg import Control

import rospy

from std_msgs.msg import Float64


def start_agent():
    rospy.init_node('agent', anonymous=True)
    # Create publisher to control
    controlPub = rospy.Publisher('control/control', Control, queue_size=1) # Queue size one due to hard real-time deadline

    # Create Layers
    lka_layer = Layer(10, 'lka', [('steering', Float64)])
    acc_layer = Layer(5, 'acc', [('throttle', Float64),('brake', Float64)])
    object_layer = Layer(10, 'object_avoid', [('brake', Float64)])

    # Create Tasks
    highway_driving = Task(
        controlPub,
        [
            object_layer, # highest priority
            acc_layer, 
            lka_layer
        ]
    )

    highway_driving.run()

    rospy.spin()

if __name__ == '__main__':
    start_agent()