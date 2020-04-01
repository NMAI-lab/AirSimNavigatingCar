#!/usr/bin/env python
from action import Action
from functools import partial

from std_msgs.msg import Empty

import rospy

class Layer:

    def __init__(self, frequency, node_name, action_types):
        self.action = Action()
        self.frequency = frequency # Should be the frequency of the most updated action
        # Layer listens for certain actions from a specific node
        for action_type in action_types: # An action type is a tuple with the first element being the name and the second the msg type
            rospy.Subscriber(node_name + '/' + action_type[0], action_type[1], partial(self.setAction, action_type[0]))

        # Clear action in layer
        rospy.Subscriber(node_name + '/clear', Empty, self.clearAction)

    def setAction(self, action_type, data):
        self.action.add(action_type, data.data)

    def getAction(self):
        return self.action.get()

    def clearAction(self, data):
        rospy.loginfo('Attempting clear')
        self.action.clear()
    