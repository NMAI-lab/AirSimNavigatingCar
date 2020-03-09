#!/usr/bin/env python
from layer import Layer
from action import Action

import time

import rospy

class Task:

    def __init__(self, controls_dict, layers = []):
        self.layers = layers # Ordered series of layers, first layer in list is fundamental
        self.running = False
        self.action = {}
        self.prev_action = {}
        self.controls = controls_dict # Each action type key maps to a publisher to control vehicle 

    def add_layer(self, layer):
        if isinstance(layer, Layer):
            self.layers.append(layer)

    def run(self):
        freq = 0
        for layer in self.layers:
            if freq == 0:
                freq = layer.frequency
            elif (layer.frequency > freq):
                freq = layer.frequency
            
        if freq > 0:
            self.running = True
            # Start running the task
            while self.running:
                # Update the task actions with each layer
                for i in range(len(self.layers) - 1, -1, -1):
                    # rospy.loginfo(i)
                    self.action.update(self.layers[i].getAction())

                rospy.loginfo(self.action)

                # if self.action != self.prev_action:
                # If action differs from previous, then publish the actions to their respective controls
                for action_type in self.action:
                    rospy.loginfo(action_type)
                    rospy.loginfo(self.action[action_type])
                    rospy.loginfo('end...')
                    self.controls[action_type].publish(self.action[action_type])
                self.prev_action = self.action
                
                time.sleep(1/freq)
