#!/usr/bin/env python
from layer import Layer
from action import Action
from controls.msg import Control

import rospy

class Task:

    def __init__(self, controls_pub, layers = []):
        self.layers = layers # Ordered series of layers, first layer in list is fundamental
        self.running = False
        self.action = {}
        self.prev_action = {}
        self.controls = controls_pub # Each action type key maps to a publisher to control vehicle 

    def add_layer(self, layer):
        if isinstance(layer, Layer):
            self.layers.append(layer)

    def run(self):
        # Obtain the highest frequency polling layer to determine loop period
        freq = 0
        for layer in self.layers:
            if freq == 0:
                freq = layer.frequency
            elif (layer.frequency > freq):
                freq = layer.frequency
        
        # If there is a valid polling rate
        if freq > 0:
            r = rospy.Rate(freq)
            self.running = True
            # Start running the task
            while self.running:
                # Update the task action with each layers preferred action
                for i in range(len(self.layers) - 1, -1, -1):
                    self.action.update(self.layers[i].getAction())

                rospy.loginfo(self.action)

                # Create a control message and populate it with the action
                control = Control()
                for action_type in self.action:
                    if action_type == "throttle":
                        control.throttle = self.action[action_type]
                    elif action_type == "brake":
                        control.brake = self.action[action_type]
                    elif action_type == "steering":
                        control.steering = self.action[action_type]
                    
                # Publish the control message to the vehicle's control process
                self.controls.publish(control)
                self.prev_action = self.action
                
                # Wait for next cycle to update the vehicle's action
                r.sleep()
